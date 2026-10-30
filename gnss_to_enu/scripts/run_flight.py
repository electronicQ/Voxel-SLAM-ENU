#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""One-shot sensor startup, mapping, save, and ENU export workflow."""

from __future__ import annotations

import argparse
import csv
import glob
import json
import os
import re
import selectors
import shlex
import shutil
import signal
import subprocess
import sys
import time
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import List, Optional, Sequence, Tuple


FINAL_MARKER = "Final trajectory written:"
ENU_META_FILENAME = "map_enu.meta.yaml"
RUN_ID_PATTERN = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_-]*$")
SAFE_PATH_PATTERN = re.compile(r"^[A-Za-z0-9_./@+\-]+$")


class FlightError(RuntimeError):
    pass


@dataclass
class ProcessHandle:
    name: str
    process: subprocess.Popen
    log_path: Path


def parse_args(argv: Optional[Sequence[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "启动传感器、开始一次 Voxel-SLAM/RTK 建图；Ctrl+C 请求 /save_only，"
            "保存完成后自动执行 ENU 对齐并退出。"
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--run-id",
        help="本次任务 ID，同时用于地图、RTK、输出和日志目录；默认按时间生成",
    )
    parser.add_argument(
        "--data-root",
        type=Path,
        default=Path.home() / "voxel_enu_data",
        help="所有地图、RTK 和输出数据的根目录",
    )
    parser.add_argument(
        "--setup",
        action="append",
        default=[],
        metavar="SETUP_BASH",
        help="执行 ROS 命令前要 source 的环境脚本，可重复传入",
    )
    parser.add_argument("--config", type=Path, help="传给 avia_mapping_record_rtk.launch 的 LiDAR 配置")
    parser.add_argument("--fix-topic", default="/rtk/navsatfix", help="RTK NavSatFix 话题")
    parser.add_argument("--lidar-topic", default="/livox/lidar", help="用于启动就绪检查的 LiDAR 话题")
    parser.add_argument(
        "--save-mode",
        choices=("save_only", "finish"),
        default="save_only",
        help="Ctrl+C 后请求只保存，或先执行最终全局 BA 再保存",
    )
    parser.add_argument(
        "--startup-delay",
        type=float,
        default=30.0,
        metavar="SECONDS",
        help="两个传感器 launch 启动后等待的时间",
    )
    parser.add_argument("--master-timeout", type=float, default=20.0, help="等待 ROS master 的最长时间")
    parser.add_argument("--topic-timeout", type=float, default=10.0, help="等待每个话题首条消息的最长时间")
    parser.add_argument("--save-timeout", type=float, default=1800.0, help="请求保存后的最长等待时间")
    parser.add_argument("--alignment-timeout", type=float, default=7200.0, help="对齐导出的最长时间")
    parser.add_argument(
        "--map-voxel-size",
        type=float,
        default=0.15,
        help="ENU 降采样体素尺寸；必须大于 0 以生成 output 目录",
    )
    parser.add_argument("--lever-arm", nargs=3, type=float, metavar=("X", "Y", "Z"),
                        help="IMU 到 GNSS 天线的杆臂；不传则使用 launch 默认值")
    parser.add_argument(
        "--manual-datum",
        nargs=3,
        type=float,
        metavar=("LAT", "LON", "ELLIPSOID_HEIGHT"),
        help="手动 ENU 原点；不传则使用第一条有效 RTK 自动设原点",
    )
    parser.add_argument("--xy-variance-max", type=float, default=0.01, help="RTK XY 方差门限")
    parser.add_argument("--z-variance-max", type=float, default=0.04, help="RTK Z 方差门限")
    parser.add_argument("--no-rviz", action="store_true", help="建图时不启动 RViz")
    parser.add_argument("--no-serial-permissions", action="store_true", help="不自动执行串口 chmod")
    parser.add_argument("--no-sensors", action="store_true", help="复用已经启动的传感器，不再启动 drivers")
    parser.add_argument("--skip-topic-check", action="store_true", help="跳过 LiDAR/RTK 话题首条消息检查")
    parser.add_argument("--keep-sensors", action="store_true", help="脚本结束时保留由本脚本启动的传感器")
    parser.add_argument("--lidar-package", default="livox_ros_driver", help="LiDAR 驱动 ROS 包名")
    parser.add_argument("--lidar-launch-file", default="livox_lidar_msg.launch", help="LiDAR launch 文件名")
    parser.add_argument("--gnss-package", default="beiyun_gnss", help="GNSS 驱动 ROS 包名")
    parser.add_argument("--gnss-launch-file", default="beiyun_gnss.launch", help="GNSS launch 文件名")
    parser.add_argument(
        "--mapping-arg",
        action="append",
        default=[],
        metavar="NAME:=VALUE",
        help="附加建图 launch 参数，可重复传入",
    )
    parser.add_argument(
        "--alignment-arg",
        action="append",
        default=[],
        metavar="NAME:=VALUE",
        help="附加对齐 launch 参数，可重复传入",
    )
    parser.add_argument("--dry-run", action="store_true", help="只打印解析后的流程与关键命令，不启动任何进程")
    return parser.parse_args(argv)


def validate_launch_args(values: Sequence[str], option_name: str) -> None:
    for value in values:
        if ":=" not in value or value.startswith(":="):
            raise FlightError(f"{option_name} 参数必须使用 NAME:=VALUE 格式: {value!r}")


def safe_absolute_path(value: Path, label: str) -> Path:
    expanded = value.expanduser()
    absolute = expanded.resolve(strict=False)
    text = str(absolute)
    if not SAFE_PATH_PATTERN.fullmatch(text):
        raise FlightError(
            f"{label} 含有空格或 Voxel-SLAM 不支持的 shell 特殊字符，"
            f"请换用仅包含字母、数字、_、-、.、/ 的路径: {text}"
        )
    return absolute


def resolve_setup_scripts(explicit: Sequence[str]) -> List[Path]:
    if explicit:
        paths = [Path(item).expanduser().resolve(strict=False) for item in explicit]
        missing = [str(path) for path in paths if not path.is_file()]
        if missing:
            raise FlightError("以下 setup 脚本不存在: " + ", ".join(missing))
        return paths

    candidates = [
        Path("/opt/ros/noetic/setup.bash"),
        Path.home() / "voxel_enu_ws" / "devel" / "setup.bash",
        Path.home() / "voxel_enu_ws" / "install" / "setup.bash",
    ]
    script_path = Path(__file__).resolve()
    if len(script_path.parents) > 3:
        workspace = script_path.parents[3]
        candidates.extend([
            workspace / "devel" / "setup.bash",
            workspace / "install" / "setup.bash",
        ])

    result: List[Path] = []
    for candidate in candidates:
        resolved = candidate.resolve(strict=False)
        if resolved.is_file() and resolved not in result:
            result.append(resolved)
    return result


class RosRunner:
    def __init__(self, setup_scripts: Sequence[Path]):
        self.setup_scripts = list(setup_scripts)
        source_lines = [
            f"source {shlex.quote(str(setup_script))}"
            for setup_script in self.setup_scripts
        ]
        self.bootstrap = "set -e\n" + "\n".join(source_lines) + '\nexec "$@"'

    def command(self, args: Sequence[str]) -> List[str]:
        return ["bash", "-c", self.bootstrap, "run-flight"] + [str(arg) for arg in args]

    def popen_stream(
        self,
        args: Sequence[str],
        name: str,
    ) -> Tuple[subprocess.Popen, List[str]]:
        command = self.command(args)
        process = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            start_new_session=True,
            env=os.environ.copy(),
        )
        return process, command

    def popen_logged(self, args: Sequence[str], log_path: Path) -> subprocess.Popen:
        with log_path.open("ab", buffering=0) as stream:
            process = subprocess.Popen(
                self.command(args),
                stdin=subprocess.DEVNULL,
                stdout=stream,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                env=os.environ.copy(),
            )
        return process

    def run_quiet(
        self,
        args: Sequence[str],
        timeout: float,
        check_stdout: bool = False,
    ) -> Tuple[int, str, str]:
        process = subprocess.Popen(
            self.command(args),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE if check_stdout else subprocess.DEVNULL,
            stderr=subprocess.PIPE,
            text=True,
            start_new_session=True,
            env=os.environ.copy(),
        )
        try:
            stdout, stderr = process.communicate(timeout=timeout)
        except subprocess.TimeoutExpired:
            terminate_process_group(process, timeout=2.0)
            stdout, stderr = process.communicate()
            return 124, stdout or "", stderr or ""
        return process.returncode, stdout or "", stderr or ""

    def require_commands(self, commands: Sequence[str]) -> None:
        shell_check = " && ".join(f"command -v {shlex.quote(command)} >/dev/null" for command in commands)
        returncode, _, stderr = self.run_quiet(["bash", "-c", shell_check], timeout=10.0)
        if returncode != 0:
            detail = stderr.strip()
            hint = "；请用 --setup 指定 ROS 工作空间的 devel/setup.bash 或 install/setup.bash"
            raise FlightError("ROS 命令不可用" + (f": {detail}" if detail else "") + hint)


def terminate_process_group(process: Optional[subprocess.Popen], timeout: float = 10.0) -> None:
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=timeout)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGTERM)
    except ProcessLookupError:
        return
    try:
        process.wait(timeout=5.0)
        return
    except subprocess.TimeoutExpired:
        pass
    try:
        os.killpg(process.pid, signal.SIGKILL)
    except ProcessLookupError:
        return
    process.wait(timeout=5.0)


class FlightSession:
    def __init__(self, args: argparse.Namespace, runner: RosRunner):
        self.args = args
        self.runner = runner
        self.phase = "init"
        self.sigint_received = False
        self.sigterm_received = False
        self.save_requested = False
        self.save_sent = False
        self.sensors: List[ProcessHandle] = []
        self.mapping: Optional[ProcessHandle] = None
        self.alignment: Optional[ProcessHandle] = None

        run_id = args.run_id or datetime.now().strftime("flight_%Y%m%d_%H%M%S")
        if not RUN_ID_PATTERN.fullmatch(run_id) or ".." in run_id:
            raise FlightError(
                "--run-id 只能包含字母、数字、下划线和连字符，且必须以字母或数字开头: "
                + repr(run_id)
            )

        self.run_id = run_id
        self.data_root = safe_absolute_path(args.data_root, "--data-root")
        self.maps_root = safe_absolute_path(self.data_root / "maps", "地图父目录")
        self.session_dir = safe_absolute_path(self.maps_root / run_id, "建图 session 目录")
        self.record_dir = safe_absolute_path(self.data_root / f"rtk_{run_id}", "RTK 记录目录")
        self.output_dir = safe_absolute_path(self.data_root / f"enu_{run_id}", "ENU 输出目录")
        self.publish_dir = safe_absolute_path(self.data_root / "output", "发布输出目录")
        self.log_dir = safe_absolute_path(self.data_root / "logs" / run_id, "日志目录")

    def install_signal_handlers(self) -> None:
        signal.signal(signal.SIGINT, self._handle_sigint)
        signal.signal(signal.SIGTERM, self._handle_sigterm)

    def _handle_sigint(self, _signum, _frame) -> None:
        self.sigint_received = True
        if self.phase == "mapping" and not self.save_requested:
            self.save_requested = True
            self.phase = "saving"
            print("\n收到 Ctrl+C：请求保存建图结果，等待后端队列完成。请继续等待。", flush=True)
        elif self.phase == "saving":
            print("建图正在保存，请继续等待保存完成的日志。", flush=True)

    def _handle_sigterm(self, _signum, _frame) -> None:
        self.sigterm_received = True

    def check_abort(self) -> None:
        if self.sigterm_received:
            raise FlightError("收到 SIGTERM，任务已中止")
        if self.sigint_received and self.phase != "saving":
            raise FlightError("收到 Ctrl+C，任务已中止")

    def interruptible_sleep(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.check_abort()
            time.sleep(min(0.2, max(0.0, deadline - time.monotonic())))

    def ensure_unique_directories(self) -> None:
        occupied = [
            path
            for path in (self.session_dir, self.record_dir, self.output_dir, self.log_dir)
            if path.exists()
        ]
        if occupied:
            listing = "\n".join(f"  {path}" for path in occupied)
            raise FlightError(
                "本次 run-id 的目录已经存在。Voxel-SLAM 和 RTK 记录均要求新目录；"
                "请换一个 --run-id：\n" + listing
            )

    def prepare_directories(self) -> None:
        self.ensure_unique_directories()
        self.maps_root.mkdir(parents=True, exist_ok=True)
        self.log_dir.mkdir(parents=True, exist_ok=False)
        metadata = {
            "run_id": self.run_id,
            "started_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "data_root": str(self.data_root),
            "session_dir": str(self.session_dir),
            "record_dir": str(self.record_dir),
            "output_dir": str(self.output_dir),
            "publish_dir": str(self.publish_dir),
            "save_mode": self.args.save_mode,
            "fix_topic": self.args.fix_topic,
            "lidar_topic": self.args.lidar_topic,
        }
        (self.log_dir / "run_info.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )

    def grant_serial_permissions(self) -> None:
        if self.args.no_serial_permissions:
            print("跳过串口权限设置。", flush=True)
            return
        devices = sorted({
            path
            for pattern in ("/dev/ttyACM*", "/dev/ttyUSB*")
            for path in glob.glob(pattern)
            if os.path.exists(path)
        })
        if not devices:
            print("警告：没有发现 /dev/ttyACM* 或 /dev/ttyUSB* 串口设备。", flush=True)
            return

        inaccessible = [
            device
            for device in devices
            if not os.access(device, os.R_OK | os.W_OK)
        ]
        if not inaccessible:
            print("串口权限已满足：" + ", ".join(devices), flush=True)
            return

        sudo = shutil.which("sudo")
        if os.geteuid() == 0:
            command = ["chmod", "666", *inaccessible]
        elif sudo:
            command = [sudo, "chmod", "666", *inaccessible]
        else:
            raise FlightError("串口无读写权限，且找不到 sudo：" + ", ".join(inaccessible))

        print("设置串口读写权限：" + ", ".join(inaccessible), flush=True)
        result = subprocess.run(command)
        if result.returncode != 0:
            raise FlightError("串口权限设置失败")
        still_inaccessible = [
            device
            for device in devices
            if not os.access(device, os.R_OK | os.W_OK)
        ]
        if still_inaccessible:
            raise FlightError("串口仍无读写权限：" + ", ".join(still_inaccessible))

    def launch_sensor(self, name: str, package: str, launch_file: str) -> ProcessHandle:
        log_path = self.log_dir / f"{name}.launch.log"
        process = self.runner.popen_logged(["roslaunch", package, launch_file], log_path)
        handle = ProcessHandle(name=name, process=process, log_path=log_path)
        self.sensors.append(handle)
        print(f"已启动 {name}，日志：{log_path}", flush=True)
        return handle

    def ensure_sensor_alive(self) -> None:
        for handle in self.sensors:
            returncode = handle.process.poll()
            if returncode is not None:
                raise FlightError(
                    f"{handle.name} launch 已退出，返回码 {returncode}；请检查 {handle.log_path}"
                )

    def wait_for_ros_master(self) -> None:
        deadline = time.monotonic() + self.args.master_timeout
        while time.monotonic() < deadline:
            self.check_abort()
            self.ensure_sensor_alive()
            returncode, _, _ = self.runner.run_quiet(["rosnode", "list"], timeout=2.0)
            if returncode == 0:
                print("ROS master 已就绪。", flush=True)
                return
            time.sleep(0.5)
        raise FlightError(
            f"等待 ROS master 超过 {self.args.master_timeout:g} 秒；请检查传感器 launch 日志"
        )

    def start_sensors(self) -> None:
        if self.args.no_sensors:
            print("按参数复用已有传感器，不启动 LiDAR/GNSS drivers。", flush=True)
            return

        self.launch_sensor("lidar", self.args.lidar_package, self.args.lidar_launch_file)
        self.wait_for_ros_master()
        self.launch_sensor("gnss", self.args.gnss_package, self.args.gnss_launch_file)

        if self.args.startup_delay > 0:
            print(f"等待传感器初始化 {self.args.startup_delay:g} 秒...", flush=True)
            self.interruptible_sleep(self.args.startup_delay)
        self.ensure_sensor_alive()

    def check_topic(self, topic: str) -> None:
        returncode, _, stderr = self.runner.run_quiet(
            ["rostopic", "echo", "-n", "1", topic],
            timeout=self.args.topic_timeout,
            check_stdout=False,
        )
        if returncode != 0:
            detail = stderr.strip()
            suffix = f"；{detail}" if detail else ""
            raise FlightError(
                f"话题 {topic} 在 {self.args.topic_timeout:g} 秒内没有收到消息{suffix}"
            )
        print(f"话题已收到数据：{topic}", flush=True)

    def check_sensor_topics(self) -> None:
        if self.args.skip_topic_check:
            print("按参数跳过传感器话题就绪检查。", flush=True)
            return
        self.check_topic(self.args.lidar_topic)
        self.check_topic(self.args.fix_topic)

    def mapping_command(self) -> List[str]:
        save_path = str(self.maps_root) + "/"
        command = [
            "roslaunch",
            "gnss_to_enu",
            "avia_mapping_record_rtk.launch",
            f"save_path:={save_path}",
            f"bagname:={self.run_id}",
            f"record_dir:={self.record_dir}",
            f"fix_topic:={self.args.fix_topic}",
            "use_sim_time:=false",
            f"rviz:={'false' if self.args.no_rviz else 'true'}",
            f"xy_variance_max:={self.args.xy_variance_max:g}",
            f"z_variance_max:={self.args.z_variance_max:g}",
        ]
        if self.args.config is not None:
            command.append(f"config:={safe_absolute_path(self.args.config, '--config')}")
        if self.args.manual_datum is not None:
            lat, lon, altitude = self.args.manual_datum
            command.extend([
                "use_auto_datum:=false",
                f"datum_lat:={lat:.12g}",
                f"datum_lon:={lon:.12g}",
                f"datum_alt:={altitude:.12g}",
            ])
        command.extend(self.args.mapping_arg)
        return command

    def alignment_command(self) -> List[str]:
        command = [
            "roslaunch",
            "gnss_to_enu",
            "align_voxel_enu.launch",
            f"session:={self.session_dir}",
            f"rtk:={self.record_dir / 'rtk_enu.csv'}",
            f"output:={self.output_dir}",
            f"publish_dir:={self.publish_dir}",
            f"map_voxel_size:={self.args.map_voxel_size:g}",
            "alignment_only:=false",
        ]
        if self.args.lever_arm is not None:
            x, y, z = self.args.lever_arm
            command.extend([
                f"lever_x:={x:.12g}",
                f"lever_y:={y:.12g}",
                f"lever_z:={z:.12g}",
            ])
        command.extend(self.args.alignment_arg)
        return command

    def start_mapping(self) -> None:
        log_path = self.log_dir / "mapping.launch.log"
        # Mark the phase before Popen so a signal arriving immediately after
        # spawn is still interpreted as a save request.
        self.phase = "mapping"
        process, command = self.runner.popen_stream(self.mapping_command(), "mapping")
        self.mapping = ProcessHandle(name="mapping", process=process, log_path=log_path)
        print("\n建图已启动。按 Ctrl+C 请求保存；脚本会自动等待保存完成并导出。", flush=True)
        print("建图命令：" + shlex.join(command), flush=True)

    def request_save(self) -> None:
        if self.save_sent:
            return
        request_param = self.args.save_mode
        print(f"发送保存请求：rosparam set /{request_param} true", flush=True)
        returncode, _, stderr = self.runner.run_quiet(
            ["rosparam", "set", f"/{request_param}", "true"],
            timeout=10.0,
        )
        if returncode != 0:
            detail = stderr.strip()
            raise FlightError("发送保存请求失败" + (f": {detail}" if detail else ""))
        self.save_sent = True

    def stream_launch(
        self,
        handle: ProcessHandle,
        timeout: Optional[float],
        stop_after_marker: bool = False,
    ) -> bool:
        assert handle.process.stdout is not None
        selector = selectors.DefaultSelector()
        selector.register(handle.process.stdout, selectors.EVENT_READ)
        os.set_blocking(handle.process.stdout.fileno(), False)
        pending = ""
        marker_seen = False
        save_deadline: Optional[float] = None
        overall_deadline = time.monotonic() + timeout if timeout is not None else None

        with handle.log_path.open("w", encoding="utf-8", buffering=1) as log_stream:
            try:
                while True:
                    if self.sigterm_received:
                        raise FlightError("收到 SIGTERM，任务已中止")
                    if self.sigint_received and self.phase != "saving":
                        raise FlightError("收到 Ctrl+C，任务已中止")
                    if self.phase == "saving" and not self.save_sent:
                        self.request_save()
                        save_deadline = time.monotonic() + self.args.save_timeout

                    events = selector.select(timeout=0.2)
                    for _key, _mask in events:
                        while True:
                            try:
                                chunk = os.read(handle.process.stdout.fileno(), 65536)
                            except BlockingIOError:
                                break
                            if not chunk:
                                break
                            text = chunk.decode("utf-8", errors="replace")
                            log_stream.write(text)
                            log_stream.flush()
                            pending += text
                            while "\n" in pending:
                                line, pending = pending.split("\n", 1)
                                print(line, flush=True)
                                if FINAL_MARKER in line:
                                    marker_seen = True
                            if len(chunk) < 65536:
                                break

                    if handle.process.poll() is not None:
                        # Drain output that was written between the last select
                        # event and process exit.
                        while True:
                            try:
                                chunk = os.read(handle.process.stdout.fileno(), 65536)
                            except BlockingIOError:
                                break
                            if not chunk:
                                break
                            text = chunk.decode("utf-8", errors="replace")
                            log_stream.write(text)
                            log_stream.flush()
                            pending += text
                        break
                    if marker_seen and stop_after_marker:
                        break
                    now = time.monotonic()
                    if save_deadline is not None and now >= save_deadline and not marker_seen:
                        raise FlightError(
                            f"等待 {FINAL_MARKER!r} 超时（{self.args.save_timeout:g} 秒）；"
                            f"请检查 {handle.log_path}"
                        )
                    if overall_deadline is not None and now >= overall_deadline:
                        raise FlightError(
                            f"{handle.name} 超过 {timeout:g} 秒仍未结束；请检查 {handle.log_path}"
                        )

                if pending:
                    log_stream.write(pending + "\n")
                    print(pending, flush=True)
                    if FINAL_MARKER in pending:
                        marker_seen = True
                return marker_seen
            finally:
                selector.close()
                handle.process.stdout.close()

    def finish_mapping(self) -> None:
        if self.mapping is None:
            raise FlightError("建图进程未启动")
        marker_seen = self.stream_launch(
            self.mapping,
            timeout=None,
            stop_after_marker=True,
        )
        if not marker_seen:
            raise FlightError(
                f"建图 launch 已退出，但日志中没有 {FINAL_MARKER!r}；"
                f"未执行对齐，请检查 {self.mapping.log_path}"
            )

        self.phase = "saved"
        print("轨迹保存日志已确认，正在停止建图 launch。", flush=True)
        terminate_process_group(self.mapping.process, timeout=20.0)
        self.mapping = None

        trajectory = self.session_dir / "alidarState.txt"
        rtk_csv = self.record_dir / "rtk_enu.csv"
        if not trajectory.is_file() or trajectory.stat().st_size == 0:
            raise FlightError(f"未找到有效轨迹文件：{trajectory}")
        pcd_count = len(list(self.session_dir.glob("*.pcd")))
        if pcd_count == 0:
            raise FlightError(f"未找到逐帧 PCD：{self.session_dir}/*.pcd")
        if not rtk_csv.is_file() or rtk_csv.stat().st_size == 0:
            raise FlightError(f"未找到 RTK 记录：{rtk_csv}")
        with rtk_csv.open(newline="", encoding="utf-8") as stream:
            rows = csv.reader(stream)
            next(rows, None)
            if next(rows, None) is None:
                raise FlightError(f"RTK 记录中没有有效测量数据：{rtk_csv}")
        datum_path = self.record_dir / ENU_META_FILENAME
        if not datum_path.is_file() or datum_path.stat().st_size == 0:
            raise FlightError(f"未找到 ENU 原点元数据：{datum_path}")
        print(
            f"保存结果已校验：{pcd_count} 个 PCD，轨迹 {trajectory}",
            flush=True,
        )

    def stop_sensors(self) -> None:
        if self.args.keep_sensors:
            print("按参数保留由本脚本启动的传感器。", flush=True)
            self.sensors = []
            return
        for handle in reversed(self.sensors):
            print(f"停止 {handle.name} launch。", flush=True)
            terminate_process_group(handle.process, timeout=10.0)
        self.sensors = []

    def run_alignment(self) -> None:
        self.check_abort()
        if self.output_dir.exists():
            raise FlightError(f"ENU 输出目录已存在，对齐工具要求新目录：{self.output_dir}")
        log_path = self.log_dir / "alignment.launch.log"
        process, command = self.runner.popen_stream(self.alignment_command(), "alignment")
        self.alignment = ProcessHandle(name="alignment", process=process, log_path=log_path)
        self.phase = "alignment"
        print("\n开始 ENU 对齐和地图导出。", flush=True)
        print("对齐命令：" + shlex.join(command), flush=True)
        self.stream_launch(
            self.alignment,
            timeout=self.args.alignment_timeout,
            stop_after_marker=False,
        )
        returncode = self.alignment.process.returncode
        self.alignment = None
        if returncode != 0:
            raise FlightError(f"对齐/导出失败，返回码 {returncode}；请检查 {log_path}")

        report_path = self.output_dir / "alignment.json"
        map_path = self.output_dir / "map_enu.pcd"
        if not report_path.is_file():
            raise FlightError(f"对齐结束但没有生成报告：{report_path}")
        if not map_path.is_file() or map_path.stat().st_size == 0:
            raise FlightError(f"对齐结束但没有生成有效 ENU 地图：{map_path}")
        try:
            report = json.loads(report_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise FlightError(f"无法读取对齐报告 {report_path}: {exc}") from exc

        status = report.get("status", "unknown")
        warnings = report.get("warnings") or []
        print(f"\nENU 导出完成，状态：{status}", flush=True)
        if warnings:
            print("对齐警告：", flush=True)
            for warning in warnings:
                print(f"  - {warning}", flush=True)
        if status not in ("complete", "complete_with_warnings"):
            raise FlightError(f"对齐报告状态不是成功完成：{status}")
        print(f"完整地图：{map_path}", flush=True)
        downsampled = self.output_dir / "map_enu_downsampled.pcd"
        published_map = self.publish_dir / "map_enu_downsampled.pcd"
        published_meta = self.publish_dir / ENU_META_FILENAME
        if not downsampled.is_file() or downsampled.stat().st_size == 0:
            raise FlightError(f"对齐结束但没有生成有效降采样地图：{downsampled}")
        if not published_map.is_file() or published_map.stat().st_size == 0:
            raise FlightError(f"未生成发布地图：{published_map}")
        if not published_meta.is_file() or published_meta.stat().st_size == 0:
            raise FlightError(f"未生成发布原点文件：{published_meta}")
        print(f"降采样地图：{downsampled}", flush=True)
        print(f"发布地图：{published_map}", flush=True)
        print(f"发布原点：{published_meta}", flush=True)
        print(f"对齐报告：{report_path}", flush=True)

    def close_processes(self) -> None:
        if self.mapping is not None:
            terminate_process_group(self.mapping.process, timeout=10.0)
            self.mapping = None
        if self.alignment is not None:
            terminate_process_group(self.alignment.process, timeout=10.0)
            self.alignment = None
        self.stop_sensors()

    def dry_run(self) -> None:
        payload = {
            "run_id": self.run_id,
            "data_root": str(self.data_root),
            "setup_scripts": [str(path) for path in self.runner.setup_scripts],
            "session_dir": str(self.session_dir),
            "record_dir": str(self.record_dir),
            "output_dir": str(self.output_dir),
            "publish_dir": str(self.publish_dir),
            "log_dir": str(self.log_dir),
            "sensor_commands": [] if self.args.no_sensors else [
                ["roslaunch", self.args.lidar_package, self.args.lidar_launch_file],
                ["roslaunch", self.args.gnss_package, self.args.gnss_launch_file],
            ],
            "mapping_command": self.mapping_command(),
            "ctrl_c_request": f"rosparam set /{self.args.save_mode} true",
            "alignment_command": self.alignment_command(),
        }
        print(json.dumps(payload, ensure_ascii=False, indent=2))

    def run(self) -> None:
        if self.args.dry_run:
            self.dry_run()
            return

        self.install_signal_handlers()
        self.runner.require_commands(("roslaunch", "rosparam", "rosnode", "rostopic"))
        self.ensure_unique_directories()
        self.grant_serial_permissions()
        self.check_abort()
        self.prepare_directories()
        try:
            if not self.args.no_sensors:
                self.start_sensors()
            self.check_sensor_topics()
            self.check_abort()
            self.start_mapping()
            self.finish_mapping()
            self.stop_sensors()
            self.run_alignment()
            self.phase = "complete"
            print("\n全部流程完成，脚本即将退出。", flush=True)
        finally:
            self.close_processes()


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = parse_args(argv)
    try:
        validate_launch_args(args.mapping_arg, "--mapping-arg")
        validate_launch_args(args.alignment_arg, "--alignment-arg")
        if args.startup_delay < 0:
            raise FlightError("--startup-delay 不能小于 0")
        if args.topic_timeout <= 0 or args.save_timeout <= 0 or args.alignment_timeout <= 0:
            raise FlightError("超时参数必须大于 0")
        if args.map_voxel_size <= 0:
            raise FlightError("--map-voxel-size 必须大于 0，以便生成 output 目录")
        if args.manual_datum is not None:
            lat, lon, _altitude = args.manual_datum
            if not -90 <= lat <= 90 or not -180 <= lon <= 180:
                raise FlightError("手动 datum 的纬经度超出有效范围")

        setup_scripts = resolve_setup_scripts(args.setup)
        runner = RosRunner(setup_scripts)
        session = FlightSession(args, runner)
        session.run()
        return 0
    except FlightError as exc:
        print(f"任务失败：{exc}", file=sys.stderr)
        return 1
    except KeyboardInterrupt:
        print("\n任务被中断。", file=sys.stderr)
        return 130


if __name__ == "__main__":
    sys.exit(main())
