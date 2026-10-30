"""Offline checks for the one-shot flight workflow manager."""

import contextlib
import importlib.util
import io
import json
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "run_flight.py"
spec = importlib.util.spec_from_file_location("run_flight", SCRIPT)
run_flight = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = run_flight
spec.loader.exec_module(run_flight)


class RunFlightTests(unittest.TestCase):
    def make_session(self, root, *extra):
        args = run_flight.parse_args([
            "--run-id", "flight01",
            "--data-root", root,
            "--dry-run",
            *extra,
        ])
        return run_flight.FlightSession(args, run_flight.RosRunner([]))

    def test_dry_run_builds_consistent_paths_and_commands(self):
        with tempfile.TemporaryDirectory() as tmp:
            session = self.make_session(
                tmp,
                "--map-voxel-size", "0.2",
                "--manual-datum", "22", "113", "10",
                "--lever-arm", "0.1", "-0.2", "0.3",
            )
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                session.run()
            payload = json.loads(output.getvalue())
            self.assertEqual(
                payload["session_dir"],
                str(Path(tmp) / "maps" / "flight01"),
            )
            self.assertEqual(
                payload["record_dir"],
                str(Path(tmp) / "rtk_flight01"),
            )
            self.assertEqual(
                payload["output_dir"],
                str(Path(tmp) / "enu_flight01"),
            )
            self.assertEqual(
                payload["publish_dir"],
                str(Path(tmp) / "output"),
            )
            self.assertIn("use_auto_datum:=false", payload["mapping_command"])
            self.assertIn("map_voxel_size:=0.2", payload["alignment_command"])
            self.assertIn(
                f"publish_dir:={Path(tmp) / 'output'}",
                payload["alignment_command"],
            )
            self.assertEqual(payload["ctrl_c_request"], "rosparam set /save_only true")

    def test_ctrl_c_during_mapping_becomes_save_request(self):
        with tempfile.TemporaryDirectory() as tmp:
            session = self.make_session(tmp)
            session.phase = "mapping"
            session._handle_sigint(None, None)
            self.assertTrue(session.sigint_received)
            self.assertTrue(session.save_requested)
            self.assertEqual(session.phase, "saving")
            self.assertFalse(session.sigterm_received)

    def test_existing_run_directory_is_rejected(self):
        with tempfile.TemporaryDirectory() as tmp:
            session = self.make_session(tmp)
            session.session_dir.mkdir(parents=True)
            with self.assertRaises(run_flight.FlightError):
                session.prepare_directories()

    def test_unsafe_voxel_path_is_rejected(self):
        with self.assertRaises(run_flight.FlightError):
            run_flight.safe_absolute_path(
                Path("/tmp/flight data"),
                "test",
            )

    def test_save_marker_is_streamed_and_written_to_log(self):
        with tempfile.TemporaryDirectory() as tmp:
            session = self.make_session(tmp)
            process = subprocess.Popen(
                [
                    sys.executable,
                    "-c",
                    "import time; print('Final trajectory written: ok', flush=True); time.sleep(1)",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            handle = run_flight.ProcessHandle(
                name="mapping",
                process=process,
                log_path=Path(tmp) / "mapping.log",
            )
            session.phase = "saving"
            session.save_sent = True
            output = io.StringIO()
            with contextlib.redirect_stdout(output):
                seen = session.stream_launch(
                    handle,
                    timeout=5.0,
                    stop_after_marker=True,
                )
            run_flight.terminate_process_group(process, timeout=2.0)
            self.assertTrue(seen)
            self.assertIn(run_flight.FINAL_MARKER, output.getvalue())
            self.assertIn(
                run_flight.FINAL_MARKER,
                handle.log_path.read_text(encoding="utf-8"),
            )


if __name__ == "__main__":
    unittest.main()
