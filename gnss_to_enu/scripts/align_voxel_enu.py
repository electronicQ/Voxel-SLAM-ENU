#!/usr/bin/env python3
"""Offline georeferencing of final Voxel-SLAM IMU poses/PCDs. No ROS required."""
import argparse
import csv
import json
from pathlib import Path
import sys
import numpy as np


def quaternion_matrix(q):
    q = np.asarray(q, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1e-12:
        raise ValueError('Invalid quaternion')
    x, y, z, w = q / norm
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def slerp(q0, q1, u):
    q0, q1 = q0 / np.linalg.norm(q0), q1 / np.linalg.norm(q1)
    dot = float(q0 @ q1)
    if dot < 0:
        q1, dot = -q1, -dot
    dot = np.clip(dot, -1., 1.)
    if dot > 0.9995:
        q = (1-u)*q0 + u*q1
        return q / np.linalg.norm(q)
    theta = np.arccos(dot)
    return (np.sin((1-u)*theta)*q0 + np.sin(u*theta)*q1) / np.sin(theta)


def read_poses(path):
    # alidarState.txt: t px py pz qx qy qz qw [v bg ba g information...]
    poses = np.atleast_2d(np.loadtxt(path))[:, :8]
    if poses.shape[1] != 8 or len(poses) < 2 or not np.isfinite(poses).all():
        raise ValueError('Trajectory needs >=2 finite t,x,y,z,qx,qy,qz,qw rows')
    if np.any(np.diff(poses[:, 0]) <= 0):
        raise ValueError('Final trajectory timestamps must be strictly increasing; do not sort frame IDs')
    norms = np.linalg.norm(poses[:, 4:8], axis=1)
    if np.any(norms < 1e-12):
        raise ValueError('Zero quaternion in trajectory')
    poses[:, 4:8] /= norms[:, None]
    return poses


def interpolate(poses, t, max_gap):
    j = np.searchsorted(poses[:, 0], t)
    if j < len(poses) and poses[j, 0] == t:
        return poses[j, 1:4], quaternion_matrix(poses[j, 4:8])
    if j == 0 or j == len(poses):
        return None
    dt = poses[j, 0] - poses[j-1, 0]
    if dt > max_gap:
        return None
    u = (t-poses[j-1, 0]) / dt
    return ((1-u)*poses[j-1, 1:4] + u*poses[j, 1:4],
            quaternion_matrix(slerp(poses[j-1, 4:8], poses[j, 4:8], u)))


def fit_rigid(source, target, weights=None):
    if len(source) < 3:
        raise ValueError('Need >=3 corresponding points')
    weights = np.ones(len(source)) if weights is None else weights
    weights = weights / weights.sum()
    mx, my = weights @ source, weights @ target
    x, y = source-mx, target-my
    # Non-collinear planar flight is valid. A straight line cannot determine full rotation.
    for cloud in (x, y):
        s = np.linalg.svd(cloud*np.sqrt(weights[:, None]), compute_uv=False)
        if s[0] < 1e-8 or s[1] < max(1e-6, s[0]*1e-3):
            raise ValueError('Degenerate trajectory: collect turns and spatial coverage')
    u, _, vt = np.linalg.svd((x*weights[:, None]).T @ y)
    correction = np.eye(3)
    correction[2, 2] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ correction @ u.T
    return rotation, my-rotation@mx


class AlignmentFitError(ValueError):
    """Rejected fit; retain its candidate exclusively for failure diagnostics."""
    def __init__(self, message, candidate=None):
        super().__init__(message)
        self.candidate = candidate


def robust_fit(source, target, weights, threshold, trials, min_pairs, min_ratio):
    if len(source) < min_pairs:
        raise AlignmentFitError('Too few spatially sampled correspondences')
    rng = np.random.RandomState(0)
    best, best_score = None, (-1, -np.inf)
    best_rt = None
    # Include all-point SVD, then deterministic RANSAC for RTK outliers.
    for trial in range(trials+1):
        ids = np.arange(len(source)) if trial == 0 else rng.choice(len(source), 3, replace=False)
        try:
            r, t = fit_rigid(source[ids], target[ids], weights[ids])
        except ValueError:
            continue
        errors = np.linalg.norm(source@r.T+t-target, axis=1)
        mask = errors <= threshold
        score = (int(mask.sum()), -float(np.minimum(errors, threshold).sum()))
        if score > best_score:
            best, best_score = mask, score
            best_rt = (r, t)
    if best is None:
        raise AlignmentFitError('Cannot find a non-degenerate alignment')
    r, t = best_rt
    for _ in range(10):
        if best.sum() < min_pairs or best.mean() < min_ratio:
            raise AlignmentFitError('Insufficient alignment inliers; check sync, lever arm and map drift', (r, t))
        try:
            r, t = fit_rigid(source[best], target[best], weights[best])
        except ValueError as exc:
            raise AlignmentFitError(str(exc), (r, t)) from exc
        mask = np.linalg.norm(source@r.T+t-target, axis=1) <= threshold
        if np.array_equal(mask, best):
            return r, t, best
        best = mask
    raise AlignmentFitError('Inlier classification did not converge', (r, t))


def pcd_header(stream):
    header = {}
    while True:
        line = stream.readline()
        if not line:
            raise ValueError('Incomplete PCD header')
        parts = line.decode('ascii').strip().split()
        if not parts or parts[0].startswith('#'):
            continue
        header[parts[0].upper()] = parts[1:]
        if parts[0].upper() == 'DATA':
            break
    return header


def read_pcd_xyz(path):
    with open(path, 'rb') as stream:
        h = pcd_header(stream)
        fields = h['FIELDS']
        sizes = list(map(int, h['SIZE']))
        types = h['TYPE']
        counts = list(map(int, h.get('COUNT', ['1']*len(fields))))
        if not (len(fields) == len(sizes) == len(types) == len(counts)):
            raise ValueError('Invalid PCD field layout: '+str(path))
        n = int(h['POINTS'][0])
        mode = h['DATA'][0]
        if mode == 'binary':
            names, formats = [], []
            for i, (name, size, typ, count) in enumerate(zip(fields, sizes, types, counts)):
                names.append(name if name != '_' else '_padding'+str(i))
                scalar = np.dtype('<'+{'F':'f', 'I':'i', 'U':'u'}[typ]+str(size))
                formats.append(scalar if count == 1 else (scalar, (count,)))
            dtype = np.dtype({'names': names, 'formats': formats})
            data = np.fromfile(stream, dtype=dtype, count=n)
            if len(data) != n:
                raise ValueError('Truncated PCD: '+str(path))
            xyz = np.column_stack([data[f] for f in ('x', 'y', 'z')])
        elif mode == 'ascii':
            data = np.atleast_2d(np.loadtxt(stream))
            offsets = np.cumsum([0]+counts[:-1]).tolist()
            xyz = data[:, [offsets[fields.index(f)] for f in ('x', 'y', 'z')]]
            if len(xyz) != n:
                raise ValueError('PCD point count mismatch: '+str(path))
        else:
            raise ValueError('Only binary/ascii PCD supported (Voxel-SLAM saves binary): '+str(path))
    if xyz.shape != (n, 3) or not np.isfinite(xyz).all():
        raise ValueError('Invalid XYZ in '+str(path))
    return xyz


def export_map(scan_dir, poses, rotation, translation, output):
    # Frame i.pcd stores IMU-frame points; no LiDAR extrinsic is applied here.
    paths = [scan_dir/(str(i)+'.pcd') for i in range(len(poses))]
    for path in paths:
        if not path.is_file():
            raise ValueError('Missing frame '+str(path)+'; use complete saved session')
    payload = output/'map_payload.tmp'
    total = 0
    try:
        with payload.open('wb') as stream:
            for path, pose in zip(paths, poses):
                xyz = read_pcd_xyz(path)
                rm = quaternion_matrix(pose[4:8])
                xyz = xyz @ (rotation@rm).T + rotation@pose[1:4] + translation
                np.asarray(xyz, dtype='<f4').tofile(stream)
                total += len(xyz)
        with (output/'map_enu.pcd').open('wb') as stream:
            stream.write(('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
                          'COUNT 1 1 1\nWIDTH %d\nHEIGHT 1\n'
                          'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n' % (total,total)).encode('ascii'))
            with payload.open('rb') as source:
                import shutil
                shutil.copyfileobj(source, stream, 1024*1024)
    finally:
        payload.unlink(missing_ok=True)
    return total


def trajectory_diagnostics(output, source, target, times, selected, threshold,
                           raw_source, raw_target, raw_map_times, raw_rtk_times,
                           candidate=None, method='none', failure=None):
    """Plot every usable matched pair, including outliers; never validate a transform."""
    import matplotlib
    matplotlib.use('Agg')  # Works on a headless ROS computer, no display or plt.show().
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection

    blue, orange = '#0072B2', '#D55E00'
    fig, axes = plt.subplots(2, 2, figsize=(15, 11), constrained_layout=True)
    title = 'ALIGNMENT FAILED - diagnostic only' if failure else 'Alignment passed quality checks'
    fig.suptitle(title+'\n'+(failure or 'All time-matched antenna positions; no outliers hidden'), fontsize=13)

    def tracks(ax, x, y, centered=False):
        for points, color, label in ((x, blue, 'SLAM antenna'), (y, orange, 'RTK antenna')):
            if not len(points):
                continue
            points = points-points[0] if centered else points
            ax.plot(points[:, 0], points[:, 1], color=color, linewidth=.65, alpha=.65)
            if label == 'SLAM antenna':
                ax.scatter(points[:, 0], points[:, 1], edgecolors=color, facecolors='none',
                           s=16, linewidths=.7, label=label)
            else:
                ax.scatter(points[:, 0], points[:, 1], color=color, marker='x',
                           s=12, linewidths=.65, label=label)
            ax.scatter(*points[0, :2], marker='o', s=65, facecolors='none', edgecolors=color)
            ax.scatter(*points[-1, :2], marker='*', s=105, color=color)
        ax.set_aspect('equal', adjustable='datalim')
        ax.grid(True, alpha=.25)
        if len(x) or len(y):
            ax.legend(loc='best', fontsize=9)

    # When there is no temporal overlap, still show the complete input trajectories.
    x = source if len(source) else raw_source
    y = target if len(target) else raw_target
    tracks(axes[0, 0], x, y, centered=True)
    axes[0, 0].set(title='Before alignment: starts translated to zero (rotation unchanged)',
                   xlabel='Map X / ENU East relative to own start [m]',
                   ylabel='Map Y / ENU North relative to own start [m]')
    axes[0, 0].text(.02, .02, 'Circle = start; star = end. Different coordinate axes before alignment.',
                     transform=axes[0, 0].transAxes, fontsize=8,
                     bbox={'facecolor':'white', 'edgecolor':'none', 'alpha':.85})

    residuals, errors = None, None
    if candidate is not None and len(source):
        r, t = candidate
        transformed = source@r.T+t
        residuals = transformed-target
        errors = np.linalg.norm(residuals, axis=1)
        tracks(axes[0, 1], transformed, target)
        # Keep ALL scatter points. Thin only connectors to prevent an unreadable web.
        ids = np.unique(np.linspace(0, len(source)-1, min(120, len(source)), dtype=int))
        segments = np.stack([transformed[ids, :2], target[ids, :2]], axis=1)
        axes[0, 1].add_collection(LineCollection(segments, colors='#777777', linewidths=.6, alpha=.55))
        axes[0, 1].set(title='After transform: '+method+' (gray = timestamp pairs)',
                       xlabel='East [m]', ylabel='North [m]')
        elapsed = np.asarray(times)-times[0]
        axes[1, 0].plot(elapsed, transformed[:, 2], '.-', color=blue, markersize=3, linewidth=.7, label='SLAM antenna')
        axes[1, 0].plot(elapsed, target[:, 2], '.-', color=orange, markersize=3, linewidth=.7, label='RTK antenna')
        axes[1, 0].set(title='Height after transform', xlabel='Time since first matched RTK [s]', ylabel='Up [m]')
        for j, name in enumerate(('dEast', 'dNorth', 'dUp')):
            axes[1, 1].plot(elapsed, residuals[:, j], linewidth=.8, label=name)
        axes[1, 1].plot(elapsed, errors, color='black', linewidth=1, label='3D distance')
        axes[1, 1].axhline(threshold, color='red', linestyle='--', linewidth=1, label='Inlier threshold')
        axes[1, 1].set(title='Residual = transformed SLAM - RTK (all matched pairs)',
                       xlabel='Time since first matched RTK [s]', ylabel='Residual [m]')
        axes[0, 1].text(.02, .02, 'Matched: %d; fit samples: %d; 3D RMSE (all): %.3f m' %
                        (len(source), len(selected), np.sqrt(np.mean(errors**2))),
                        transform=axes[0, 1].transAxes, fontsize=8,
                        bbox={'facecolor':'white', 'edgecolor':'none', 'alpha':.85})
    else:
        axes[0, 1].text(.5, .5, 'No usable rigid transform\nNo ENU-aligned trajectory available',
                         ha='center', va='center', transform=axes[0, 1].transAxes)
        axes[0, 1].set_title('Alignment unavailable')
        for points, stamps, color, label in ((raw_source, raw_map_times, blue, 'SLAM antenna'),
                                             (raw_target, raw_rtk_times, orange, 'RTK antenna')):
            if len(points):
                axes[1, 0].plot(np.asarray(stamps)-raw_map_times[0], points[:, 2]-points[0, 2],
                                '.-', color=color, markersize=3, linewidth=.7, label=label)
        axes[1, 0].set(title='Input heights relative to own start (different coordinate axes)',
                       xlabel='Input timestamp minus first SLAM timestamp [s]', ylabel='Relative height [m]')
        axes[1, 1].text(.5, .5, 'No residuals without a usable transform\nMatched pairs: %d' % len(source),
                         ha='center', va='center', transform=axes[1, 1].transAxes)
        axes[1, 1].set_title('Residual unavailable')
    for ax in axes[1]:
        ax.grid(True, alpha=.25)
        if ax.lines:
            ax.legend(loc='best', fontsize=8)
    try:
        fig.savefig(output/'trajectory_comparison.png', dpi=160)
    finally:
        plt.close(fig)

    diagnostic = {'status':'failed' if failure else 'passed', 'failure_reason':failure,
                  'comparison':'time-matched antenna positions; all matched points included',
                  'candidate_method':method, 'candidate_validated':not bool(failure) and candidate is not None,
                  'matched':len(source), 'fit_points':len(selected), 'connector_limit':120,
                  'candidate_T_enu_map':None}
    if candidate is not None:
        transform = np.eye(4)
        transform[:3, :3], transform[:3, 3] = candidate
        diagnostic['candidate_T_enu_map'] = transform.tolist()
    (output/'diagnostic.json').write_text(json.dumps(diagnostic, indent=2, allow_nan=False)+'\n')
    selected_set = set(selected.tolist())
    with (output/'matched_pairs.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['timestamp','map_x','map_y','map_z','rtk_e','rtk_n','rtk_u','fit_sample'])
        for i, (stamp, p, q) in enumerate(zip(times, source, target)):
            writer.writerow([format(stamp,'.9f'), *p, *q, int(i in selected_set)])
    if errors is not None:
        with (output/'residuals.csv').open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(['timestamp','dx','dy','dz','norm','fit_sample','inlier'])
            for i, (stamp, residual, err) in enumerate(zip(times, residuals, errors)):
                writer.writerow([format(stamp,'.9f'), *residual, err, int(i in selected_set), int(err <= threshold)])
    print('Trajectory diagnostic image: '+str((output/'trajectory_comparison.png').resolve()))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True, type=Path, help='Voxel-SLAM session directory with alidarState.txt and i.pcd')
    parser.add_argument('--rtk', required=True, type=Path, help='rtk_enu.csv recorded by gnss_to_enu')
    parser.add_argument('--datum', type=Path, help='Defaults to datum.json beside RTK CSV')
    parser.add_argument('--output', required=True, type=Path, help='New output directory; existing paths are refused')
    parser.add_argument('--lever-arm', nargs=3, type=float, required=True, metavar=('X','Y','Z'), help='IMU origin -> antenna, expressed in IMU axes, metres')
    parser.add_argument('--time-offset', type=float, default=0., help='t_map = t_RTK + offset, seconds')
    parser.add_argument('--max-gap', type=float, default=.25, help='Maximum trajectory interpolation interval, seconds')
    parser.add_argument('--sample-distance', type=float, default=1., help='3D RTK displacement between fitting points, metres')
    parser.add_argument('--inlier-threshold', type=float, default=.5, help='RANSAC Euclidean residual gate, metres')
    parser.add_argument('--min-pairs', type=int, default=10)
    parser.add_argument('--min-inlier-ratio', type=float, default=.7)
    parser.add_argument('--max-rmse', type=float, default=.3, help='Maximum inlier 3D RMSE; failing produces no export')
    parser.add_argument('--ransac-trials', type=int, default=500)
    parser.add_argument('--alignment-only', action='store_true', help='Export alignment diagnostics without reading PCDs')
    args = parser.parse_args(argv)
    if (args.max_gap <= 0 or args.sample_distance < 0 or args.inlier_threshold <= 0 or
            args.min_pairs < 3 or not 0 < args.min_inlier_ratio <= 1 or args.max_rmse <= 0 or args.ransac_trials < 0):
        raise ValueError('Invalid alignment thresholds')
    if not np.isfinite(args.lever_arm + [args.time_offset, args.max_gap, args.sample_distance,
                                       args.inlier_threshold, args.max_rmse]).all():
        raise ValueError('Non-finite parameters')
    if args.output.exists():
        raise ValueError('Output already exists; select a new directory')
    poses = read_poses(args.session/'alidarState.txt')
    datum_path = args.datum or args.rtk.parent/'datum.json'
    datum = json.loads(datum_path.read_text())
    for key in ('latitude', 'longitude', 'altitude'):
        if not np.isfinite(float(datum[key])):
            raise ValueError('Invalid datum')
    source, target, times, weights = [], [], [], []
    raw_target, raw_rtk_times = [], []
    excluded = 0
    last_stamp = -np.inf
    with args.rtk.open() as stream:
        for row in csv.DictReader(stream):
            stamp = float(row['timestamp'])
            y = np.array([float(row[k]) for k in ('east','north','up')])
            var = np.array([float(row[k]) for k in ('var_e','var_n','var_u')])
            if (not np.isfinite(np.r_[stamp,y,var]).all() or np.any(var <= 0) or stamp <= last_stamp):
                excluded += 1
                continue
            last_stamp = stamp
            raw_target.append(y)
            raw_rtk_times.append(stamp)
            pose = interpolate(poses, stamp+args.time_offset, args.max_gap)
            if pose is None:
                excluded += 1
                continue
            p, r = pose
            source.append(p+r@np.asarray(args.lever_arm))
            target.append(y)
            times.append(stamp)
            # Scalar inverse mean variance; no claim of full anisotropic covariance weighting.
            weights.append(1./max(float(var.mean()), 1e-6))
    source = np.asarray(source).reshape(-1, 3)
    target = np.asarray(target).reshape(-1, 3)
    weights = np.asarray(weights)
    raw_target = np.asarray(raw_target).reshape(-1, 3)
    raw_source = np.array([pose[1:4]+quaternion_matrix(pose[4:8])@args.lever_arm for pose in poses])
    selected, last = [], None
    for i, point in enumerate(target):
        if last is None or np.linalg.norm(point-last) >= args.sample_distance:
            selected.append(i)
            last = point
    selected = np.asarray(selected, dtype=int)
    args.output.mkdir(parents=True)

    def diagnostics(candidate=None, method='none', failure=None):
        trajectory_diagnostics(args.output, source, target, times, selected, args.inlier_threshold,
                               raw_source, raw_target, poses[:, 0], raw_rtk_times,
                               candidate, method, failure)

    def failed_report(reason, metrics=None):
        (args.output/'alignment.json').write_text(json.dumps({
            'status':'alignment_failed', 'failure_reason':reason, 'T_enu_map':None,
            'datum':datum, 'matched':len(source), 'excluded':excluded,
            'metrics':metrics, 'diagnostic':'diagnostic.json', 'plot':'trajectory_comparison.png'
        }, indent=2, allow_nan=False)+'\n')

    try:
        if len(source) < args.min_pairs:
            raise AlignmentFitError('Too few time-matched RTK points')
        r, t, inliers = robust_fit(source[selected], target[selected], weights[selected],
                                  args.inlier_threshold, args.ransac_trials, args.min_pairs, args.min_inlier_ratio)
    except ValueError as exc:
        candidate = getattr(exc, 'candidate', None)
        method = 'rejected RANSAC candidate' if candidate is not None else 'none'
        if candidate is None:
            try:
                candidate = fit_rigid(source, target, weights)
                method = 'all-pair SVD, visualization only'
            except ValueError:
                pass
        failed_report(str(exc))
        diagnostics(candidate, method, str(exc))
        raise
    residuals = source@r.T+t-target
    errors = np.linalg.norm(residuals, axis=1)
    accepted = errors <= args.inlier_threshold
    rmse = float(np.sqrt(np.mean(errors[selected][inliers]**2)))
    all_rmse = float(np.sqrt(np.mean(errors**2)))
    stats = {'matched':len(source), 'excluded':excluded, 'fit_points':len(selected),
             'fit_inliers':int(inliers.sum()), 'fit_inlier_rmse_m':rmse,
             'all_rmse_m':all_rmse, 'all_median_m':float(np.median(errors)),
             'all_p95_m':float(np.percentile(errors,95)), 'all_max_m':float(errors.max()),
             'all_inlier_ratio':float(accepted.mean())}
    if rmse > args.max_rmse or accepted.mean() < args.min_inlier_ratio:
        reason = 'Alignment failed quality gate: '+json.dumps(stats)
        failed_report(reason, stats)
        diagnostics((r, t), 'rejected final fit', 'Alignment failed quality gate')
        raise ValueError(reason)
    diagnostics((r, t), 'validated fit')
    transform = np.eye(4)
    transform[:3,:3], transform[:3,3] = r, t
    report = {'status':'alignment_validated', 'datum':datum, 'T_enu_map':transform.tolist(),
              'convention':'p_enu = R_enu_map @ p_map + t_enu_map; scale=1',
              'lever_arm_imu_m':args.lever_arm, 'time_offset_s':args.time_offset,
              'interpolation':'linear position / quaternion SLERP; no extrapolation',
              'weighting':'inverse mean ENU variance (scalar)',
              'parameters':{k:v for k,v in vars(args).items() if isinstance(v,(float,int,bool))},
              'inputs':{'session':str(args.session.resolve()), 'rtk':str(args.rtk.resolve())}, 'metrics':stats}
    def save_report():
        (args.output/'alignment.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    save_report()
    np.savetxt(args.output/'T_enu_map.txt', transform, fmt='%.12g')
    with (args.output/'residuals.csv').open('w') as stream:
        writer = csv.writer(stream)
        writer.writerow(['timestamp','dx','dy','dz','norm','fit_sample','inlier'])
        selected_set = set(selected.tolist())
        for i, (stamp, residual, err) in enumerate(zip(times,residuals,errors)):
            writer.writerow([format(stamp,'.9f'), *residual, err, int(i in selected_set), int(accepted[i])])
    # Export TUM poses for every final frame, retaining original frame order.
    # Quaternion multiplication q_enu_map * q_map_imu.
    # Stable rotation -> quaternion via symmetric eigensystem (xyzw).
    def matrix_quaternion(m):
        k = np.array([[m[0,0]-m[1,1]-m[2,2], m[0,1]+m[1,0], m[0,2]+m[2,0], m[2,1]-m[1,2]],
                      [m[0,1]+m[1,0], m[1,1]-m[0,0]-m[2,2], m[1,2]+m[2,1], m[0,2]-m[2,0]],
                      [m[0,2]+m[2,0], m[1,2]+m[2,1], m[2,2]-m[0,0]-m[1,1], m[1,0]-m[0,1]],
                      [m[2,1]-m[1,2], m[0,2]-m[2,0], m[1,0]-m[0,1], np.trace(m)]])/3.
        _, vectors = np.linalg.eigh(k)
        return vectors[:,-1]
    with (args.output/'trajectory_enu.tum').open('w') as stream:
        for pose in poses:
            q = matrix_quaternion(r@quaternion_matrix(pose[4:8]))
            p = r@pose[1:4]+t
            stream.write(' '.join(format(float(v), '.9f') for v in [pose[0],*p,*q])+'\n')
    if not args.alignment_only:
        try:
            report['map_points'] = export_map(args.session, poses, r, t, args.output)
            report['status'] = 'complete'
        except Exception:
            report['status'] = 'map_export_failed'
            (args.output/'map_enu.pcd').unlink(missing_ok=True)
            save_report()
            raise
    save_report()
    print(json.dumps(report, indent=2))


def run_ros():
    """One-shot ROS1 entry point; the numerical CLI remains usable without ROS."""
    import rospy
    rospy.init_node('align_voxel_enu')
    argv = []
    for name in ('session', 'rtk', 'output'):
        argv.extend(['--'+name, str(rospy.get_param('~'+name))])
    lever = rospy.get_param('~lever_arm')
    if not isinstance(lever, list) or len(lever) != 3:
        raise ValueError('~lever_arm must contain three IMU-frame metre values')
    argv.extend(['--lever-arm', *map(str, lever)])
    for name, default in (('time_offset', 0.), ('max_gap', .25), ('sample_distance', 1.),
                          ('inlier_threshold', .5), ('min_pairs', 10),
                          ('min_inlier_ratio', .7), ('max_rmse', .3), ('ransac_trials', 500)):
        argv.extend(['--'+name.replace('_', '-'), str(rospy.get_param('~'+name, default))])
    datum = rospy.get_param('~datum', '')
    if datum:
        argv.extend(['--datum', datum])
    alignment_only = rospy.get_param('~alignment_only', False)
    if not isinstance(alignment_only, bool):
        raise ValueError('~alignment_only must be a ROS boolean')
    if alignment_only:
        argv.append('--alignment-only')
    rospy.loginfo('Starting offline ENU alignment; SLAM trajectory saving must already be complete')
    main(argv)
    rospy.loginfo('Offline ENU alignment finished successfully')


if __name__ == '__main__':
    try:
        if '--ros-params' in sys.argv[1:]:
            run_ros()
        else:
            main()
    except (ValueError, OSError, KeyError, ImportError) as exc:
        print('ENU alignment failed: '+str(exc), file=sys.stderr)
        sys.exit(1)
