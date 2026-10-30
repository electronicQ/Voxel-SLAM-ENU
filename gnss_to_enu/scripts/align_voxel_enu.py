#!/usr/bin/env python3
"""Offline georeferencing of final Voxel-SLAM IMU poses/PCDs. No ROS required."""
import argparse
import csv
import json
import os
from pathlib import Path
import shutil
import sys
import numpy as np

ENU_META_FILENAME = 'map_enu.meta.yaml'


def read_enu_origin(path):
    """Read the compact enu_origin YAML; accept explicit legacy JSON files."""
    path = Path(path)
    if path.suffix.lower() == '.json':
        data = json.loads(path.read_text())
        data = data.get('enu_origin', data)
    else:
        data = {}
        in_origin = False
        for raw_line in path.read_text().splitlines():
            line = raw_line.split('#', 1)[0].rstrip()
            if not line.strip():
                continue
            if not line.startswith((' ', '\t')):
                in_origin = line.strip() == 'enu_origin:'
                continue
            if in_origin and ':' in line:
                key, value = line.strip().split(':', 1)
                if key in ('latitude', 'longitude', 'altitude'):
                    data[key] = value.strip().strip("'\"")
    missing = [key for key in ('latitude', 'longitude', 'altitude') if key not in data]
    if missing:
        raise ValueError('ENU origin metadata missing: '+', '.join(missing))
    origin = {key: float(data[key]) for key in ('latitude', 'longitude', 'altitude')}
    if not np.isfinite(list(origin.values())).all():
        raise ValueError('Invalid ENU origin metadata')
    return origin


def atomic_copy(source, destination):
    """Publish one file without exposing a partially copied destination."""
    source, destination = Path(source), Path(destination)
    destination.parent.mkdir(parents=True, exist_ok=True)
    temporary = destination.with_name('.'+destination.name+'.tmp')
    try:
        shutil.copy2(source, temporary)
        os.replace(temporary, destination)
    finally:
        temporary.unlink(missing_ok=True)


def publish_outputs(output, datum_path, publish_dir):
    """Expose only the deliverables needed by the downstream map loader."""
    output, datum_path, publish_dir = Path(output), Path(datum_path), Path(publish_dir)
    downsampled = output/'map_enu_downsampled.pcd'
    if not downsampled.is_file() or downsampled.stat().st_size == 0:
        raise ValueError('Published output requires a nonempty map_enu_downsampled.pcd; use map_voxel_size > 0')
    if not datum_path.is_file() or datum_path.stat().st_size == 0:
        raise ValueError('ENU origin metadata is missing: '+str(datum_path))
    published_map = publish_dir/'map_enu_downsampled.pcd'
    published_meta = publish_dir/ENU_META_FILENAME
    atomic_copy(downsampled, published_map)
    atomic_copy(datum_path, published_meta)
    return {'directory':str(publish_dir.resolve()),
            'downsampled_map':str(published_map.resolve()),
            'enu_origin':str(published_meta.resolve())}


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


def write_xyz_pcd(path, payload, count):
    """Write standard binary XYZ without loading a complete map into memory."""
    import shutil
    with path.open('wb') as stream:
        stream.write(('VERSION .7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
                      'COUNT 1 1 1\nWIDTH %d\nHEIGHT 1\n'
                      'VIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n' % (count,count)).encode('ascii'))
        with payload.open('rb') as source:
            shutil.copyfileobj(source,stream,1024*1024)


def export_map(scan_dir, poses, rotation, translation, output, map_voxel_size=.15):
    """Save full map AND global voxel centroids; disk-backed accumulation spans all frames."""
    import sqlite3
    if not np.isfinite(map_voxel_size) or map_voxel_size < 0:
        raise ValueError('map_voxel_size must be finite and nonnegative')
    paths = [scan_dir/(str(i)+'.pcd') for i in range(len(poses))]
    for path in paths:
        if not path.is_file():
            raise ValueError('Missing frame '+str(path)+'; use complete saved session')
    payload = output/'map_payload.tmp'
    reduced_payload = output/'map_downsampled_payload.tmp'
    database = output/'map_voxels.tmp.sqlite'
    connection = None
    total, reduced = 0, None
    try:
        if map_voxel_size > 0:
            connection=sqlite3.connect(str(database))
            connection.execute('PRAGMA journal_mode=OFF')
            connection.execute('PRAGMA synchronous=OFF')
            connection.execute('PRAGMA cache_size=-32768')  # ~32 MiB page cache, not whole-map storage.
            connection.execute('CREATE TABLE voxels (ix INTEGER,iy INTEGER,iz INTEGER,sx REAL,sy REAL,sz REAL,n INTEGER, PRIMARY KEY(ix,iy,iz)) WITHOUT ROWID')
        with payload.open('wb') as stream:
            for path, pose in zip(paths, poses):
                xyz = read_pcd_xyz(path)
                rm = quaternion_matrix(pose[4:8])
                xyz = xyz @ (rotation@rm).T + rotation@pose[1:4] + translation
                # Downsample exactly the coordinates written to the full float32 map.
                saved = np.asarray(xyz,dtype='<f4')
                if not np.isfinite(saved).all():
                    raise ValueError('Nonfinite/overflowing transformed map coordinates in '+str(path))
                saved.tofile(stream)
                total += len(saved)
                if connection is not None and len(saved):
                    points=saved.astype(np.float64)
                    cells=np.floor(points/map_voxel_size)
                    if not np.isfinite(cells).all() or np.any(cells < -(2.**63)) or np.any(cells >= 2.**63):
                        raise ValueError('Voxel indices exceed int64 range; increase map_voxel_size')
                    keys,inverse,counts=np.unique(cells.astype(np.int64),axis=0,return_inverse=True,return_counts=True)
                    sums=np.column_stack([np.bincount(inverse,weights=points[:,j],minlength=len(keys)) for j in range(3)])
                    # Each frame contributes sums/counts, so final centroids are point-weighted,
                    # not an average of per-frame centroids or one retained point per frame.
                    rows=((int(k[0]),int(k[1]),int(k[2]),float(v[0]),float(v[1]),float(v[2]),int(n))
                          for k,v,n in zip(keys,sums,counts))
                    connection.executemany('INSERT INTO voxels VALUES (?,?,?,?,?,?,?) '
                        'ON CONFLICT(ix,iy,iz) DO UPDATE SET sx=sx+excluded.sx,sy=sy+excluded.sy,sz=sz+excluded.sz,n=n+excluded.n',rows)
                    connection.commit()
        write_xyz_pcd(output/'map_enu.pcd',payload,total)
        if connection is not None:
            reduced=int(connection.execute('SELECT COUNT(*) FROM voxels').fetchone()[0])
            cursor=connection.execute('SELECT sx/n,sy/n,sz/n FROM voxels ORDER BY ix,iy,iz')
            with reduced_payload.open('wb') as stream:
                while True:
                    batch=cursor.fetchmany(65536)
                    if not batch:
                        break
                    np.asarray(batch,dtype='<f4').tofile(stream)
            write_xyz_pcd(output/'map_enu_downsampled.pcd',reduced_payload,reduced)
    finally:
        if connection is not None:
            connection.close()
        for temporary in (payload,reduced_payload,database):
            temporary.unlink(missing_ok=True)
    return total,reduced


def trajectory_diagnostics(output, source, target, times, selected, threshold,
                           raw_source, raw_target, raw_map_times, raw_rtk_times,
                           candidate=None, method='none', failure=None, variances=None,
                           selected_mask=None, quality_flags=None, fixed_flags=None):
    """Plot every usable matched pair, including outliers; never validate a transform."""
    import matplotlib
    matplotlib.use('Agg')  # Works on a headless ROS computer, no display or plt.show().
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection

    variances = np.full((len(source),3),np.nan) if variances is None else variances
    selected_mask = np.zeros(len(source),bool) if selected_mask is None else selected_mask
    quality_flags = np.ones(len(source),bool) if quality_flags is None else quality_flags
    fixed_flags = np.full(len(source),-1) if fixed_flags is None else fixed_flags
    blue, orange = '#0072B2', '#D55E00'
    fig, axes = plt.subplots(2, 2, figsize=(15, 11), constrained_layout=True)
    title = ('NO COMPUTABLE TRANSFORM - no map exported' if candidate is None else
             ('TRANSFORM COMPUTED - QUALITY WARNINGS (export enabled)' if failure else 'Alignment passed quality checks'))
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
        if np.any(selected_mask):
            for axis in axes[1]:
                axis.axvspan(elapsed[selected_mask][0],elapsed[selected_mask][-1],color='green',alpha=.07,label='Selected-window extent')
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

    diagnostic = {'status':('failed' if candidate is None else ('warnings' if failure else 'passed')),
                  'failure_reason':failure if candidate is None else None,
                  'warning_summary':failure if candidate is not None else None,
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
        writer.writerow(['timestamp','map_x','map_y','map_z','rtk_e','rtk_n','rtk_u','fit_sample',
                         'var_e','var_n','var_u','selected_window','quality_pass','rtk_fixed'])
        for i, (stamp, p, q) in enumerate(zip(times, source, target)):
            writer.writerow([format(stamp,'.9f'), *p, *q, int(i in selected_set),*variances[i],
                             int(selected_mask[i]),int(quality_flags[i]),int(fixed_flags[i])])
    if errors is not None:
        with (output/'residuals.csv').open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(['timestamp','dx','dy','dz','norm','fit_sample','inlier'])
            for i, (stamp, residual, err) in enumerate(zip(times, residuals, errors)):
                writer.writerow([format(stamp,'.9f'), *residual, err, int(i in selected_set), int(err <= threshold)])
    print('Trajectory diagnostic image: '+str((output/'trajectory_comparison.png').resolve()))


def match_records(poses, records, offset, max_gap, lever):
    points = np.full((len(records), 3), np.nan)
    for i, record in enumerate(records):
        pose = interpolate(poses, record[0]+offset, max_gap)
        if pose is not None:
            p, r = pose
            points[i] = p+r@lever
    return points


def choose_samples(target, mask, distance):
    ids, last = [], None
    for i in np.flatnonzero(mask):
        if last is None or np.linalg.norm(target[i]-last) >= distance:
            ids.append(i)
            last = target[i]
    return np.asarray(ids, dtype=int)


def trimmed_fit(x, y, w, minimum):
    keep = np.arange(len(x))
    count = max(minimum, int(np.ceil(.7*len(x))))
    for _ in range(6):
        r, t = fit_rigid(x[keep], y[keep], w[keep])
        errors = np.linalg.norm(x@r.T+t-y, axis=1)
        new = np.sort(np.argsort(errors)[:count])
        if np.array_equal(new, keep):
            break
        keep = new
    r, t = fit_rigid(x[keep], y[keep], w[keep])
    errors = np.linalg.norm(x@r.T+t-y, axis=1)
    return r, t, errors, float(np.sqrt(np.average(errors[keep]**2, weights=w[keep])))


def motion_geometry(x):
    if len(x) < 3:
        return 0., 0.
    centered = x-x.mean(axis=0)
    values = np.linalg.svd(centered, compute_uv=False)
    return float(np.linalg.norm(np.ptp(x, axis=0))), float(values[1]/max(values[0], 1e-12))


def read_quality(path, stamps):
    """Exact stamped sidecar avoids cross-topic callback ordering assumptions."""
    fixed = np.full(len(stamps), -1, dtype=int)  # -1 is UNKNOWN, not float/fixed.
    if not path.is_file():
        return fixed
    table = {}
    with path.open() as stream:
        for row in csv.DictReader(stream):
            key = format(float(row['timestamp']), '.9f')
            typ = row['pos_type'].upper()
            valid = row['sol_status'].upper() == 'SOL_COMPUTED'
            table[key] = int(valid and typ in ('NARROW_INT', 'L1_INT', 'WIDE_INT', 'RTK_FIXED', 'FIXED'))
    for i, stamp in enumerate(stamps):
        fixed[i] = table.get(format(stamp, '.9f'), -1)
    return fixed


def automatic_alignment(poses, records, fixed, args):
    """Compare overlapping multi-scale windows, then require transform consensus."""
    target = records[:, 1:4]
    times = records[:, 0]
    weights = 1./np.maximum(records[:, 4:7].mean(axis=1), 1e-6)
    quality = np.ones(len(records), dtype=bool)
    if args.max_xy_variance > 0:
        quality &= np.max(records[:, 4:6], axis=1) <= args.max_xy_variance
    if args.max_z_variance > 0:
        quality &= records[:, 6] <= args.max_z_variance
    # Known non-fixed solutions are never preferred over fixed or unknown records.
    preferred = quality & (fixed != 0)
    notes = []
    if np.sum(fixed >= 0) == 0:
        notes.append('RTK fixed/float metadata unavailable; selection uses covariance, motion and consistency only')
    if preferred.sum() < args.min_pairs:
        preferred = quality.copy()
        notes.append('Too few fixed/unknown quality points; known non-fixed records may be used as fallback')
    if preferred.sum() < args.min_pairs:
        preferred[:] = True
        notes.append('Too few covariance-qualified points; falling back to all valid records (UNTRUSTED)')
    if len(records) < 3:
        raise AlignmentFitError('Need at least three valid RTK records for a rigid transform')
    if args.auto_time_offset:
        intervals = int(np.ceil((args.offset_max-args.offset_min)/args.offset_step))
        offsets = np.unique(np.r_[np.linspace(args.offset_min, args.offset_max, intervals+1), args.time_offset])
    else:
        offsets = np.array([args.time_offset])
    # Cache interpolation per offset. Window scoring does not repeat IMU interpolation.
    predictions = [match_records(poses, records, float(o), args.max_gap, args.lever_arm) for o in offsets]
    base_index = int(np.argmin(abs(offsets-args.time_offset)))
    duration = float(times[-1]-times[0])
    base = min(args.window_seconds, max(duration, .1))
    windows = set()
    for width in (base*.5, base, min(base*2, duration)):
        if width <= 0:
            continue
        end_start = max(0., duration-width)
        starts = np.unique(np.r_[np.arange(0, end_start+1e-6, max(width/4., .1)), end_start])
        for start in starts:
            windows.add((round(float(start), 6), round(float(min(start+width, duration)), 6)))
    windows = sorted(windows)
    if len(windows) > 120:
        windows = [windows[i] for i in np.unique(np.linspace(0, len(windows)-1, 120, dtype=int))]
    candidates, attempts = [], []
    reference = times[0]
    for number, (start, end) in enumerate(windows):
        mask = preferred & (times >= reference+start) & (times <= reference+end)
        # Fixed solutions get priority within each window, if they provide enough geometry.
        fixed_mask = mask & (fixed == 1)
        if fixed_mask.sum() >= args.min_pairs:
            points = predictions[base_index][fixed_mask]
            if (np.isfinite(points).all() and motion_geometry(points)[0] >= args.min_motion_span
                    and motion_geometry(points)[1] >= args.min_motion_ratio):
                mask = fixed_mask
        ids = choose_samples(target, mask, args.sample_distance)
        if len(ids) > 160:
            ids = ids[np.unique(np.linspace(0, len(ids)-1, 160, dtype=int))]
        common = np.ones(len(ids), dtype=bool)
        for points in predictions:
            common &= np.isfinite(points[ids]).all(axis=1)
        ids = ids[common]
        row = {'window_id':number, 'start_s':start, 'end_s':end, 'pairs':len(ids), 'reason':''}
        if len(ids) < args.min_pairs:
            row['reason'] = 'insufficient_common_support'
            attempts.append(row)
            continue
        span, ratio = motion_geometry(predictions[base_index][ids])
        row.update(span_m=span, motion_ratio=ratio)
        if span < args.min_motion_span or ratio < args.min_motion_ratio:
            row['reason'] = 'insufficient_motion_geometry'
            attempts.append(row)
            continue
        trials = []
        for offset_index, points in enumerate(predictions):
            try:
                r, t, errors, score = trimmed_fit(points[ids], target[ids], weights[ids], args.min_pairs)
                trials.append((score, offset_index, r, t))
            except ValueError:
                pass
        if not trials:
            row['reason'] = 'degenerate_fit'
            attempts.append(row)
            continue
        score, oi, r, t = min(trials, key=lambda item:item[0])
        scores = [item[0] for item in trials]
        time_warning = ''
        if args.auto_time_offset and np.ptp(scores) < 1e-5:
            # Don't infer an arbitrary time shift when rotation/translation can absorb it.
            trial = next((item for item in trials if item[1] == base_index), None)
            if trial is not None:
                score, oi, r, t = trial
            time_warning = 'time_offset_unobservable_using_requested_offset'
        elif args.auto_time_offset and (abs(offsets[oi]-args.offset_min)<1e-8 or abs(offsets[oi]-args.offset_max)<1e-8):
            time_warning = 'best_offset_at_search_boundary'
        error = np.linalg.norm(predictions[oi][ids]@r.T+t-target[ids], axis=1)
        local_mask = error <= args.inlier_threshold
        rmse = float(np.sqrt(np.mean(error[local_mask]**2))) if local_mask.any() else float(np.sqrt(np.mean(error**2)))
        qualified = local_mask.sum() >= args.min_pairs and local_mask.mean() >= args.min_inlier_ratio and rmse <= args.max_rmse and not time_warning
        row.update(offset_s=float(offsets[oi]), trimmed_rmse_m=score, inlier_ratio=float(local_mask.mean()),
                   inlier_rmse_m=rmse, mean_variance=float(records[ids,4:7].mean()),
                   fixed_fraction=float(np.mean(fixed[ids] == 1)), qualified=bool(qualified), reason=time_warning)
        attempts.append(row)
        candidates.append({'row':row, 'r':r, 't':t, 'oi':oi, 'ids':ids, 'qualified':qualified, 'support':[]})
    # Prefer stable solutions supported by distinct, neighbouring time windows.
    for candidate in candidates:
        row = candidate['row']
        for other in candidates:
            if not candidate['qualified'] or not other['qualified']:
                continue
            second = other['row']
            overlap = max(0., min(row['end_s'],second['end_s'])-max(row['start_s'],second['start_s']))
            shortest = min(row['end_s']-row['start_s'],second['end_s']-second['start_s'])
            if other is not candidate and overlap > .8*shortest:
                continue  # Nearly duplicate windows do not count as independent support.
            center_gap = abs((row['start_s']+row['end_s']-second['start_s']-second['end_s'])/2)
            if center_gap > max(row['end_s']-row['start_s'], second['end_s']-second['start_s']):
                continue
            angle = float(np.degrees(np.arccos(np.clip((np.trace(candidate['r']@other['r'].T)-1)/2,-1,1))))
            probes = predictions[candidate['oi']][np.unique(np.r_[candidate['ids'],other['ids']])]
            delta = probes@candidate['r'].T+candidate['t']-(probes@other['r'].T+other['t'])
            disagreement = float(np.sqrt(np.mean(np.sum(delta**2,axis=1))))
            if (angle <= args.consistency_angle and disagreement <= args.consistency_distance
                    and abs(row['offset_s']-second['offset_s']) <= args.consistency_time):
                candidate['support'].append(other)
        row['consistent_windows'] = len(candidate['support'])
    stable = [c for c in candidates if c['qualified'] and len(c['support']) >= args.min_consistent_windows]
    (args.output/'auto_windows.json').write_text(json.dumps(attempts, indent=2, allow_nan=False)+'\n')
    pool = stable or [c for c in candidates if c['qualified']] or candidates
    if pool:
        # Consensus first; then balance geometry/coverage, covariance and residual, not smallest RMSE alone.
        def rank(c):
            row = c['row']
            return (len(c['support']), row['fixed_fraction'],
                    row['inlier_ratio']*min(row['span_m'],100.)/(max(row['trimmed_rmse_m'],.05)*np.sqrt(max(row['mean_variance'],1e-6))))
        best = max(pool, key=rank)
        if not stable:
            notes.append('No multi-window consensus met requirements; exporting best available candidate with warnings')
        oi = best['oi']
        selected_windows = best['support'] if stable else [best]
        # Keep the selected seed as well even when overlapping support rules exclude it.
        if not any(item is best for item in selected_windows):
            selected_windows = [best]+selected_windows
        seed_r, seed_t = best['r'], best['t']
        seed_offset = float(offsets[oi])
        eligible = np.zeros(len(records), dtype=bool)
        for item in selected_windows:
            eligible |= preferred & (times >= reference+item['row']['start_s']) & (times <= reference+item['row']['end_s'])
        source = predictions[oi]
        eligible &= np.isfinite(source).all(axis=1)
        ids = choose_samples(target, eligible, args.sample_distance)
        try:
            r, t, _ = robust_fit(source[ids],target[ids],weights[ids],args.inlier_threshold,args.ransac_trials,args.min_pairs,args.min_inlier_ratio)
        except ValueError as exc:
            candidate = getattr(exc,'candidate',None)
            r, t = candidate if candidate is not None else (seed_r,seed_t)
            notes.append('Final consensus refinement did not pass: '+str(exc))
        selected_rows = [item['row'] for item in selected_windows]
        if best['row']['reason']:
            notes.append(best['row']['reason'])
        trusted = bool(stable)
    else:
        # No window passed motion requirements: still attempt an actual non-degenerate global fit.
        notes.append('No usable automatic segment; falling back to global fit without a trusted segment')
        source = predictions[base_index]
        eligible = preferred & np.isfinite(source).all(axis=1)
        ids = choose_samples(target,eligible,args.sample_distance)
        if len(ids) < 3:
            eligible = np.isfinite(source).all(axis=1)
            ids = choose_samples(target,eligible,args.sample_distance)
        if len(ids) < 3:
            ids = np.flatnonzero(np.isfinite(source).all(axis=1))
        r,t = fit_rigid(source[ids],target[ids],weights[ids])
        seed_offset = float(offsets[base_index])
        selected_rows, trusted = [], False
    picked = {row['window_id'] for row in selected_rows}
    for row in attempts:
        row['selected_for_refinement'] = row['window_id'] in picked
    (args.output/'auto_windows.json').write_text(json.dumps(attempts,indent=2,allow_nan=False)+'\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig,axes = plt.subplots(2,1,figsize=(11,7),constrained_layout=True)
    for row in attempts:
        if 'trimmed_rmse_m' not in row:
            continue
        center=(row['start_s']+row['end_s'])/2
        color='#0072B2' if row['qualified'] else '#999999'
        marker='*' if row['selected_for_refinement'] else 'o'
        size=90 if row['selected_for_refinement'] else 15
        axes[0].scatter(center,row['trimmed_rmse_m'],c=color,marker=marker,s=size)
        axes[1].scatter(center,row['offset_s'],c=color,marker=marker,s=size)
    axes[0].set(title='Automatic windows: blue=qualified; gray=not qualified; star=selected',ylabel='Trimmed RMSE [m]')
    axes[1].set(xlabel='Window center relative to first valid RTK [s]',ylabel='Candidate time offset [s]')
    for axis in axes: axis.grid(True,alpha=.25)
    fig.savefig(args.output/'automatic_selection.png',dpi=150)
    plt.close(fig)
    return r,t,seed_offset,eligible,selected_rows,attempts,notes,trusted,quality


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--session', required=True, type=Path, help='Voxel-SLAM session directory with alidarState.txt and i.pcd')
    parser.add_argument('--rtk', required=True, type=Path, help='rtk_enu.csv recorded by gnss_to_enu')
    parser.add_argument('--datum', type=Path,
                        help='Defaults to map_enu.meta.yaml beside RTK CSV')
    parser.add_argument('--output', required=True, type=Path, help='New output directory; existing paths are refused')
    parser.add_argument('--publish-dir', type=Path,
                        help='Optional folder for map_enu_downsampled.pcd and map_enu.meta.yaml only')
    parser.add_argument('--lever-arm', nargs=3, type=float, required=True, metavar=('X','Y','Z'), help='IMU origin -> antenna, expressed in IMU axes, metres')
    parser.add_argument('--time-offset', type=float, default=0., help='t_map = t_RTK + offset, seconds')
    parser.add_argument('--max-gap', type=float, default=.25, help='Maximum trajectory interpolation interval, seconds')
    parser.add_argument('--sample-distance', type=float, default=1., help='3D RTK displacement between fitting points, metres')
    parser.add_argument('--inlier-threshold', type=float, default=.5, help='RANSAC Euclidean residual gate, metres')
    parser.add_argument('--min-pairs', type=int, default=10)
    parser.add_argument('--min-inlier-ratio', type=float, default=.7)
    parser.add_argument('--max-rmse', type=float, default=.3, help='Maximum inlier 3D RMSE; failing produces no export')
    parser.add_argument('--ransac-trials', type=int, default=500)
    parser.add_argument('--map-voxel-size', type=float, default=.15, help='Global ENU voxel centroid size [m]; 0 disables extra downsampled PCD')
    parser.add_argument('--alignment-only', action='store_true', help='Export alignment diagnostics without reading PCDs')
    parser.add_argument('--auto-time-offset', action='store_true', help='Search time offset separately in automatic candidate windows')
    parser.add_argument('--offset-min', type=float, default=-2.)
    parser.add_argument('--offset-max', type=float, default=2.)
    parser.add_argument('--offset-step', type=float, default=.1)
    parser.add_argument('--window-seconds', type=float, default=120., help='Automatic window base length; also tests half/double length')
    parser.add_argument('--max-xy-variance', type=float, default=.04, help='Preferred fit-point variance upper bound [m^2]; 0 disables upper bound')
    parser.add_argument('--max-z-variance', type=float, default=.25)
    parser.add_argument('--min-motion-span', type=float, default=10., help='Preferred window trajectory extent [m]')
    parser.add_argument('--min-motion-ratio', type=float, default=.02, help='Preferred window second/first spatial singular value ratio')
    parser.add_argument('--consistency-distance', type=float, default=2., help='Maximum neighbouring transforms disagreement at trajectory probes [m]')
    parser.add_argument('--consistency-angle', type=float, default=2., help='Maximum neighbouring rotation difference [deg]')
    parser.add_argument('--consistency-time', type=float, default=.3, help='Maximum neighbouring offset difference [s]')
    parser.add_argument('--min-consistent-windows', type=int, default=2)
    args = parser.parse_args(argv)
    if (args.max_gap <= 0 or args.sample_distance < 0 or args.inlier_threshold <= 0 or
            args.min_pairs < 3 or not 0 < args.min_inlier_ratio <= 1 or args.max_rmse <= 0 or args.ransac_trials < 0):
        raise ValueError('Invalid alignment thresholds')
    if not np.isfinite(args.lever_arm + [args.time_offset, args.max_gap, args.sample_distance,
                                       args.inlier_threshold, args.max_rmse]).all():
        raise ValueError('Non-finite parameters')
    parameters = [args.offset_min,args.offset_max,args.offset_step,args.window_seconds,
                  args.max_xy_variance,args.max_z_variance,args.min_motion_span,args.min_motion_ratio,
                  args.consistency_distance,args.consistency_angle,args.consistency_time]
    if (not np.isfinite(parameters).all() or args.offset_min >= args.offset_max or args.offset_step <= 0
            or (args.offset_max-args.offset_min)/args.offset_step > 200 or args.window_seconds <= 0
            or min(parameters[4:]) < 0 or args.min_motion_ratio >= 1 or args.min_consistent_windows < 2):
        raise ValueError('Invalid automatic selection parameters (<=201 time search grid points)')
    if not np.isfinite(args.map_voxel_size) or args.map_voxel_size < 0:
        raise ValueError('map_voxel_size must be finite and nonnegative')
    if args.publish_dir is not None and not args.alignment_only and args.map_voxel_size <= 0:
        raise ValueError('--publish-dir requires --map-voxel-size > 0')
    if args.output.exists():
        raise ValueError('Output already exists; select a new directory')
    poses = read_poses(args.session/'alidarState.txt')
    if args.datum is not None:
        datum_path = args.datum
    else:
        datum_path = args.rtk.parent/ENU_META_FILENAME
        legacy_datum = args.rtk.parent/'datum.json'
        if not datum_path.exists() and legacy_datum.exists():
            datum_path = legacy_datum
    datum = read_enu_origin(datum_path)
    records, invalid = [], 0
    last_stamp = -np.inf
    with args.rtk.open() as stream:
        for row in csv.DictReader(stream):
            record = np.array([float(row[k]) for k in ('timestamp','east','north','up','var_e','var_n','var_u')])
            if (not np.isfinite(record).all() or np.any(record[4:] <= 0) or record[0] <= last_stamp):
                invalid += 1
                continue
            last_stamp = record[0]
            records.append(record)
    records = np.asarray(records).reshape(-1,7)
    fixed = read_quality(args.rtk.parent/'rtk_quality.csv', records[:,0])
    raw_target, raw_rtk_times = records[:,1:4], records[:,0]
    raw_source = np.array([pose[1:4]+quaternion_matrix(pose[4:8])@args.lever_arm for pose in poses])
    args.output.mkdir(parents=True)
    parameters = {k:v for k,v in vars(args).items() if isinstance(v,(float,int,bool))}
    common = {'datum':datum,'parameters':parameters,'lever_arm_imu_m':args.lever_arm,
              'inputs':{'session':str(args.session.resolve()),'rtk':str(args.rtk.resolve())},
              'fit_reference_rtk_timestamp':float(records[0,0]) if len(records) else None}
    try:
        r,t,offset,eligible,windows,attempts,warnings,trusted,quality = automatic_alignment(poses,records,fixed,args)
    except ValueError as exc:
        predictions = match_records(poses,records,args.time_offset,args.max_gap,args.lever_arm)
        matched = np.isfinite(predictions).all(axis=1)
        trajectory_diagnostics(args.output,predictions[matched],raw_target[matched],raw_rtk_times[matched],
                               np.array([],dtype=int),args.inlier_threshold,raw_source,raw_target,
                               poses[:,0],raw_rtk_times,None,'none',str(exc))
        report = {**common,'status':'alignment_failed','quality_passed':False,'T_enu_map':None,
                  'failure_reason':str(exc),'warnings':['No computable non-degenerate rigid transform; no map was fabricated']}
        (args.output/'alignment.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        raise
    predictions = match_records(poses,records,offset,args.max_gap,args.lever_arm)
    matched = np.isfinite(predictions).all(axis=1)
    source,target,times = predictions[matched],raw_target[matched],raw_rtk_times[matched]
    variances,quality_flags,fixed_flags = records[matched,4:7],quality[matched],fixed[matched]
    eligible = eligible[matched]
    selected = choose_samples(target,eligible,args.sample_distance)
    residuals = source@r.T+t-target
    errors = np.linalg.norm(residuals,axis=1)
    accepted = errors <= args.inlier_threshold
    local = errors[selected]
    local_inliers = local <= args.inlier_threshold
    local_rmse = float(np.sqrt(np.mean(local[local_inliers]**2))) if local_inliers.any() else None
    local_ratio = float(local_inliers.mean()) if len(local_inliers) else 0.
    stats = {'matched':len(source),'excluded':invalid+int((~matched).sum()),'fit_points':len(selected),
             'fit_inliers':int(local_inliers.sum()),'fit_inlier_ratio':local_ratio,'fit_inlier_rmse_m':local_rmse,
             'all_rmse_m':float(np.sqrt(np.mean(errors**2))), 'all_median_m':float(np.median(errors)),
             'all_p95_m':float(np.percentile(errors,95)), 'all_max_m':float(errors.max()),
             'all_inlier_ratio':float(accepted.mean()),'covariance_qualified_matched':int(quality_flags.sum()),
             'rtk_fixed_matched':int(np.sum(fixed_flags==1)),'rtk_solution_unknown_matched':int(np.sum(fixed_flags==-1))}
    violations=[]
    if local_inliers.sum() < args.min_pairs: violations.append('fit inliers %d < min_pairs %d'%(local_inliers.sum(),args.min_pairs))
    if local_ratio < args.min_inlier_ratio: violations.append('fit inlier ratio %.3f < %.3f'%(local_ratio,args.min_inlier_ratio))
    if local_rmse is None or local_rmse > args.max_rmse: violations.append('fit inlier RMSE %s exceeds max_rmse %.3f m'%(str(local_rmse),args.max_rmse))
    if accepted.mean() < args.min_inlier_ratio: violations.append('whole-trajectory inlier ratio %.3f < %.3f'%(accepted.mean(),args.min_inlier_ratio))
    if not trusted: violations.append('no reliable multi-window consensus')
    warnings += violations
    if stats['all_p95_m'] > args.inlier_threshold:
        warnings.append('Some GNSS/SLAM sections disagree; 95th-percentile error %.3f m. One rigid transform cannot reconcile varying errors'%stats['all_p95_m'])
    warnings = list(dict.fromkeys(warnings))
    quality_passed = not violations
    for warning in warnings:
        print('WARNING: '+warning, file=sys.stderr)
    method='automatic multi-window consensus' if trusted else 'best available fallback (UNTRUSTED)'
    trajectory_diagnostics(args.output,source,target,times,selected,args.inlier_threshold,
                           raw_source,raw_target,poses[:,0],raw_rtk_times,(r,t),method,
                           'See alignment.json warnings; export still enabled' if warnings else None,
                           variances,eligible,quality_flags,fixed_flags)
    transform=np.eye(4);transform[:3,:3],transform[:3,3]=r,t
    report={**common,'status':'alignment_with_warnings' if warnings else 'alignment_validated',
            'quality_passed':quality_passed,'warnings':warnings,'T_enu_map':transform.tolist(),
            'time_offset_s':offset,'requested_time_offset_s':args.time_offset,
            'trusted_segment_found':trusted,'selected_windows':windows,'metrics':stats,
            'export_policy':'quality warnings do not block export; uncomputable transform or file IO errors do',
            'convention':'p_enu = R_enu_map @ p_map + t_enu_map; scale=1',
            'weighting':'inverse mean ENU variance (scalar)'}
    def save_report():
        (args.output/'alignment.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    save_report()
    np.savetxt(args.output/'T_enu_map.txt',transform,fmt='%.12g')
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
            report['map_points'],report['downsampled_map_points'] = export_map(
                args.session,poses,r,t,args.output,args.map_voxel_size)
            report['map_voxel_size_m'] = args.map_voxel_size
            report['map_files'] = {'full':'map_enu.pcd',
                                   'downsampled':'map_enu_downsampled.pcd' if args.map_voxel_size > 0 else None}
            report['status'] = 'complete_with_warnings' if warnings else 'complete'
        except Exception:
            report['status'] = 'map_export_failed'
            (args.output/'map_enu.pcd').unlink(missing_ok=True)
            (args.output/'map_enu_downsampled.pcd').unlink(missing_ok=True)
            save_report()
            raise
        if args.publish_dir is not None:
            try:
                report['published_files'] = publish_outputs(
                    args.output, datum_path, args.publish_dir)
            except Exception:
                report['status'] = 'publish_failed'
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
    publish_dir = rospy.get_param('~publish_dir', '')
    if publish_dir:
        argv.extend(['--publish-dir', publish_dir])
    lever = rospy.get_param('~lever_arm')
    if not isinstance(lever, list) or len(lever) != 3:
        raise ValueError('~lever_arm must contain three IMU-frame metre values')
    argv.extend(['--lever-arm', *map(str, lever)])
    for name, default in (('time_offset', 0.), ('max_gap', .25), ('sample_distance', 1.),
                          ('inlier_threshold', .5), ('min_pairs', 10),
                          ('min_inlier_ratio', .7), ('max_rmse', .3), ('ransac_trials', 500), ('map_voxel_size',.15),
                          ('offset_min',-2.),('offset_max',2.),('offset_step',.1),('window_seconds',120.),
                          ('max_xy_variance',.04),('max_z_variance',.25),('min_motion_span',10.),
                          ('min_motion_ratio',.02),('consistency_distance',2.),('consistency_angle',2.),
                          ('consistency_time',.3),('min_consistent_windows',2)):
        argv.extend(['--'+name.replace('_', '-'), str(rospy.get_param('~'+name, default))])
    automatic = rospy.get_param('~auto_time_offset', True)
    if not isinstance(automatic,bool):
        raise ValueError('~auto_time_offset must be a ROS boolean')
    if automatic:
        argv.append('--auto-time-offset')
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
    rospy.loginfo('Offline ENU export task finished; inspect quality_passed and warnings in alignment.json')


if __name__ == '__main__':
    try:
        if '--ros-params' in sys.argv[1:]:
            run_ros()
        else:
            main()
    except (ValueError, OSError, KeyError, ImportError) as exc:
        print('ENU alignment failed: '+str(exc), file=sys.stderr)
        sys.exit(1)
