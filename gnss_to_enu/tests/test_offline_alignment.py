"""Run with python3 -m unittest discover -s src/gnss_to_enu/tests -v."""
import csv
import contextlib
import io
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
from types import SimpleNamespace
import numpy as np

SCRIPT = Path(__file__).resolve().parents[1]/'scripts'/'align_voxel_enu.py'
spec = importlib.util.spec_from_file_location('alignment', SCRIPT)
a = importlib.util.module_from_spec(spec)
spec.loader.exec_module(a)


def write_meta(path, latitude=22., longitude=113., altitude=10.):
    Path(path).write_text(
        'enu_origin:\n'
        f'  latitude: {latitude:.8f}\n'
        f'  longitude: {longitude:.8f}\n'
        f'  altitude: {altitude:.8f}\n')


class AlignmentTests(unittest.TestCase):
    def test_ros_parameter_entrypoint(self):
        params={'~session':'/data/session with spaces', '~rtk':'/data/rtk.csv',
                '~output':'/data/enu', '~lever_arm':[.1,-.2,.3], '~alignment_only':True,
                '~sample_distance':2., '~datum':'/data/map_enu.meta.yaml',
                '~publish_dir':'/data/output', '~map_voxel_size':0.5}
        ros=SimpleNamespace(init_node=lambda name:None, loginfo=lambda msg:None,
                            get_param=lambda key,*default:params[key] if key in params else default[0])
        with patch.dict('sys.modules', {'rospy':ros}), patch.object(a,'main') as main:
            a.run_ros()
            argv=main.call_args[0][0]
            self.assertEqual(argv[argv.index('--session')+1],'/data/session with spaces')
            self.assertEqual(argv[argv.index('--lever-arm')+1:argv.index('--lever-arm')+4],['0.1','-0.2','0.3'])
            self.assertIn('--alignment-only',argv)
            self.assertEqual(argv[argv.index('--map-voxel-size')+1],'0.5')
            self.assertEqual(argv[argv.index('--sample-distance')+1],'2.0')
            self.assertEqual(argv[argv.index('--datum')+1],'/data/map_enu.meta.yaml')
            self.assertEqual(argv[argv.index('--publish-dir')+1],'/data/output')
            params['~alignment_only']='false'
            with self.assertRaises(ValueError): a.run_ros()

    def test_slerp_gap_bounds_and_antipodes(self):
        poses = np.array([[1,0,0,0,0,0,0,1], [1.1,2,0,0,0,0,1,0]], dtype=float)
        p,r = a.interpolate(poses, 1.05, .2)
        np.testing.assert_allclose(p, [1,0,0], atol=1e-12)
        np.testing.assert_allclose(r@[1,0,0], [0,1,0], atol=1e-12)
        self.assertIsNone(a.interpolate(poses, .9, .2))
        self.assertIsNone(a.interpolate(poses, 1.05, .05))
        np.testing.assert_allclose(a.slerp(np.array([0.,0,0,1]), np.array([0.,0,0,-1]), .5), [0,0,0,1])

    def test_robust_fit_and_line_rejection(self):
        rng = np.random.RandomState(4)
        x = rng.normal(size=(100,3))*10
        r = a.quaternion_matrix([.2,.1,.3,.9])
        t = np.array([2.,4.,6.])
        y = x@r.T+t
        y[:15] += 30
        rr,tt,mask = a.robust_fit(x,y,np.ones(100),.1,300,10,.7)
        self.assertEqual(mask.sum(),85)
        np.testing.assert_allclose(rr,r,atol=1e-12)
        np.testing.assert_allclose(tt,t,atol=1e-12)
        line = np.column_stack([np.arange(10),np.zeros((10,2))])
        with self.assertRaises(ValueError):
            a.fit_rigid(line,line)

    def test_end_to_end_lever_arm_pcd_and_outputs(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            session=root/'session'; session.mkdir()
            stamp=1700000000.+np.arange(40)*.1
            theta=np.linspace(0, 2*np.pi,40)
            poses=np.column_stack([stamp,10*np.cos(theta),8*np.sin(theta),theta*.1,
                                   np.zeros(40),np.zeros(40),np.sin(theta/2),np.cos(theta/2)])
            np.savetxt(session/'alidarState.txt',poses,fmt='%.12f')
            r=a.quaternion_matrix([.1,.2,.3,.9]); t=np.array([100.,20.,3.])
            lever=np.array([.4,-.2,.3])
            xyz=np.array([[1.,2.,3.],[-1.,0.,2.]])
            header=b'VERSION .7\nFIELDS x y z intensity\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\nDATA binary\n'
            for i in range(40):
                with (session/(str(i)+'.pcd')).open('wb') as f:
                    f.write(header)
                    np.column_stack([xyz,np.zeros(2)]).astype('<f4').tofile(f)
            write_meta(root/'map_enu.meta.yaml')
            with (root/'rtk_enu.csv').open('w') as f:
                w=csv.writer(f); w.writerow(['timestamp','east','north','up','var_e','var_n','var_u'])
                # Use midpoint times to exercise interpolation rather than exact matches.
                for i in range(39):
                    time=(stamp[i]+stamp[i+1])/2
                    p,rot=a.interpolate(poses,time,.25)
                    enu=r@(p+rot@lever)+t
                    w.writerow([format(time,'.9f'),*enu,.0004,.0004,.0016])
            out=root/'output'
            published=root/'published'
            args=['--session',str(session),'--rtk',str(root/'rtk_enu.csv'),'--output',str(out),
                  '--publish-dir',str(published),
                  '--lever-arm',*map(str,lever),'--sample-distance','0.5','--inlier-threshold','0.01']
            a.main(args)
            tr=np.loadtxt(out/'T_enu_map.txt')
            np.testing.assert_allclose(tr[:3,:3],r,atol=1e-9)
            np.testing.assert_allclose(tr[:3,3],t,atol=1e-9)
            cloud=a.read_pcd_xyz(out/'map_enu.pcd')
            expected=xyz@(r@a.quaternion_matrix(poses[0,4:])).T+r@poses[0,1:4]+t
            np.testing.assert_allclose(cloud[:2],expected,atol=1e-5)
            trajectory=np.loadtxt(out/'trajectory_enu.tum')
            np.testing.assert_allclose(a.quaternion_matrix(trajectory[10,4:]),r@a.quaternion_matrix(poses[10,4:]),atol=1e-8)
            self.assertIn(json.loads((out/'alignment.json').read_text())['status'],('complete','complete_with_warnings'))
            self.assertTrue((out/'map_enu_downsampled.pcd').exists())
            report=json.loads((out/'alignment.json').read_text())
            self.assertEqual(report['map_voxel_size_m'],.15)
            self.assertEqual(report['downsampled_map_points'],len(a.read_pcd_xyz(out/'map_enu_downsampled.pcd')))
            self.assertTrue((out/'trajectory_comparison.png').stat().st_size > 10000)
            self.assertIsNotNone(json.loads((out/'diagnostic.json').read_text())['candidate_T_enu_map'])
            self.assertEqual(
                {path.name for path in published.iterdir()},
                {'map_enu_downsampled.pcd', 'map_enu.meta.yaml'})
            self.assertEqual(
                (published/'map_enu.meta.yaml').read_text(),
                (root/'map_enu.meta.yaml').read_text())
            with self.assertRaises(ValueError):
                a.main(args)
            # A computable transform MUST export the map even when quality thresholds fail.
            warning_args=args.copy()
            warning_args[warning_args.index('--output')+1]=str(root/'warning_export')
            warning_args+=['--min-pairs','1000']
            with contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
                a.main(warning_args)
            warning_report=json.loads((root/'warning_export'/'alignment.json').read_text())
            self.assertEqual(warning_report['status'],'complete_with_warnings')
            self.assertFalse(warning_report['quality_passed'])
            self.assertTrue((root/'warning_export'/'map_enu.pcd').exists())
            (session/'3.pcd').unlink()
            args[args.index('--output')+1]=str(root/'missing')
            with self.assertRaises(ValueError):
                a.main(args)
            self.assertFalse((root/'missing'/'map_enu.pcd').exists())
            self.assertFalse((root/'missing'/'map_enu_downsampled.pcd').exists())
            self.assertEqual(json.loads((root/'missing'/'alignment.json').read_text())['status'],'map_export_failed')

    def test_failure_diagnostics(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            session=root/'session'; session.mkdir()
            n=40
            stamps=1700000000.+np.arange(n)*.1
            angle=np.linspace(0,2*np.pi,n)
            xyz=np.column_stack([10*np.cos(angle),8*np.sin(angle),angle*.1])
            write_meta(root/'map_enu.meta.yaml')
            cases=('quality_gate','insufficient_inliers','too_few_samples','degenerate','no_time_overlap','no_valid_rtk')
            for case in cases:
                with self.subTest(case=case):
                    points=xyz.copy()
                    if case=='degenerate':
                        points=np.column_stack([np.arange(n),np.zeros((n,2))])
                    poses=np.column_stack([stamps,points,np.zeros((n,3)),np.ones(n)])
                    np.savetxt(session/'alidarState.txt',poses,fmt='%.12f')
                    target=points+np.array([5.,-2.,1.])
                    if case in ('quality_gate','insufficient_inliers'):
                        target=target+np.random.RandomState(8).normal(size=(n,3))*.1
                    csvpath=root/(case+'.csv')
                    with csvpath.open('w') as f:
                        w=csv.writer(f)
                        w.writerow(['timestamp','east','north','up','var_e','var_n','var_u'])
                        for time,position in zip(stamps,target):
                            w.writerow([format(time+(1000 if case=='no_time_overlap' else 0),'.9f'),
                                        *position,0 if case=='no_valid_rtk' else .001,.001,.001])
                    output=root/('out_'+case)
                    args=['--session',str(session),'--rtk',str(csvpath),'--output',str(output),
                          '--lever-arm','0','0','0','--sample-distance','0','--alignment-only']
                    if case=='quality_gate':
                        args+=['--max-rmse','0.00001']
                    if case=='insufficient_inliers':
                        args+=['--inlier-threshold','0.001','--min-inlier-ratio','0.99']
                    if case=='too_few_samples':
                        args+=['--sample-distance','1000']
                    uncomputable=case in ('degenerate','no_time_overlap','no_valid_rtk')
                    if uncomputable:
                        with self.assertRaises(ValueError), contextlib.redirect_stdout(io.StringIO()):
                            a.main(args)
                    else:
                        with contextlib.redirect_stdout(io.StringIO()): a.main(args)
                    self.assertTrue((output/'trajectory_comparison.png').stat().st_size > 10000)
                    report=json.loads((output/'alignment.json').read_text())
                    self.assertFalse((output/'map_enu.pcd').exists())  # alignment-only
                    self.assertFalse((output/'map_enu_downsampled.pcd').exists())
                    if uncomputable:
                        self.assertEqual(report['status'],'alignment_failed')
                        self.assertIsNone(report['T_enu_map'])
                        self.assertFalse((output/'T_enu_map.txt').exists())
                    else:
                        self.assertEqual(report['status'],'alignment_with_warnings')
                        self.assertTrue(report['warnings'])
                        self.assertIsNotNone(report['T_enu_map'])
                        self.assertTrue((output/'T_enu_map.txt').exists())

    def test_automatic_segments_quality_sidecar_and_time_search(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp); session=root/'session'; session.mkdir()
            t=np.arange(0,185,.1); stamps=1700000000.+t
            xyz=np.column_stack([12*np.sin(.07*t)+.004*t*t,15*np.cos(.05*t)+.2*t,2*np.sin(.03*t)])
            yaw=.03*t+.0002*t*t
            poses=np.column_stack([stamps,xyz,np.zeros((len(t),2)),np.sin(yaw/2),np.cos(yaw/2)])
            np.savetxt(session/'alidarState.txt',poses,fmt='%.12f')
            write_meta(root/'map_enu.meta.yaml')
            r=a.quaternion_matrix([.1,.2,.3,.9]); tr=np.array([20.,-10.,5.]); lever=np.array([.3,-.2,.1])
            rt=1700000000.+np.arange(2,180,1.)
            with (root/'rtk.csv').open('w') as f,(root/'rtk_quality.csv').open('w') as q:
                writer=csv.writer(f); quality=csv.writer(q)
                writer.writerow(['timestamp','east','north','up','var_e','var_n','var_u'])
                quality.writerow(['timestamp','sol_status','pos_type'])
                for i,stamp in enumerate(rt):
                    p,rot=a.interpolate(poses,stamp+1.,.2)
                    y=r@(p+rot@lever)+tr
                    bad=i<35 or i>135
                    if bad: y+=np.array([12.,-10.,4.])
                    v=.5 if bad else .0004
                    writer.writerow([format(stamp,'.9f'),*y,v,v,v])
                    quality.writerow([format(stamp,'.9f'),'SOL_COMPUTED','NARROW_FLOAT' if bad else 'NARROW_INT'])
            output=root/'out'
            args=['--session',str(session),'--rtk',str(root/'rtk.csv'),'--output',str(output),
                  '--lever-arm',*map(str,lever),'--alignment-only','--auto-time-offset',
                  '--offset-min','0','--offset-max','2','--offset-step','.2','--max-gap','.2',
                  '--window-seconds','60','--sample-distance','0','--min-inlier-ratio','.8',
                  '--inlier-threshold','.1','--max-rmse','.05']
            with contextlib.redirect_stdout(io.StringIO()),contextlib.redirect_stderr(io.StringIO()):
                a.main(args)
            report=json.loads((output/'alignment.json').read_text())
            self.assertAlmostEqual(report['time_offset_s'],1.)
            self.assertTrue(report['trusted_segment_found'])
            self.assertGreaterEqual(len(report['selected_windows']),2)
            self.assertLess(report['metrics']['all_inlier_ratio'],.8)
            self.assertFalse(report['quality_passed'])
            self.assertEqual(report['status'],'alignment_with_warnings')
            transform=np.loadtxt(output/'T_enu_map.txt')
            np.testing.assert_allclose(transform[:3,:3],r,atol=1e-7)
            np.testing.assert_allclose(transform[:3,3],tr,atol=1e-7)
            pairs=np.genfromtxt(output/'matched_pairs.csv',delimiter=',',names=True)
            selected=pairs['fit_sample']>0
            self.assertTrue(np.all(pairs['quality_pass'][selected]==1))
            self.assertTrue(np.all(pairs['rtk_fixed'][selected]==1))
            self.assertTrue((output/'automatic_selection.png').exists())
            self.assertTrue((output/'auto_windows.json').exists())
            self.assertTrue((output/'trajectory_comparison.png').exists())

    def test_quality_sidecar_exact_stamp_and_unknown(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'rtk_quality.csv'
            stamps=np.array([1700000000.,1700000001.,1700000002.])
            np.testing.assert_array_equal(a.read_quality(p,stamps),[-1,-1,-1])
            p.write_text('timestamp,sol_status,pos_type\n1700000000.000000000,SOL_COMPUTED,NARROW_INT\n1700000001.000000000,SOL_COMPUTED,NARROW_FLOAT\n')
            np.testing.assert_array_equal(a.read_quality(p,stamps),[1,0,-1])

    def test_global_voxel_centroids_across_frames_and_negative_coordinates(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);session=root/'session';session.mkdir();out=root/'out';out.mkdir()
            clouds=[np.array([[.1,.1,.1],[.3,.1,.1],[1.1,.1,.1]]),
                    np.array([[.1,.1,.1],[.1,.1,.1],[.1,.1,.1],[.3,.1,.1],[1.1,.1,.1]])]
            for i,cloud in enumerate(clouds):
                raw=root/('raw'+str(i))
                cloud.astype('<f4').tofile(raw)
                a.write_xyz_pcd(session/(str(i)+'.pcd'),raw,len(cloud))
            poses=np.array([[1.,0,0,0,0,0,0,1],[2.,.05,0,0,0,0,0,1]])
            rotation=np.array([[0.,-1,0],[1,0,0],[0,0,1]])
            translation=np.array([0.,-.2,0.])
            count,reduced=a.export_map(session,poses,rotation,translation,out,.5)
            self.assertEqual((count,reduced),(8,3))
            full=a.read_pcd_xyz(out/'map_enu.pcd')
            expected_full=np.vstack([clouds[0],clouds[1]+[.05,0,0]])@rotation.T+translation
            np.testing.assert_allclose(full,expected_full,atol=1e-7)
            sparse=a.read_pcd_xyz(out/'map_enu_downsampled.pcd')
            # All frames combined, point-weighted centroid. Negative voxel uses floor.
            np.testing.assert_allclose(sparse,[[-.1,-.0625,.1],[-.1,.125,.1],[-.1,.925,.1]],atol=1e-7)
            self.assertFalse(list(out.glob('*.tmp*')))
            disabled=root/'disabled';disabled.mkdir()
            self.assertEqual(a.export_map(session,poses,rotation,translation,disabled,0),(8,None))
            self.assertFalse((disabled/'map_enu_downsampled.pcd').exists())
            np.testing.assert_array_equal(a.read_pcd_xyz(disabled/'map_enu.pcd'),full)
            with self.assertRaises(ValueError):a.export_map(session,poses,rotation,translation,out,-1)

    def test_voxel_export_failure_cleans_temporary_database(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp);session=root/'session';session.mkdir();out=root/'out';out.mkdir()
            raw=root/'raw';np.array([[1.,2.,3.]],dtype='<f4').tofile(raw)
            a.write_xyz_pcd(session/'0.pcd',raw,1)
            (session/'1.pcd').write_text('broken PCD')
            poses=np.array([[1.,0,0,0,0,0,0,1],[2.,0,0,0,0,0,0,1]])
            with self.assertRaises(ValueError):
                a.export_map(session,poses,np.eye(3),np.zeros(3),out,.2)
            self.assertFalse(list(out.glob('*.tmp*')))
            self.assertFalse((out/'map_enu.pcd').exists())

    def test_binary_padding_ascii_and_truncation(self):
        with tempfile.TemporaryDirectory() as tmp:
            p=Path(tmp)/'a.pcd'
            header=b'FIELDS x y z _\nSIZE 4 4 4 1\nTYPE F F F U\nCOUNT 1 1 1 4\nPOINTS 1\nDATA binary\n'
            p.write_bytes(header+np.array([1,2,3],dtype='<f4').tobytes()+b'\x00'*4)
            np.testing.assert_allclose(a.read_pcd_xyz(p),[[1,2,3]])
            p.write_bytes(header+b'\x00')
            with self.assertRaises(ValueError): a.read_pcd_xyz(p)
            p.write_text('FIELDS intensity x y z\nSIZE 4 4 4 4\nTYPE F F F F\nCOUNT 1 1 1 1\nPOINTS 1\nDATA ascii\n0 1 2 3\n')
            np.testing.assert_allclose(a.read_pcd_xyz(p),[[1,2,3]])


if __name__ == '__main__':
    unittest.main()
