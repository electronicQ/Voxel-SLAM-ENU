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


class AlignmentTests(unittest.TestCase):
    def test_ros_parameter_entrypoint(self):
        params={'~session':'/data/session with spaces', '~rtk':'/data/rtk.csv',
                '~output':'/data/enu', '~lever_arm':[.1,-.2,.3], '~alignment_only':True,
                '~sample_distance':2., '~datum':'/data/datum.json'}
        ros=SimpleNamespace(init_node=lambda name:None, loginfo=lambda msg:None,
                            get_param=lambda key,*default:params[key] if key in params else default[0])
        with patch.dict('sys.modules', {'rospy':ros}), patch.object(a,'main') as main:
            a.run_ros()
            argv=main.call_args[0][0]
            self.assertEqual(argv[argv.index('--session')+1],'/data/session with spaces')
            self.assertEqual(argv[argv.index('--lever-arm')+1:argv.index('--lever-arm')+4],['0.1','-0.2','0.3'])
            self.assertIn('--alignment-only',argv)
            self.assertEqual(argv[argv.index('--sample-distance')+1],'2.0')
            self.assertEqual(argv[argv.index('--datum')+1],'/data/datum.json')
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
            (root/'datum.json').write_text(json.dumps({'latitude':22.,'longitude':113.,'altitude':10.}))
            with (root/'rtk_enu.csv').open('w') as f:
                w=csv.writer(f); w.writerow(['timestamp','east','north','up','var_e','var_n','var_u'])
                # Use midpoint times to exercise interpolation rather than exact matches.
                for i in range(39):
                    time=(stamp[i]+stamp[i+1])/2
                    p,rot=a.interpolate(poses,time,.25)
                    enu=r@(p+rot@lever)+t
                    w.writerow([format(time,'.9f'),*enu,.0004,.0004,.0016])
            out=root/'output'
            args=['--session',str(session),'--rtk',str(root/'rtk_enu.csv'),'--output',str(out),
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
            self.assertEqual(json.loads((out/'alignment.json').read_text())['status'],'complete')
            self.assertTrue((out/'trajectory_comparison.png').stat().st_size > 10000)
            self.assertTrue(json.loads((out/'diagnostic.json').read_text())['candidate_validated'])
            with self.assertRaises(ValueError):
                a.main(args)
            (session/'3.pcd').unlink()
            args[args.index('--output')+1]=str(root/'missing')
            with self.assertRaises(ValueError):
                a.main(args)
            self.assertFalse((root/'missing'/'map_enu.pcd').exists())
            self.assertEqual(json.loads((root/'missing'/'alignment.json').read_text())['status'],'map_export_failed')

    def test_failure_diagnostics(self):
        with tempfile.TemporaryDirectory() as tmp:
            root=Path(tmp)
            session=root/'session'; session.mkdir()
            n=40
            stamps=1700000000.+np.arange(n)*.1
            angle=np.linspace(0,2*np.pi,n)
            xyz=np.column_stack([10*np.cos(angle),8*np.sin(angle),angle*.1])
            (root/'datum.json').write_text(json.dumps({'latitude':22.,'longitude':113.,'altitude':10.}))
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
                    with self.assertRaises(ValueError), contextlib.redirect_stdout(io.StringIO()):
                        a.main(args)
                    self.assertTrue((output/'trajectory_comparison.png').stat().st_size > 10000)
                    report=json.loads((output/'alignment.json').read_text())
                    self.assertEqual(report['status'],'alignment_failed')
                    self.assertIsNone(report['T_enu_map'])
                    diag=json.loads((output/'diagnostic.json').read_text())
                    self.assertFalse(diag['candidate_validated'])
                    self.assertFalse((output/'T_enu_map.txt').exists())
                    self.assertFalse((output/'map_enu.pcd').exists())
                    self.assertTrue((output/'matched_pairs.csv').exists())
                    if case in ('degenerate','no_time_overlap','no_valid_rtk'):
                        self.assertIsNone(diag['candidate_T_enu_map'])
                        self.assertFalse((output/'residuals.csv').exists())
                    else:
                        self.assertIsNotNone(diag['candidate_T_enu_map'])
                        self.assertTrue((output/'residuals.csv').exists())

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
