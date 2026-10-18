"""Converter contract checks using minimal ROS message mocks (ROS1 not required)."""
import csv
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import ModuleType, SimpleNamespace as S
import unittest
from unittest.mock import patch
import numpy as np


def odometry():
    return S(header=S(), child_frame_id='', pose=S(pose=S(position=S(),orientation=S()),covariance=[]))


class RecordingTests(unittest.TestCase):
    def test_manual_datum_recording(self):
        self.check_recording(False)

    def test_auto_datum_recording(self):
        self.check_recording(True)

    def check_recording(self, auto):
        with tempfile.TemporaryDirectory() as tmp:
            params={'~record_dir':tmp,'~use_auto_datum':auto,'~datum_lat':22.,
                    '~datum_lon':113.,'~datum_alt':10.,'~min_T_intervel':10.}
            ros=ModuleType('rospy')
            ros.init_node=lambda *a,**k: None
            ros.get_param=lambda key,default=None: params.get(key,default)
            ros.on_shutdown=lambda callback: None
            ros.loginfo=lambda *a,**k: None
            ros.logwarn_throttle=lambda *a,**k: None
            published=[]
            ros.Publisher=lambda *a,**k: S(publish=published.append)
            ros.Subscriber=lambda *a,**k: None
            sensor=ModuleType('sensor_msgs.msg'); sensor.NavSatFix=S(COVARIANCE_TYPE_UNKNOWN=0)
            nav=ModuleType('nav_msgs.msg'); nav.Odometry=odometry
            with patch.dict(sys.modules,{'rospy':ros,'sensor_msgs':ModuleType('sensor_msgs'),
                                         'sensor_msgs.msg':sensor,'nav_msgs':ModuleType('nav_msgs'),'nav_msgs.msg':nav}):
                path=Path(__file__).resolve().parents[1]/'scripts'/'gnss_to_enu.py'
                spec=importlib.util.spec_from_file_location('converter',path)
                module=importlib.util.module_from_spec(spec); spec.loader.exec_module(module)
                node=module.GNSSToENU()
                def fix(t, lon=113.):
                    return S(latitude=22., longitude=lon, altitude=10.,
                             header=S(stamp=S(to_sec=lambda:t)),status=S(status=0),
                             position_covariance=[.0004,0,0,0,.0004,0,0,0,.0016],position_covariance_type=2)
                if auto:
                    self.assertFalse((Path(tmp)/'datum.json').exists())
                    invalid_first=fix(1699999999.)
                    invalid_first.status.status=-1
                    node.gnss_cb(invalid_first)
                    self.assertFalse(node.has_origin)
                first=fix(1700000000.)
                node.gnss_cb(first)
                node.gnss_cb(fix(1700000000.1,113.000001))
                node.gnss_cb(fix(1700000000.1))  # duplicate
                invalid=fix(1700000000.2); invalid.latitude=float('nan'); node.gnss_cb(invalid)
                unknown=fix(1700000000.3); unknown.position_covariance_type=0; node.gnss_cb(unknown)
                node.close_recording()
                with (Path(tmp)/'rtk_enu.csv').open() as stream: rows=list(csv.DictReader(stream))
                self.assertEqual(len(rows),2)
                self.assertEqual(len(published),1)
                self.assertIs(published[0].header.stamp,first.header.stamp)
                self.assertEqual(published[0].header.frame_id,'enu')
                self.assertAlmostEqual(float(rows[0]['var_u']),.0016)
                np.testing.assert_allclose([float(rows[0][k]) for k in ('east','north','up')],0.,atol=1e-8)
                self.assertGreater(float(rows[1]['east']),0.)
                datum=json.loads((Path(tmp)/'datum.json').read_text())
                self.assertEqual(datum['latitude'],22.)
                self.assertEqual(datum['longitude'],113.)
                self.assertEqual(datum['altitude'],10.)
                self.assertEqual(datum['use_auto_datum'],auto)
                with self.assertRaises(RuntimeError): module.GNSSToENU()


if __name__ == '__main__': unittest.main()
