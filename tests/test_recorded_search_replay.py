import unittest
from recorded_search_replay import in_camera,project_free
from search_sim import Point3
from obstacles import Obstacle

class RecordedReplayTest(unittest.TestCase):
    def camera(self):return {'K':[725,0,640,0,725,360,0,0,1],'D':[0,0,0,0,0],'width':1280,'height':720}
    def test_measured_camera_offset_and_fixed_body_axes(self):
        camera=self.camera();vehicle=Point3(0,0,1.4)
        self.assertTrue(in_camera(Point3(.60,0,0),vehicle,camera,-.16))
        self.assertFalse(in_camera(Point3(.64,0,0),vehicle,camera,-.16))
        self.assertTrue(in_camera(Point3(.64,0,0),vehicle,camera,-.08588))
        self.assertTrue(in_camera(Point3(0,.9,0),vehicle,camera,-.16))
        self.assertFalse(in_camera(Point3(.9,0,0),vehicle,camera,-.16))
    def test_search_endpoint_projection_avoids_obstacle_without_target_positions(self):
        obstacle=Obstacle('tree',0,0,.6,.6,1.5);p=Point3(0,0,1.4)
        q=project_free(p,[obstacle],(-2,2,-2,2),.05,.3)
        self.assertGreater(max(abs(q.x),abs(q.y)),.6)
        self.assertEqual(q.z,p.z)
        with self.assertRaises(RuntimeError):project_free(p,[obstacle],(-2,2,-2,2),.05,.3,max_offset=.15)
    def test_distortion_does_not_fold_outside_rays_into_image(self):
        camera=self.camera();camera['D']=[.00586686,.0179105,-.00100641,.00147156,-.0264851]
        for y in (2.2,2.3,2.4,5,10):
            self.assertFalse(in_camera(Point3(0,y,0),Point3(0,0,1.4),camera))
