import math,unittest
from installed_camera import project,camera_origin,quaternion_from_rpy
from search_sim import Point3

class InstalledCameraTest(unittest.TestCase):
    camera={'K':[725,0,640,0,725,360,0,0,1],'D':[0]*5,'width':1280,'height':720}
    def test_yaw_rotates_view_without_translating_mount(self):
        p=Point3(0,.9,0);body=Point3(0,0,1.4)
        self.assertIsNotNone(project(p,body,self.camera))
        self.assertIsNone(project(p,body,self.camera,quaternion_from_rpy(yaw=math.pi/2)))
        self.assertEqual(camera_origin(body,quaternion_from_rpy(yaw=1)),(0,0,1.24))
    def test_tilt_rotates_mount_and_optical_axis(self):
        pitch=math.radians(10);q=quaternion_from_rpy(pitch=pitch);body=Point3(0,0,1.4)
        origin=camera_origin(body,q)
        self.assertAlmostEqual(origin[0],-.16*math.sin(pitch))
        axis_ground=Point3(-1.4*math.tan(pitch),0,0)
        u,v=project(axis_ground,body,self.camera,q)
        self.assertAlmostEqual(u,640);self.assertAlmostEqual(v,360)
    def test_invalid_and_upward_poses(self):
        with self.assertRaises(ValueError):camera_origin(Point3(0,0,1),(0,0,0,0))
        self.assertIsNone(project(Point3(0,0,0),Point3(0,0,1.4),self.camera,quaternion_from_rpy(roll=math.pi)))
    def test_rigid_world_rotation_preserves_pixels(self):
        body=Point3(0,0,1.4);p=Point3(.2,.3,0);yaw=.73
        rotated=Point3(math.cos(yaw)*p.x-math.sin(yaw)*p.y,math.sin(yaw)*p.x+math.cos(yaw)*p.y,0)
        expected=project(p,body,self.camera)
        actual=project(rotated,body,self.camera,quaternion_from_rpy(yaw=yaw))
        for a,b in zip(expected,actual):self.assertAlmostEqual(a,b)
