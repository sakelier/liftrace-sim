"""Rigid downward camera geometry in world/body FLU; no gimbal or flight dynamics."""
import math


def quaternion_from_rpy(roll=0.0, pitch=0.0, yaw=0.0):
    cr,sr=math.cos(roll/2),math.sin(roll/2)
    cp,sp=math.cos(pitch/2),math.sin(pitch/2)
    cy,sy=math.cos(yaw/2),math.sin(yaw/2)
    return (sr*cp*cy-cr*sp*sy, cr*sp*cy+sr*cp*sy,
            cr*cp*sy-sr*sp*cy, cr*cp*cy+sr*sp*sy)


def rotation(q):
    if len(q)!=4 or not all(math.isfinite(v) for v in q):
        raise ValueError('Expected finite quaternion xyzw')
    norm=math.sqrt(sum(v*v for v in q))
    if norm<1e-12:raise ValueError('Zero quaternion')
    x,y,z,w=(v/norm for v in q)
    return ((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),
            (2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),
            (2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)))


def camera_origin(vehicle, q=(0,0,0,1), offset=-.16):
    r=rotation(q)
    return tuple(v+row[2]*offset for v,row in zip((vehicle.x,vehicle.y,vehicle.z),r))


def project(point, vehicle, camera, q=(0,0,0,1), offset=-.16):
    """Return pixels or None outside the conservative calibrated field of view.

    Optical axes in body: X=-Y, Y=-X, Z=-Z. The offset rotates with
    the body; it is not a permanent world-vertical translation when tilted.
    """
    r=rotation(q);origin=camera_origin(vehicle,q,offset)
    delta=[v-o for v,o in zip((point.x,point.y,point.z),origin)]
    body=[sum(r[j][i]*delta[j] for j in range(3)) for i in range(3)]
    depth=-body[2]
    if depth<=0:return None
    x,y=-body[1]/depth,-body[0]/depth
    k,d=camera['K'],camera['D']
    if not (0<=k[0]*x+k[2]<camera['width'] and 0<=k[4]*y+k[5]<camera['height']):return None
    r2=x*x+y*y;radial=1+d[0]*r2+d[1]*r2*r2+d[4]*r2*r2*r2
    xd=x*radial+2*d[2]*x*y+d[3]*(r2+2*x*x)
    yd=y*radial+d[2]*(r2+2*y*y)+2*d[3]*x*y
    u,v=k[0]*xd+k[2],k[4]*yd+k[5]
    return (u,v) if 0<=u<camera['width'] and 0<=v<camera['height'] else None
