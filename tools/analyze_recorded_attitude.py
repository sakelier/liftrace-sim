#!/usr/bin/env python3
"""Summarize recorded body attitude/course; never claims detection or flight PASS."""
import argparse,csv,json,math
from pathlib import Path

def angle_diff(a,b):return math.atan2(math.sin(a-b),math.cos(a-b))

def summarize(path,min_z=.5,min_speed=.2):
    poses=[]
    for row in csv.DictReader(path.open()):
        p={k:float(row[k]) for k in ['t','x','y','z','qx','qy','qz','qw']}
        if not all(math.isfinite(v) for v in p.values()):continue
        norm=math.sqrt(sum(p[k]**2 for k in ['qx','qy','qz','qw']))
        if norm<1e-12:continue
        x,y,z,w=(p[k]/norm for k in ['qx','qy','qz','qw'])
        p['roll']=math.atan2(2*(w*x+y*z),1-2*(x*x+y*y))
        p['pitch']=math.asin(max(-1,min(1,2*(w*y-z*x))))
        p['yaw']=math.atan2(2*(w*z+x*y),1-2*(y*y+z*z));poses.append(p)
    selected=[p for p in poses if p['z']>=min_z];differences=[]
    for a,b in zip(poses,poses[1:]):
        dt=b['t']-a['t'];dx=b['x']-a['x'];dy=b['y']-a['y']
        if 0<dt<.5 and min(a['z'],b['z'])>=min_z and math.hypot(dx,dy)/dt>=min_speed:
            differences.append(abs(math.degrees(angle_diff(b['yaw'],math.atan2(dy,dx)))))
    def stats(values):
        s=sorted(values)
        return {'samples':len(s),'median':s[len(s)//2] if s else None,'p95':s[min(len(s)-1,int(.95*len(s)))] if s else None,'max':max(s) if s else None}
    return {'source':str(path),'scope':'recorded_attitude_not_camera_detection','filter_raw_pose_z_min':min_z,'moving_speed_min_mps':min_speed,
            'roll_abs_deg':stats([abs(math.degrees(p['roll'])) for p in selected]),'pitch_abs_deg':stats([abs(math.degrees(p['pitch'])) for p in selected]),
            'yaw_deg':stats([math.degrees(p['yaw']) for p in selected]),'moving_course_yaw_abs_difference_deg':stats(differences),
            'moving_course_difference_over_30deg_fraction':sum(v>30 for v in differences)/len(differences) if differences else None}

if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('pose_csv',type=Path);parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--min-z',type=float,default=.5);args=parser.parse_args()
    result=summarize(args.pose_csv,args.min_z);args.output.parent.mkdir(parents=True,exist_ok=True);args.output.write_text(json.dumps(result,indent=2));print(json.dumps(result,indent=2))
