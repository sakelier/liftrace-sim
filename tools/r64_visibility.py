#!/usr/bin/env python3
"""Pose-aware geometric visibility, not detector accuracy or a flight simulator."""
import argparse,json,sys,math
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from installed_camera import project
from search_sim import Point3

def analyze(data):
    output=[]
    for case in data['cases']:
        counts={t['class']:{'center_visible_samples':0,'full_circle_visible_samples':0} for t in case['targets']}
        for row in case['track']:
            if row[8] not in ['SEARCH','EXECUTING']:continue
            vehicle=Point3(*row[1:4]);q=row[4:8]
            for target in case['targets']:
                x,y=target['world_x'],target['world_y'];c=counts[target['class']]
                if project(Point3(x,y,0),vehicle,data['camera'],q,data['fc_camera_offset_z']) is not None:c['center_visible_samples']+=1
                if target['class']!='red_cross' and all(project(Point3(x+.5*math.cos(k*math.pi/8),y+.5*math.sin(k*math.pi/8),0),vehicle,data['camera'],q,data['fc_camera_offset_z']) is not None for k in range(16)):c['full_circle_visible_samples']+=1
        output.append({'seed':case['seed'],'gate':case['gate'],'target_wall_overlap':case['target_wall_overlap'],'sample_counts':counts})
    return {'scope':'Approximate 1Hz geometric frustum samples; no wall/tree occlusion or detector probability implied','cases':output}

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--input',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args();a.output.write_text(json.dumps(analyze(json.loads(a.input.read_text())),indent=2))
