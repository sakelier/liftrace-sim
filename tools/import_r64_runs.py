#!/usr/bin/env python3
"""Import compact real SITL traces; no fitting failure samples into successes."""
import argparse,bisect,csv,json,sys,xml.etree.ElementTree as ET
from pathlib import Path
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from r2026_scene import rectangle_vertices,polygons_overlap

p=argparse.ArgumentParser();p.add_argument('--matrix',type=Path,required=True);p.add_argument('--pilot',type=Path,required=True);p.add_argument('--output',type=Path,required=True);a=p.parse_args()
matrix=json.loads(a.matrix.read_text());assert matrix['status']=='COMPLETE'
runs=[(11,a.pilot)]+[(x['seed'],Path(x['run_dir'])) for x in matrix['results']];cases=[]
for seed,run in runs:
    gate=json.loads((run/'gate_status.json').read_text());events=[json.loads(l) for l in (run/'key_events.jsonl').read_text().splitlines()]
    phases=sorted((e['ros_sec'],e['data'].get('phase','IDLE')) for e in events if e['kind']=='mission');times=[x[0] for x in phases]
    track=[];last_t=-1e9
    for row in csv.DictReader((run/'truth_pose.csv').open()):
        t=float(row['t'])
        if t-last_t<.98:continue
        last_t=t;j=bisect.bisect_right(times,t)-1
        track.append([round(float(row[k])+( .22 if k=='z' else 0),6) for k in ['t','x','y','z','qx','qy','qz','qw']]+[phases[j][1] if j>=0 else 'IDLE'])
    targets=yaml.safe_load((run/'random_field_truth.yaml').read_text())['targets'];walls=[];world=ET.parse(run/'scenario_inputs/field.world').getroot();field=next(x for x in world.iter('model') if x.get('name')=='toudi2')
    for wall in field.findall('link'):
        if not wall.get('name','').startswith('Wall'):continue
        pose=[float(x) for x in wall.findtext('pose').split()];size=[float(x) for x in wall.findtext('collision/geometry/box/size').split()];walls.append({'name':wall.get('name'),'center':pose[:2],'size':size[:2],'height':size[2]})
    overlap=[]
    for t in targets:
        side=.35 if t['class']=='red_cross' else 1.0;poly=rectangle_vertices(t['world_x'],t['world_y'],side,side,t['yaw'])
        for w in walls:
            if polygons_overlap(poly,rectangle_vertices(*w['center'],*w['size'])):overlap.append({'target':t['class'],'wall':w['name']})
    cases.append({'seed':seed,'source':json.loads((run/'scenario_inputs/source.json').read_text())['head'],'run':run.name,'gate':gate['status'],'gate_errors':gate['errors'],'releases':gate['metrics']['release_commit_count'],'mission_ros_sec':gate['metrics']['mission_ros_sec'],'targets':targets,'walls':walls,'target_wall_overlap':overlap,'track_columns':['ros_sec','x','y','fc_agl','qx','qy','qz','qw','phase'],'track_hz_approx':1,'track':track})
out={'schema_version':1,'scope':'Recorded SITL geometry only; not hardware timing or detection probability','fc_camera_offset_z':-.16,'body_dimensions':[.5,.5,.37],'simulation_envelope':[.55,.55,.4],'camera':json.loads((a.pilot/'actual_camera_info.json').read_text()),'cases':sorted(cases,key=lambda c:c['seed'])}
a.output.parent.mkdir(parents=True,exist_ok=True);a.output.write_text(json.dumps(out,separators=(',',':')))
print(json.dumps([{'seed':c['seed'],'gate':c['gate'],'overlap':c['target_wall_overlap']} for c in cases],indent=2))
