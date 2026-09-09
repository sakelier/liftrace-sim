#!/usr/bin/env python3
"""Recorded-layout geometry screening, not a flight Gate or detector accuracy test."""
from __future__ import annotations
import argparse,csv,json,math,statistics
from pathlib import Path
from search_sim import Point3,SearchArea,M0Parameters,generate_boustrophedon
from obstacles import Obstacle,route_collisions,plan_route_astar,line_of_sight_blockers
from installed_camera import project,camera_origin,quaternion_from_rpy

def obstacles_from_scene(scene, tree_radius):
    obstacles=[]
    for wall in scene['walls']:
        xs=[p[0] for p in wall['polygon']];ys=[p[1] for p in wall['polygon']]
        obstacles.append(Obstacle(wall['id'],(min(xs)+max(xs))/2,(min(ys)+max(ys))/2,max(xs)-min(xs),max(ys)-min(ys),wall['height']))
    for tree in scene['trees']:
        obstacles.append(Obstacle(tree['id'],tree['x'],tree['y'],2*tree_radius,2*tree_radius,tree['height']))
    return obstacles

def is_free(p,obstacles,clearance):
    return not route_collisions([p,p],obstacles,clearance)

def project_free(p,obstacles,bounds,resolution,clearance,max_offset=.75):
    min_x,max_x,min_y,max_y=bounds
    ix=round((p.x-min_x)/resolution);iy=round((p.y-min_y)/resolution)
    candidates=[];steps=math.ceil(max_offset/resolution)
    for dx in range(-steps,steps+1):
        for dy in range(-steps,steps+1):
            q=Point3(min_x+(ix+dx)*resolution,min_y+(iy+dy)*resolution,p.z)
            distance=math.hypot(q.x-p.x,q.y-p.y)
            if distance<=max_offset+1e-9 and min_x<=q.x<=max_x and min_y<=q.y<=max_y:
                candidates.append((distance,q.x,q.y,q))
    for _,_,_,q in sorted(candidates):
        if is_free(q,obstacles,clearance):return q
    raise RuntimeError('No nearby free search endpoint')

def build_route(scene,strategy,obstacles,height=1.4,spacing=.7,speed=.5,clearance=.30,resolution=.05):
    a=scene['search_bounds'];area=SearchArea(a['min_x'],a['max_x'],a['min_y'],a['max_y'])
    params=M0Parameters(area,'y' if strategy.startswith('y_') else 'x',spacing,height,speed,1.0)
    base=generate_boustrophedon(params)
    bounds=(area.min_x-1,area.max_x+1,area.min_y-1,area.max_y+1)
    if strategy=='x_north_first':base=list(reversed(base))
    if strategy=='x_free_intervals':
        base=[];ny=math.ceil((area.max_y-area.min_y)/spacing)
        for row in range(ny+1):
            y=min(area.max_y,area.min_y+row*spacing)
            line=[Point3(area.min_x+i*resolution,y,height) for i in range(round((area.max_x-area.min_x)/resolution)+1)]
            segments=[];current=[]
            for q in line:
                if is_free(q,obstacles,clearance):current.append(q)
                elif current:segments.append((current[0],current[-1]));current=[]
            if current:segments.append((current[0],current[-1]))
            if row%2:segments=[(b,a) for a,b in reversed(segments)]
            for pair in segments:base.extend(pair)
    raw=[Point3(0,0,height)]+base
    original_collisions=len(route_collisions(raw,obstacles,clearance))
    blocked_endpoints=sum(not is_free(q,obstacles,clearance) for q in raw)
    if strategy=='x_raw':
        return raw,{'feasible':not original_collisions,'original_collisions':original_collisions,'blocked_endpoints':blocked_endpoints,'max_endpoint_shift_m':0,'replanned_segments':0}
    projected=[project_free(q,obstacles,bounds,resolution,clearance) for q in raw]
    shifts=[math.hypot(a.x-b.x,a.y-b.y) for a,b in zip(raw,projected)]
    unique=[projected[0]]
    for q in projected[1:]:
        if q!=unique[-1]:unique.append(q)
    plan=plan_route_astar(unique,obstacles,bounds,resolution,clearance)
    collisions=route_collisions(plan.waypoints,obstacles,clearance)
    assert not collisions
    return list(plan.waypoints),{'feasible':True,'original_collisions':original_collisions,'blocked_endpoints':blocked_endpoints,'max_endpoint_shift_m':max(shifts),'replanned_segments':plan.replanned_segment_count}

def in_camera(point,vehicle,camera,offset=-.16,body_quaternion=(0,0,0,1)):
    return project(point,vehicle,camera,body_quaternion,offset) is not None

def target_samples(target):
    x,y=target['x'],target['y'];center=Point3(x,y,0)
    if target['class']!='red_cross':return [center]+[Point3(x+dx,y+dy,0) for dx,dy in ((.5,0),(-.5,0),(0,.5),(0,-.5))]
    yaw=target['yaw'];c,s=math.cos(yaw),math.sin(yaw)
    return [center]+[Point3(x+c*dx-s*dy,y+s*dx+c*dy,0) for dx,dy in ((-.175,-.175),(-.175,.175),(.175,.175),(.175,-.175))]

def evaluate_route(route,targets,camera,obstacles,speed=.5,offset=-.16,sample_hz=10,hold=.2,body_quaternion=(0,0,0,1)):
    ids=[t['class'] for t in targets];samples={t['class']:target_samples(t) for t in targets}
    streak={(name,mode):0 for name in ids for mode in ('center','refine')}
    first={(name,mode):None for name in ids for mode in ('center','refine')}
    distance=elapsed=0.;previous_direction=None
    for start,end in zip(route,route[1:]):
        length=math.hypot(end.x-start.x,end.y-start.y)
        if length<1e-9:continue
        direction=((end.x-start.x)/length,(end.y-start.y)/length)
        if previous_direction and sum(a*b for a,b in zip(direction,previous_direction))<1-1e-8:
            elapsed+=1.0
            for key in streak:streak[key]=0
        previous_direction=direction;duration=length/speed;count=max(1,math.ceil(duration*sample_hz));dt=duration/count
        for i in range(count):
            f=(i+.5)/count;p=Point3(start.x+(end.x-start.x)*f,start.y+(end.y-start.y)*f,start.z)
            cam=Point3(*camera_origin(p,body_quaternion,offset))
            for name,points in samples.items():
                center=in_camera(points[0],p,camera,offset,body_quaternion) and not line_of_sight_blockers(cam,points[0],obstacles)
                refine=center and all(in_camera(q,p,camera,offset,body_quaternion) for q in points) and sum(not line_of_sight_blockers(cam,q,obstacles) for q in points)>=3
                for mode,visible in [('center',center),('refine',refine)]:
                    key=(name,mode);streak[key]=streak[key]+dt if visible else 0
                    if first[key] is None and streak[key]>=hold:first[key]=elapsed+(i+.5)*dt
        elapsed+=duration;distance+=length
    result={'route_distance_m':distance,'nominal_search_s':elapsed,'center_seen':sum(first[n,'center'] is not None for n in ids),'refine_seen':sum(first[n,'refine'] is not None for n in ids),
            'first_center_s':{n:first[n,'center'] for n in ids},'first_refine_s':{n:first[n,'refine'] for n in ids}}
    times=sorted(x for x in result['first_center_s'].values() if x is not None)
    result['third_center_visible_s']=times[2] if len(times)>=3 else None
    return result

def compare(scene,camera,tree_radius=.4,body_quaternion=(0,0,0,1)):
    obstacles=obstacles_from_scene(scene,tree_radius);rows=[];routes={}
    for strategy in ('x_raw','x_boundary','y_boundary','x_north_first','x_free_intervals'):
        route,meta=build_route(scene,strategy,obstacles);routes[strategy]=[[p.x,p.y,p.z] for p in route]
        for layout in scene['layouts']:
            metrics=evaluate_route(route,layout['targets'],camera,obstacles,body_quaternion=body_quaternion)
            rows.append(dict(seed=layout['seed'],strategy=strategy,**meta,**metrics))
    summary=[]
    for name in routes:
        subset=[r for r in rows if r['strategy']==name];times=[r['third_center_visible_s'] for r in subset if r['third_center_visible_s'] is not None]
        summary.append({'strategy':name,'feasible':subset[0]['feasible'],'route_distance_m':subset[0]['route_distance_m'],'nominal_search_s':subset[0]['nominal_search_s'],
                        'mean_center_seen':statistics.mean(r['center_seen'] for r in subset),'mean_refine_seen':statistics.mean(r['refine_seen'] for r in subset),
                        'layouts_three_centers':len(times),'mean_third_center_visible_s':statistics.mean(times) if times else None,
                        'blocked_original_endpoints':subset[0]['blocked_endpoints'],'max_endpoint_shift_m':subset[0]['max_endpoint_shift_m']})
    return {'scope':'OFFLINE_GEOMETRY_ONLY','assumptions':{'known_static_map':True,'target_positions_used_by_route_builder':False,'tree_bbox_radius_m':tree_radius,'clearance_m':.30,'fc_agl_m':1.40,'camera_offset_z_m':-.16,'body_quaternion_xyzw':list(body_quaternion),'attitude_mode':'constant_sensitivity_not_dynamics','nominal_speed_mps':.5,'turn_penalty_s':1.0,'visibility_hold_s':.2,'detector_probability_model':None,'flight_dynamics_or_gate_model':False},'routes':routes,'summary':summary,'rows':rows}

def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--scene',type=Path,required=True);parser.add_argument('--camera',type=Path,required=True);parser.add_argument('--output',type=Path,required=True);parser.add_argument('--tree-radius',type=float,default=.4);parser.add_argument('--roll-deg',type=float,default=0);parser.add_argument('--pitch-deg',type=float,default=0);parser.add_argument('--yaw-deg',type=float,default=0);args=parser.parse_args()
    result=compare(json.loads(args.scene.read_text()),json.loads(args.camera.read_text()),args.tree_radius,quaternion_from_rpy(*[math.radians(v) for v in (args.roll_deg,args.pitch_deg,args.yaw_deg)]))
    args.output.mkdir(parents=True,exist_ok=True);(args.output/'comparison.json').write_text(json.dumps(result,indent=2))
    with (args.output/'comparison.csv').open('w',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=result['summary'][0]);writer.writeheader();writer.writerows(result['summary'])
    print(json.dumps(result['summary'],indent=2))
if __name__=='__main__':main()
