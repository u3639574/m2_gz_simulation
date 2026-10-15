#!/usr/bin/env python3
"""Generate the teaching robot/world; SI units. Run from any working directory."""
from pathlib import Path
import math
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[1]

def child(parent, tag, text=None, **attrs):
    e = ET.SubElement(parent, tag, attrs)
    if text is not None: e.text = str(text)
    return e

def vec(values): return ' '.join(f'{v:.9g}' for v in values)

def box(parent, size):
    child(child(child(parent, 'geometry'), 'box'), 'size', vec(size))

def body(model, name, pos, size, mass, color, friction=0.7):
    link = child(model, 'link', name=name); child(link, 'pose', vec((*pos,0,0,0)))
    inertial=child(link,'inertial');child(inertial,'mass',mass); inertia=child(inertial,'inertia')
    x,y,z=size
    for key,val in [('ixx',mass*(y*y+z*z)/12),('iyy',mass*(x*x+z*z)/12),('izz',mass*(x*x+y*y)/12),('ixy',0),('ixz',0),('iyz',0)]:child(inertia,key,val)
    visual=child(link,'visual',name='visual');box(visual,size)
    mat=child(visual,'material');child(mat,'ambient',color);child(mat,'diffuse',color)
    collision=child(link,'collision',name='collision');box(collision,size)
    ode=child(child(child(collision,'surface'),'friction'),'ode');child(ode,'mu',friction);child(ode,'mu2',friction)
    return link

def joint(model,name,kind,parent,link,xyz,lower=None,upper=None,damping=0.0):
    j=child(model,'joint',name=name,type=kind);child(j,'parent',parent);child(j,'child',link)
    a=child(j,'axis');child(a,'xyz',xyz)
    if lower is not None:
        lim=child(a,'limit');child(lim,'lower',lower);child(lim,'upper',upper)
    child(child(a,'dynamics'),'damping',damping)
    return j

def plugin(parent, system, params):
    stem=''.join(('-'+c.lower() if c.isupper() else c) for c in system).lstrip('-')
    p=child(parent,'plugin',filename=f'gz-sim-{stem}-system',name=f'gz::sim::systems::{system}')
    for k,v in params:child(p,k,v)
    return p

def write(root,path):
    ET.indent(root,space='  ');ET.ElementTree(root).write(path,encoding='utf-8',xml_declaration=True)

sdf=ET.Element('sdf',version='1.10');world=child(sdf,'world',name='arm_world')
physics=child(world,'physics',name='1ms',type='ignored');child(physics,'max_step_size',0.001);child(physics,'real_time_factor',1.0)
for name in ['Physics','UserCommands','SceneBroadcaster','Contact']:plugin(world,name,[])
light=child(world,'light',type='directional',name='sun');child(light,'pose','0 0 10 0 0 0');child(light,'diffuse','0.8 0.8 0.8 1');child(light,'specular','0.2 0.2 0.2 1');child(light,'direction','-0.3 0.2 -0.9');child(light,'cast_shadows','true')
inc=child(world,'include');child(inc,'uri','model://robocon_ground')

# Exactly three nominal Earth Blocks inside Storage_Red: x=[4.5,5.5], y=[3.5,5.5].
for i,y in enumerate((3.85,4.45,5.05),1):
    m=child(world,'model',name=f'earth_block_{i}');child(m,'pose',f'4.95 {y} 0.176 0 0 0')
    body(m,'block',(0,0,0),(.35,.35,.35),.6,'0.85 0.16 0.10 1',1.0)
    plugin(m,'PosePublisher',[('publish_link_pose','false'),('publish_model_pose','true'),('use_pose_vector_msg','true'),('update_frequency',50)])

m=child(world,'model',name='two_arm_robot',canonical_link='base')
# Facing +X, 0.35 m away from the first block's gripping pose.
child(m,'pose','3.70 3.85 0.005 0 0 0')
body(m,'base',(0,0,.24),(.65,.60,.16),30,'0.18 0.20 0.24 1')
# Mast is 1.6m long. Its pivot is at z=.32; all arm coordinates are local to it.
mast=body(m,'vertical_arm',(0,0,1.12),(.12,.12,1.6),6,'0.50 0.53 0.58 1')
j=joint(m,'base_to_vertical','revolute','base','vertical_arm','0 0 1',damping=.2)
child(j,'pose','0 0 -0.8 0 0 0')
beam=body(m,'horizontal_arm',(.45,0,.47),(.90,.10,.08),3,'0.03 0.38 0.80 1')
# Frame must rotate with the mast; absolute model-axis joints would be wrong after yaw.
beam.find('pose').set('relative_to','vertical_arm');beam.find('pose').text='.45 0 -.65 0 0 0'
joint(m,'vertical_to_horizontal','prismatic','vertical_arm','horizontal_arm','0 0 1',0,1.1,.1)
for side,sign in [('left',1),('right',-1)]:
    finger=body(m,f'gripper_{side}',(.45,sign*.07,-.25),(.18,.04,.32),.5,'0.08 0.08 0.08 1',2.0)
    finger.find('pose').set('relative_to','horizontal_arm')
    # Open gap: 2*(.07+.22)-.04=.54m; closes against the .35m cube at q=+/-.125m.
    joint(m,f'gripper_{side}_joint','prismatic','horizontal_arm',f'gripper_{side}','0 1 0',0 if sign>0 else -.22,.22 if sign>0 else 0,.2)
    sensor=child(finger,'sensor',name=f'{side}_contact',type='contact');child(sensor,'always_on','true');child(sensor,'update_rate',100)
    contact=child(sensor,'contact');child(contact,'collision','collision');child(contact,'topic',f'/gripper/{side}/contacts')
    plugin(m,'JointPositionController',[('joint_name',f'gripper_{side}_joint'),('topic',f'/gripper_{side}_joint/cmd_pos'),('p_gain',400),('i_gain',0),('d_gain',15),('cmd_max',25),('cmd_min',-25)])

# Four-wheel skid steer using Gazebo's tutorial DiffDrive, without ROS wheel PIDs.
for end,x in [('front',.23),('rear',-.23)]:
    for side,y in [('left',.34),('right',-.34)]:
        name=f'{side}_{end}_wheel';l=child(m,'link',name=name);child(l,'pose',vec((x,y,.12,math.pi/2,0,0)))
        inertial=child(l,'inertial');child(inertial,'mass',1);i=child(inertial,'inertia')
        for k,v in [('ixx',.00400833),('iyy',.00400833),('izz',.0072),('ixy',0),('ixz',0),('iyz',0)]:child(i,k,v)
        for tag in ['visual','collision']:
            e=child(l,tag,name=tag);c=child(child(e,'geometry'),'cylinder');child(c,'radius',.12);child(c,'length',.07)
            if tag=='visual':mat=child(e,'material');child(mat,'ambient','.04 .04 .04 1');child(mat,'diffuse','.04 .04 .04 1')
            else:
                ode=child(child(child(e,'surface'),'friction'),'ode');child(ode,'mu',1);child(ode,'mu2',.2);child(ode,'fdir1','1 0 0')
        j=joint(m,name+'_joint','revolute','base',name,'0 0 -1')
# Cylinder local -Z is chassis +Y due to the link's 90-degree roll.
plugin(m,'DiffDrive', [('left_joint','left_front_wheel_joint'),('left_joint','left_rear_wheel_joint'),('right_joint','right_front_wheel_joint'),('right_joint','right_rear_wheel_joint'),('wheel_separation',.68),('wheel_radius',.12),('topic','/chassis/cmd_vel'),('odom_topic','/chassis/odometry'),('odom_publish_frequency',50),('max_linear_acceleration',.5),('max_angular_acceleration',.8)])
plugin(m,'JointStatePublisher',[('topic','/world/arm_world/model/two_arm_robot/joint_state')]+[('joint_name',n) for n in ['vertical_to_horizontal','base_to_vertical','gripper_left_joint','gripper_right_joint']])
for name in ['vertical_to_horizontal','base_to_vertical']:plugin(m,'ApplyJointForce',[('joint_name',name)])
plugin(m,'PosePublisher',[('publish_link_pose','false'),('publish_model_pose','true'),('use_pose_vector_msg','true'),('update_frequency',50)])

# Keys from keyboard_key.h; keep arrows out of test_mode so each has one owner.
import re
keys=dict((k,int(v)) for k,v in re.findall(r'\b([A-Z]+)\s*=\s*(\d+)',(ROOT/'code/include/m2_gz_simulation/base/keyboard_key.h').read_text()))
for key,v,w in [('UP',.2,0),('DOWN',-.2,0),('LEFT',0,.4),('RIGHT',0,-.4),('X',0,0)]:
    p=plugin(world,'TriggeredPublisher',[]);inp=child(p,'input',type='gz.msgs.Int32',topic='/keyboard/keypress');child(inp,'match',keys[key],field='data');child(p,'output',f'linear: {{x: {v}}}, angular: {{z: {w}}}',type='gz.msgs.Twist',topic='/chassis/cmd_vel')
write(sdf,ROOT/'model/two_arm_robot.sdf')
