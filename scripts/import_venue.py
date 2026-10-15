#!/usr/bin/env python3
"""Reproduce the local field import. Upstream commit is recorded in SOURCE.md."""
from pathlib import Path
import xml.etree.ElementTree as E
import shutil
ROOT = Path(__file__).resolve().parents[1]
workspace = ROOT.parents[1]
import argparse
parser = argparse.ArgumentParser(description='Import the pinned upstream venue, removing fixed game objects')
parser.add_argument('--source', type=Path, default=workspace/'vendor/gazebo_models')
args = parser.parse_args()
import subprocess
expected = '1391b36e4707cdfc7071d6ad5509bc9a8b828603'
actual = subprocess.check_output(['git', '-C', str(args.source), 'rev-parse', 'HEAD'], text=True).strip()
if actual != expected:
    parser.error(f'Expected pinned venue commit {expected}, got {actual}')
src = args.source/'robocon_ground'
dst = ROOT/'model/robocon_ground'
(dst/'meshes').mkdir(parents=True, exist_ok=True)
ns={'c':'http://www.collada.org/2005/11/COLLADASchema'};E.register_namespace('',ns['c'])
tree=E.parse(src/'meshes/Untitled.dae');root=tree.getroot();scene=root.find('.//c:visual_scene',ns)
for n in list(scene):
 name=n.get('name','')
 if name.startswith('Cube.') and int(name.split('.')[1])>=20 or name=='Plane.015':scene.remove(n)
tree.write(dst/'meshes/venue.dae',encoding='utf-8',xml_declaration=True)
# Static triangle collisions follow the imported solid geometry, including ramps/stairs.
# Remove marking planes and floors; a dedicated floor box avoids duplicate contacts.
for n in list(scene):
 if not n.get('name','').startswith(('Cube','Cylinder','Boundary_Wall')):scene.remove(n)
tree.write(dst/'meshes/collision.dae',encoding='utf-8',xml_declaration=True)
(dst/'model.config').write_text('<?xml version="1.0"?><model><name>robocon_ground</name><version>1.0</version><sdf version="1.10">model.sdf</sdf><description>Imported 2027 field, no game objects</description></model>')
(dst/'model.sdf').write_text('''<?xml version="1.0"?>
<sdf version="1.10"><model name="robocon_ground"><static>true</static><link name="venue">
<visual name="field"><geometry><mesh><uri>model://robocon_ground/meshes/venue.dae</uri></mesh></geometry></visual>
<collision name="solids"><geometry><mesh><uri>model://robocon_ground/meshes/collision.dae</uri></mesh></geometry></collision>
<collision name="floor"><pose>0 0 -0.01 0 0 0</pose><geometry><box><size>11 11 0.02</size></box></geometry><surface><friction><ode><mu>1</mu><mu2>1</mu2></ode></friction></surface></collision>
</link></model></sdf>''')
(dst/'SOURCE.md').write_text('''Source: https://github.com/jimi4-lei/gazebo_models
Pinned commit: 1391b36e4707cdfc7071d6ad5509bc9a8b828603
Original: robocon_ground/meshes/Untitled.dae. Local changes: removed fixed Sky Block nodes Cube.020–Cube.043 and Plane.015; collision mesh retains solid venue geometry, omits markings; added floor box. This is the supplied community field model, not a claim of a certified competition digital twin. Upstream provides no explicit license file; attribution is retained for this requested local integration.
''')
shutil.copy(args.source/'README.md',dst/'UPSTREAM_README.md')
