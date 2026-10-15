#!/usr/bin/env python3
"""Static mechanics/topic invariants that physics smoke tests depend on."""
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET
import yaml
ROOT = Path(__file__).resolve().parents[1]


class WorldContract(unittest.TestCase):
    def setUp(self):
        self.world = ET.parse(ROOT/'model/two_arm_robot.sdf').getroot().find('world')
        self.robot = self.world.find("model[@name='two_arm_robot']")

    def test_three_nominal_blocks_fully_inside_storage(self):
        blocks = [m for m in self.world.findall('model') if m.get('name').startswith('earth_block_')]
        self.assertEqual(len(blocks),3)
        for m in blocks:
            x,y,z,*_ = map(float,m.findtext('pose').split())
            self.assertEqual(m.findtext('link/collision/geometry/box/size'),'0.35 0.35 0.35')
            self.assertEqual(float(m.findtext('link/inertial/mass')),.6)
            self.assertTrue(4.5 <= x-.175 < x+.175 <= 5.5)
            self.assertTrue(3.5 <= y-.175 < y+.175 <= 5.5)

    def test_gripper_clearance_and_friction(self):
        left=self.robot.find("link[@name='gripper_left']")
        right=self.robot.find("link[@name='gripper_right']")
        left_y=float(left.findtext('pose').split()[1]);right_y=float(right.findtext('pose').split()[1])
        width=float(left.findtext('collision/geometry/box/size').split()[1])
        opening=float(self.robot.findtext("joint[@name='gripper_left_joint']/axis/limit/upper"))
        self.assertAlmostEqual(left_y-right_y+2*opening-width,.54)
        self.assertGreater(left_y-right_y+2*opening-width,.35*1.05)
        self.assertEqual(float(left.findtext('collision/surface/friction/ode/mu')),2.0)

    def test_actuator_ownership_and_four_wheels(self):
        self.assertIsNone(self.robot.find("joint[@name='world_to_base']"))
        self.assertEqual(self.robot.find("joint[@name='base_to_vertical']").get('type'),'revolute')
        drive=self.robot.find("plugin[@name='gz::sim::systems::DiffDrive']")
        self.assertEqual(len(drive.findall('left_joint')),2)
        self.assertEqual(len(drive.findall('right_joint')),2)
        self.assertIsNone(self.robot.find("plugin[@name='gz::sim::systems::JointController']"))
        self.assertEqual(len(self.robot.findall("plugin[@name='gz::sim::systems::ApplyJointForce']")),2)
        for e in self.robot.findall('joint'):
            self.assertNotIn('world',[e.findtext('parent'),e.findtext('child')])

    def test_bridge_contacts_and_force_topics(self):
        bridge=yaml.safe_load((ROOT/'config/ros_gz_bridge.yaml').read_text())
        gz_topics={row['gz_topic_name'] for row in bridge}
        for name in ('vertical_to_horizontal','base_to_vertical'):
            self.assertIn(f'/model/two_arm_robot/joint/{name}/cmd_force',gz_topics)
        for side in ('left','right'):
            self.assertIn(self.robot.findtext(f"link[@name='gripper_{side}']/sensor/contact/topic"),gz_topics)
        self.assertNotIn('/vertical_to_horizontal/cmd_vel',gz_topics)

    def test_no_fixed_sky_blocks_in_imported_scene(self):
        tree=ET.parse(ROOT/'model/robocon_ground/meshes/venue.dae')
        ns={'c':'http://www.collada.org/2005/11/COLLADASchema'}
        names={e.get('name') for e in tree.findall('.//c:visual_scene/c:node',ns)}
        self.assertIn('Storage_Red',names)
        self.assertTrue(all(f'Cube.{i:03d}' not in names for i in range(20,44)))

if __name__=='__main__':unittest.main()
