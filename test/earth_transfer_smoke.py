#!/usr/bin/env python3
"""Headless contact/pose verification of the T-key demo; run after sourcing the workspace."""
import argparse
import json
import math
from pathlib import Path
import time
import rclpy
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from simulation_smoke import Simulation


def run(root, config=None):
    root.mkdir(parents=True, exist_ok=True)
    sim = Simulation('fsm', root, [f'fsm_config:={config}'] if config else [])
    stages = []; samples = []; status = ['']; result = {}
    def stage(msg):
        status[0] = msg.data
        entry = {'stage': msg.data, 'poses': dict(sim.poses)}
        stages.append(entry); print(json.dumps(entry), flush=True)
    sub = sim.node.create_subscription(String, '/m2_gz_simulation/chassis/autopath_status', stage,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        sim.wait(lambda: len(sim.state)==12 and all(n in sim.poses for n in (
            'two_arm_robot', 'earth_block_1', 'earth_block_2', 'earth_block_3')) and sim.odom is not None, 60)
        sim.spin(1)
        start = time.monotonic()
        sim.key(ord('T'))
        next_sample = 0
        while time.monotonic()-start < 150:
            rclpy.spin_once(sim.node, timeout_sec=.01)
            if time.monotonic() >= next_sample:
                samples.append({'stage': status[0], 'poses': dict(sim.poses),
                    'stamp': sim.odom.header.stamp.sec+sim.odom.header.stamp.nanosec*1e-9,
                    'vx': sim.odom.twist.twist.linear.x, 'vy': sim.odom.twist.twist.linear.y,
                    'yaw_rate': sim.odom.twist.twist.angular.z, 'gripping': sim.gripping()})
                next_sample = time.monotonic()+.1
            if status[0]=='complete' or status[0].startswith('failed:'): break
        result['status'] = status[0]
        if status[0]=='complete': sim.spin(3)
        result['final_poses'] = sim.poses.copy()
        assert status[0]=='complete', status[0]
        block = sim.poses['earth_block_1']
        # The whole 0.35 m cube must lie on the 1 m transfer square, floor z=.6.
        assert -1.625 <= block[0] <= -.975 and 3.175 <= block[1] <= 3.825, block
        assert abs(block[2]-.775)<.035, block
        assert abs(sim.state['vertical_to_horizontal'][0])<.01
        assert abs(sim.state['gripper_left_joint'][0]-.22)<.01
        assert abs(sim.odom.twist.twist.linear.x)<.015
        result['block_in_transfer_area'] = True
        assert [s['stage'] for s in stages] == ['prepare', 'pickup', 'close_grip',
            'confirm_grasp', 'raise', 'approach_red', 'turn_arm', 'ascend_red', 'release_and_lower', 'complete']
        result['duration_sim_s'] = samples[-1]['stamp']-samples[0]['stamp']
        for name in ('pickup', 'approach_red', 'ascend_red'):
            motion=[s for s in samples if s['stage']==name]
            result.setdefault('motion', {})[name] = {
                'duration_s': motion[-1]['stamp']-motion[0]['stamp'],
                'peak_speed_m_s': max(math.hypot(s['vx'],s['vy']) for s in motion)}
        approach=[s['poses']['two_arm_robot'] for s in samples if s['stage']=='approach_red']
        x0,y0=approach[0][:2]; x1,y1=approach[-1][:2]
        deviation=max(abs((x1-x0)*(y-y0)-(y1-y0)*(x-x0))/math.hypot(x1-x0,y1-y0)
                      for x,y,*_ in approach)
        assert deviation > .025, ('expected a curved approach', deviation)
        result['approach_deviation_from_chord_m'] = deviation
        # The other two storage blocks must remain undisturbed.
        for i, y in [(2,4.45),(3,5.05)]:
            p=sim.poses[f'earth_block_{i}']; assert abs(p[0]-4.95)<.01 and abs(p[1]-y)<.01
    finally:
        result.update(stages=stages, samples=samples)
        (root/'report.json').write_text(json.dumps(result, indent=2))
        sim.close()
    return result

if __name__=='__main__':
    parser=argparse.ArgumentParser(); parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--config',type=Path); args=parser.parse_args()
    run(args.output.resolve(), args.config)
