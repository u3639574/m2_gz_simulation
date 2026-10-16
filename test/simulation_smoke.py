#!/usr/bin/env python3
"""Real Gazebo regressions: field, cube grasp, cascade, swerve and raw sensors.
Source install/setup.bash, then run this script. No GUI is required.
"""
import argparse
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState, Imu, LaserScan
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from ros_gz_interfaces.msg import Contacts
from nav_msgs.msg import Odometry


class Simulation:
    def __init__(self, mode, root):
        os.environ['ROS_DOMAIN_ID'] = '175'
        os.environ['GZ_PARTITION'] = f'm2_robocon_{os.getpid()}_{mode}'
        os.environ['ROS_LOG_DIR'] = str(root / 'ros_logs')
        for key in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT',
                    'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'):
            os.environ.pop(key, None)
        self.log_path = root / f'{mode}.log'
        self.log = self.log_path.open('w')
        self.process = subprocess.Popen([
            'ros2', 'launch', 'm2_gz_simulation', 'simulation.launch.py',
            'gui:=false', f'command_source:={mode}'], stdout=self.log,
            stderr=subprocess.STDOUT, start_new_session=True)
        rclpy.init(); self.node = rclpy.create_node('robocon_smoke_' + mode)
        self.state = {}; self.poses = {}; self.contact = {}; self.effort = {}; self.velocity_target = {}
        self.odom = None; self.imu = None; self.scan = None; self.subs = []
        for msg_type,topic,attr in [(Imu,'imu','imu'),(LaserScan,'scan','scan')]:
            self.subs.append(self.node.create_subscription(msg_type,
                '/m2_gz_simulation/'+topic, lambda msg,a=attr: setattr(self,a,msg), qos_profile_sensor_data))
        self.subs.append(self.node.create_subscription(JointState,
            '/m2_gz_simulation/joint_states', self.joints, qos_profile_sensor_data))
        for name in ('two_arm_robot', 'earth_block_1', 'earth_block_2', 'earth_block_3'):
            self.subs.append(self.node.create_subscription(TFMessage,
                '/m2_gz_simulation/poses/' + name, self.pose, qos_profile_sensor_data))
        for side in ('left', 'right'):
            self.subs.append(self.node.create_subscription(Contacts,
                f'/m2_gz_simulation/gripper/{side}/contacts',
                lambda msg, s=side: self.contacts(s, msg), qos_profile_sensor_data))
        for axis in ('lift', 'yaw'):
            self.subs.append(self.node.create_subscription(Float64,
                f'/m2_gz_simulation/{axis}/cmd_effort',
                lambda msg, a=axis: self.effort.update({a: msg.data}), 10))
            self.subs.append(self.node.create_subscription(Float64,
                f'/m2_gz_simulation/{axis}/velocity_target',
                lambda msg, a=axis: self.velocity_target.update({a: msg.data}), 10))
        self.subs.append(self.node.create_subscription(Odometry,
            '/m2_gz_simulation/chassis/odometry',
            lambda msg: setattr(self, 'odom', msg), qos_profile_sensor_data))
        self.drive = self.node.create_publisher(Twist, '/m2_gz_simulation/chassis/cmd_vel', 1)
        self.publishers = {}

    def joints(self, msg):
        for name, position, velocity in zip(msg.name, msg.position, msg.velocity):
            self.state[name] = (position, velocity)

    def pose(self, msg):
        for t in msg.transforms:
            q = t.transform.rotation
            yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
            self.poses[t.child_frame_id] = (
                t.transform.translation.x, t.transform.translation.y,
                t.transform.translation.z, yaw)

    def contacts(self, side, msg):
        names = {name.split('::')[0] for c in msg.contacts
                 for name in (c.collision1.name, c.collision2.name) if name.startswith('earth_block_')}
        self.contact[side] = (names, time.monotonic())

    def gripping(self):
        now = time.monotonic()
        return all(side in self.contact and 'earth_block_1' in self.contact[side][0] and
                   now-self.contact[side][1] < .25 for side in ('left', 'right'))

    def wait(self, predicate, timeout=20):
        deadline = time.monotonic()+timeout
        while time.monotonic() < deadline:
            if self.process.poll() is not None:
                raise AssertionError(f'Launch exited; see {self.log_path}')
            rclpy.spin_once(self.node, timeout_sec=.005)
            if predicate(): return
        raise AssertionError(f'Timeout: state={self.state}, poses={self.poses}, effort={self.effort}')

    def spin(self, seconds):
        until = time.monotonic()+seconds
        self.wait(lambda: time.monotonic() >= until, seconds+3)

    def key(self, code):
        response = subprocess.run(['gz', 'topic', '-t', '/keyboard/keypress',
            '-m', 'gz.msgs.Int32', '-p', f'data: {code}'], capture_output=True, text=True, timeout=5)
        assert response.returncode == 0, response.stderr
        self.spin(.05)

    def scalar(self, topic, value):
        if topic not in self.publishers:
            pub = self.node.create_publisher(Float64, '/m2_gz_simulation/'+topic, 1)
            self.publishers[topic] = pub
            self.wait(lambda: pub.get_subscription_count() > 0)
        self.publishers[topic].publish(Float64(data=float(value)))

    def settled(self, name, goal, tolerance=.005, angular=False):
        q, v = self.state[name]
        error = math.remainder(q-goal, 2*math.pi) if angular else q-goal
        return abs(error)<tolerance and abs(v)<.01

    def world_control(self, request):
        completed = subprocess.run(['gz', 'service', '-s', '/world/arm_world/control',
            '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000', '--req', request], capture_output=True, text=True, timeout=5)
        assert completed.returncode == 0 and 'data: true' in completed.stdout

    def close(self):
        self.drive.publish(Twist())
        self.node.destroy_node(); rclpy.shutdown()
        if self.process.poll() is None: self.process.send_signal(signal.SIGINT)
        try: self.process.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(self.process.pid, signal.SIGKILL); self.process.wait(timeout=5)
        self.log.close()
        text = self.log_path.read_text()
        assert self.process.returncode == 0, text[-3000:]
        assert 'process has died' not in text and 'escalating to' not in text, text[-3000:]
        assert 'Error in destruction' not in text, text[-3000:]


def run(mode, root):
    sim = Simulation(mode, root); result = {}
    try:
        sim.wait(lambda: len(sim.state)==12 and len(sim.poses)==4 and sim.odom is not None, 60)
        sim.wait(lambda: sim.imu is not None and sim.scan is not None, 30)
        sim.spin(1)
        assert len(sim.scan.ranges)==720
        finite_ranges=[r for r in sim.scan.ranges if math.isfinite(r)]
        assert len(finite_ranges)>100 and min(finite_ranges)>=sim.scan.range_min
        assert sim.scan.header.frame_id and sim.imu.header.frame_id
        assert abs(sim.imu.linear_acceleration.z-9.8)<.5
        # The middle ray looks along +X at the first block's front face.
        # This also catches a sensor accidentally mounted above the blocks.
        expected=sim.poses['earth_block_1'][0]-.175-(sim.poses['two_arm_robot'][0]+.34)
        front=sim.scan.ranges[360]
        assert abs(front-expected)<.03, ('front block range',front,expected)
        stamp=lambda msg: msg.header.stamp.sec+msg.header.stamp.nanosec*1e-9
        scan_stamp,imu_stamp=stamp(sim.scan),stamp(sim.imu)
        sim.spin(.5)
        assert stamp(sim.scan)>scan_stamp and stamp(sim.imu)>imu_stamp
        result['sensors']={'scan_samples':len(sim.scan.ranges), 'finite_returns':len(finite_ranges),
                           'front_block_range_m':front,'expected_front_block_range_m':expected,
                           'imu_frame':sim.imu.header.frame_id,'lidar_frame':sim.scan.header.frame_id}
        for i, y in enumerate((3.85, 4.45, 5.05), 1):
            pose = sim.poses[f'earth_block_{i}']
            assert abs(pose[0]-4.95)<.005 and abs(pose[1]-y)<.005
            assert abs(pose[2]-.175)<.005
        result['three_blocks_on_storage_floor'] = True
        if mode == 'fsm':
            sim.wait(lambda: sim.settled('gripper_left_joint',.22))
            # Stage the robot at the first block. The tested arrow mapping is below.
            msg = Twist(); msg.linear.x=.15; sim.drive.publish(msg)
            sim.wait(lambda: sim.poses['two_arm_robot'][0] >= 4.04, 10)
            sim.drive.publish(Twist()); sim.spin(.7)
            initial_height = sim.poses['earth_block_1'][2]
            sim.key(81)  # Q: contact-confirmed close -> raise
            sim.wait(lambda: sim.gripping())
            sim.wait(lambda: sim.settled('vertical_to_horizontal',1.1), 20)
            sim.spin(1)
            result['lifted_block_height_m'] = sim.poses['earth_block_1'][2]
            assert result['lifted_block_height_m']-initial_height > 1.05
            assert sim.gripping()
            result['loaded_lift_effort_N'] = sim.effort['lift']
            assert 40 < sim.effort['lift'] < 55
            # Full yaw presets with a real payload retained by friction.
            sim.key(70)  # F: 180 degrees
            sim.wait(lambda: sim.settled('base_to_vertical',math.pi,angular=True), 20)
            assert sim.poses['earth_block_1'][2] > 1.20 and sim.gripping()
            sim.key(82)  # R: zero
            sim.wait(lambda: sim.settled('base_to_vertical',0,angular=True), 20)
            assert sim.poses['earth_block_1'][2] > 1.20 and sim.gripping()
            result['payload_retained_through_yaw'] = True
            # Lower while still gripping, then release; also test basic S/E/W/D.
            sim.key(83); sim.wait(lambda: sim.settled('vertical_to_horizontal',0))
            sim.key(69); sim.wait(lambda: sim.settled('gripper_left_joint',.22))
            sim.wait(lambda: abs(sim.poses['earth_block_1'][2]-.175)<.01)
            sim.key(87); sim.wait(lambda: sim.settled('vertical_to_horizontal',1.1))
            sim.key(68); sim.wait(lambda: sim.settled('gripper_left_joint',0))
            sim.key(65)  # A: parallel open and lower
            sim.wait(lambda: sim.state['vertical_to_horizontal'][1]<-.01 and sim.state['gripper_left_joint'][0]>.01)
            sim.wait(lambda: sim.settled('vertical_to_horizontal',0) and sim.settled('gripper_left_joint',.22))
            result['parallel_open_and_lower'] = True
            # Degree API validates range before scheduling. 359 -> 1 uses short arc.
            sim.scalar('yaw/target_degrees',359)
            sim.wait(lambda: sim.settled('base_to_vertical',math.radians(359),angular=True))
            before = sim.state['base_to_vertical'][0]
            sim.scalar('yaw/target_degrees',1)
            sim.wait(lambda: sim.settled('base_to_vertical',math.radians(1),angular=True))
            assert abs(sim.state['base_to_vertical'][0]-before)<.15
            sim.scalar('yaw/target_degrees',360); sim.spin(.3)
            assert sim.settled('base_to_vertical',math.radians(1),angular=True)
            result['degree_api_shortest_path_and_rejection'] = True
            # Swerve keys must move the real body, not just wheel odometry.
            for key, vx, vy, wz in ((16777235,1,0,0),(16777237,-1,0,0),
                                     (16777234,0,1,0),(16777236,0,-1,0),(90,0,0,1),(67,0,0,-1)):
                before = sim.poses['two_arm_robot']
                sim.key(key)
                sim.wait(lambda: (sim.odom.twist.twist.linear.x*vx>.05 if vx else
                                  sim.odom.twist.twist.linear.y*vy>.05 if vy else
                                  sim.odom.twist.twist.angular.z*wz>.1), 10)
                sim.spin(.6)
                after = sim.poses['two_arm_robot']
                dx=(after[0]-before[0])*math.cos(before[3])+(after[1]-before[1])*math.sin(before[3])
                dy=-(after[0]-before[0])*math.sin(before[3])+(after[1]-before[1])*math.cos(before[3])
                dyaw=math.remainder(after[3]-before[3],2*math.pi)
                if vx or vy:
                    assert dx*vx+dy*vy > .03, (key,before,after)
                    assert abs(dyaw)<.08, (key,'translation caused yaw',dyaw)
                    assert abs(dy if vx else dx)<.05, (key,'cross-axis drift',dx,dy)
                else:
                    assert dyaw*wz > .03, (key,before,after)
                    assert sim.imu.angular_velocity.z*wz>.1, ('IMU yaw',sim.imu)
                result.setdefault('chassis_motion',[]).append({'key':key,'dx':dx,'dy':dy,'dyaw':dyaw})
                sim.key(88)
                sim.wait(lambda: abs(sim.odom.twist.twist.linear.x)<.005 and abs(sim.odom.twist.twist.linear.y)<.005 and abs(sim.odom.twist.twist.angular.z)<.005)
            result['four_arrows_two_yaw_keys_and_stop'] = True
        elif mode == 'position':
            assert not sim.node.get_publishers_info_by_topic('/m2_gz_simulation/lift/p_setpoint')
            for goal in (.5, 1.0, .2):
                sim.scalar('lift/p_setpoint', goal)
                sim.wait(lambda: sim.settled('vertical_to_horizontal',goal))
            sim.scalar('yaw/p_setpoint', math.pi/2)
            sim.wait(lambda: sim.settled('base_to_vertical',math.pi/2,angular=True))
            sim.world_control('pause: true'); sim.spin(.5)
            assert sim.effort['lift']==0 and sim.effort['yaw']==0
            sim.world_control('pause: false')
            sim.wait(lambda: sim.settled('vertical_to_horizontal',.2))
            # A combined Twist exercises the kinematics beyond the discrete keys.
            before=sim.poses['two_arm_robot']
            msg=Twist(); msg.linear.x=.15; msg.linear.y=.10; msg.angular.z=.20
            sim.drive.publish(msg)
            sim.wait(lambda: sim.odom.twist.twist.linear.x>.10 and
                     sim.odom.twist.twist.linear.y>.06 and sim.odom.twist.twist.angular.z>.12, 12)
            sim.spin(.7)
            after=sim.poses['two_arm_robot']
            assert after[0]-before[0]>.03 and after[1]-before[1]>.02
            assert math.remainder(after[3]-before[3],2*math.pi)>.05
            # Reset during motion must discard the latched chassis command.
            sim.world_control('reset: {all: true}'); sim.spin(.5)
            sim.scalar('lift/p_setpoint',.4)
            sim.wait(lambda: sim.settled('vertical_to_horizontal',.4))
            assert abs(sim.odom.twist.twist.linear.x)<.005 and abs(sim.odom.twist.twist.linear.y)<.005
            assert abs(sim.odom.twist.twist.angular.z)<.005
            # Harmonic's existing PosePublisher diagnostics can freeze on reset.
            # OdometryPublisher continues to report the physical ground-truth pose.
            assert abs(sim.odom.pose.pose.position.x-3.70)<.02
            assert abs(sim.odom.pose.pose.position.y-3.85)<.02
            scan_stamp,imu_stamp=stamp(sim.scan),stamp(sim.imu)
            sim.spin(.5)
            assert stamp(sim.scan)>scan_stamp and stamp(sim.imu)>imu_stamp
            result['combined_swerve_twist_and_reset_stop'] = True
            result['isolated_cascade_pause_reset'] = True
        else:
            assert not sim.node.get_publishers_info_by_topic('/m2_gz_simulation/lift/cmd_effort')
            sim.scalar('lift/cmd_effort',45.0)
            sim.wait(lambda: sim.state['vertical_to_horizontal'][0]>.03, 5)
            sim.scalar('lift/cmd_effort',0.0)
            sim.wait(lambda: abs(sim.state['vertical_to_horizontal'][0])<.005)
            result['direct_force_actuation'] = True
    finally:
        sim.close()
    result['clean_shutdown'] = True
    return result


def main():
    parser=argparse.ArgumentParser();parser.add_argument('--mode',choices=['all','fsm','position','effort'],default='all');args=parser.parse_args()
    root=Path(tempfile.mkdtemp(prefix='m2_robocon_test_'));print(f'Artifacts: {root}',flush=True)
    report={}
    try:
        for mode in (('fsm','position','effort') if args.mode=='all' else (args.mode,)):
            report[mode]=run(mode,root);print(mode,json.dumps(report[mode]),flush=True)
    finally:(root/'report.json').write_text(json.dumps(report,indent=2))

if __name__=='__main__':main()
