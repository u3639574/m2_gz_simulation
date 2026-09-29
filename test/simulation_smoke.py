#!/usr/bin/env python3
"""Opt-in Gazebo regressions for the three command sources; source ROS first."""
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
from std_srvs.srv import Trigger


def run_mode(mode, root):
    os.environ['ROS_DOMAIN_ID'] = '173'
    os.environ['GZ_PARTITION'] = f'm2_sim_robot_test_{os.getpid()}_{mode}'
    os.environ['ROS_LOG_DIR'] = str(root / 'ros_logs')
    for key in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT',
                'FASTRTPS_DEFAULT_PROFILES_FILE', 'FASTDDS_DEFAULT_PROFILES_FILE'):
        os.environ.pop(key, None)
    log_path = root / f'{mode}.log'
    log = log_path.open('w')
    process = subprocess.Popen(
        ['ros2', 'launch', 'm2_gz_simulation', 'simulation.launch.py',
         'gui:=false', f'command_source:={mode}'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = rclpy.create_node(f'smoke_{mode}')
    state = {}
    velocities = []
    setpoints = []
    result = {}

    def receive(msg):
        for name, position, velocity in zip(msg.name, msg.position, msg.velocity):
            state[name] = (position, velocity)

    subscriptions = [
        node.create_subscription(JointState, '/m2_gz_simulation/joint_states',
                                 receive, qos_profile_sensor_data),
        node.create_subscription(Float64, '/m2_gz_simulation/lift/cmd_vel',
                                 lambda msg: velocities.append(msg.data), 10),
        node.create_subscription(Float64, '/m2_gz_simulation/lift/p_setpoint',
                                 lambda msg: setpoints.append(msg.data), 10)]

    def wait(predicate, timeout=12):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise AssertionError(f'{mode} launch exited: {process.returncode}')
            rclpy.spin_once(node, timeout_sec=0.01)
            if predicate():
                return
        raise AssertionError(f'{mode} timed out: state={state}, velocity={velocities[-5:]}')

    def spin_for(seconds):
        until = time.monotonic() + seconds
        wait(lambda: time.monotonic() >= until, seconds + 2)

    def publisher(topic):
        pub = node.create_publisher(Float64, f'/m2_gz_simulation/{topic}', 1)
        wait(lambda: pub.get_subscription_count() > 0)
        spin_for(0.05)
        return pub

    def settled(target):
        position, velocity = state['vertical_to_horizontal']
        return abs(position - target) < 0.004 and abs(velocity) < 0.01

    def control_world(request):
        response = subprocess.run([
            'gz', 'service', '-s', '/world/arm_world/control',
            '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000', '--req', request],
            capture_output=True, text=True, timeout=5)
        assert response.returncode == 0 and 'data: true' in response.stdout

    try:
        wait(lambda: len(state) == 3, 25)
        spin_for(0.5)  # Allow feedback/command discovery to settle.
        if mode == 'velocity':
            assert not node.get_publishers_info_by_topic('/m2_gz_simulation/lift/cmd_vel')
            velocity = publisher('lift/cmd_vel')
            before = state['vertical_to_horizontal'][0]
            velocity.publish(Float64(data=0.1))
            spin_for(1.0)
            delta = state['vertical_to_horizontal'][0] - before
            assert 0.07 < delta < 0.13, delta
            velocity.publish(Float64(data=0.0))
            spin_for(0.3)
            assert abs(state['vertical_to_horizontal'][1]) < 0.01
            result['travel_at_0_1_m_s'] = delta
        else:
            if mode == 'fsm':
                assert not setpoints, 'FSM must not command zero before an explicit target'
                goal = publisher('lift/target_position')
            else:
                assert not node.get_publishers_info_by_topic('/m2_gz_simulation/lift/p_setpoint')
                goal = publisher('lift/p_setpoint')
            goal.publish(Float64(data=0.2))
            wait(lambda: settled(0.2))
            spin_for(0.4)
            assert settled(0.2)
            result['positive_target_position'] = state['vertical_to_horizontal'][0]
            # Fails if repeated setpoints restart the acceleration ramp at ~0.008.
            assert max(velocities) > 0.1
            goal.publish(Float64(data=-0.2))
            wait(lambda: settled(-0.2))
            result['negative_target_position'] = state['vertical_to_horizontal'][0]
            assert max(abs(v) for v in velocities) <= 0.250001
            result['max_commanded_velocity'] = max(abs(v) for v in velocities)
            if mode == 'fsm':
                assert len(setpoints) > 100, 'Verify the real continuous FSM setpoint stream'
                toggle = node.create_client(Trigger, '/m2_gz_simulation/gripper/toggle')
                assert toggle.wait_for_service(timeout_sec=3)
                future = toggle.call_async(Trigger.Request())
                wait(future.done)
                assert future.result().success
                wait(lambda: abs(state['gripper_left_joint'][0]) < 0.002 and
                     abs(state['gripper_right_joint'][0]) < 0.002)
                result['gripper_command_scheduled'] = True
                goal.publish(Float64(data=0.3)); spin_for(0.1)
                control_world('pause: true'); spin_for(0.5)
                assert abs(velocities[-1]) < 1e-12
                control_world('pause: false')
                wait(lambda: settled(0.3))
                result['pause_resume'] = True
                control_world('reset: {all: true}')
                spin_for(0.5)
                goal.publish(Float64(data=0.1))
                wait(lambda: settled(0.1))
                result['reset_recovery'] = True
                # Inject on Gazebo Transport: exercises the real Int32 bridge and mode.
                def key(code):
                    completed = subprocess.run([
                        'gz', 'topic', '-t', '/keyboard/keypress', '-m', 'gz.msgs.Int32',
                        '-p', f'data: {code}'], capture_output=True, text=True, timeout=5)
                    assert completed.returncode == 0, completed.stderr

                def grip_at(displacement):
                    return all(abs(state[name][0] - target) < 0.003 and
                               abs(state[name][1]) < 0.01 for name, target in (
                                   ('gripper_left_joint', displacement),
                                   ('gripper_right_joint', -displacement)))

                key(16777237)  # DOWN: assembly at base top, not the lower SDF limit.
                wait(lambda: settled(-0.06)); spin_for(0.15)
                key(16777234)  # LEFT / open
                wait(lambda: grip_at(0.08)); spin_for(0.15)
                key(16777236)  # RIGHT / close
                wait(lambda: grip_at(0.0)); spin_for(0.15)
                key(16777235)  # UP
                wait(lambda: settled(0.4)); spin_for(0.15)
                key(16777237)
                wait(lambda: settled(-0.06)); spin_for(0.15)
                key(16777234)
                wait(lambda: grip_at(0.08)); spin_for(0.15)

                # Lift target must remain at bottom until both fingers have closed.
                setpoints.clear()
                key(81)  # Q: sequential close then raise
                wait(lambda: bool(setpoints) and setpoints[-1] == 0.4)
                assert grip_at(0.0), f'Lift started before closure: {state}'
                wait(lambda: settled(0.4)); spin_for(0.15)
                result['keyboard_sequence'] = True

                # A: both actions start together; observe opening while lift descends.
                key(65)
                wait(lambda: state['gripper_left_joint'][0] > 0.005 and
                     state['vertical_to_horizontal'][1] < -0.01)
                assert state['vertical_to_horizontal'][0] > -0.05
                wait(lambda: settled(-0.06) and grip_at(0.08)); spin_for(0.15)
                result['keyboard_parallel'] = True
                result['keyboard_basic_commands'] = True

            else:
                left = publisher('gripper/left/cmd_pos')
                right = publisher('gripper/right/cmd_pos')
                left.publish(Float64(data=0.05)); right.publish(Float64(data=-0.05))
                wait(lambda: abs(state['gripper_left_joint'][0] - 0.05) < 0.002 and
                     abs(state['gripper_right_joint'][0] + 0.05) < 0.002)
                result['direct_gripper'] = True
    finally:
        node.destroy_node(); rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL); process.wait(timeout=5)
        log.close()
    output = log_path.read_text()
    assert process.returncode == 0
    assert 'process has died' not in output and 'escalating to' not in output, output[-4000:]
    assert 'Error in destruction' not in output, output[-4000:]
    expected = ['gazebo', 'parameter_bridge']
    if mode != 'velocity': expected.append('control_node')
    if mode == 'fsm': expected.append('fsm_node')
    for name in expected:
        assert re.search(r'\[' + name + r'-\d+\]: process has finished cleanly', output), name
    result['clean_shutdown'] = True
    return result


def main():
    root = Path(tempfile.mkdtemp(prefix='m2_sim_robot_test_'))
    print(f'Artifacts: {root}', flush=True)
    report = {}
    try:
        for mode in ('fsm', 'position', 'velocity'):
            report[mode] = run_mode(mode, root)
            print(f'{mode}: {json.dumps(report[mode])}', flush=True)
    finally:
        (root / 'report.json').write_text(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
