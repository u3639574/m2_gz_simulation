#!/usr/bin/env python3
"""Headless integration test. Source ROS and the workspace before running.

Uses isolated ROS/Gazebo discovery domains; starts and stops its own launch.
Logs and a JSON measurement report are kept in the printed temporary directory.
"""

import json
import os
from pathlib import Path
import re
import signal
import subprocess
import tempfile
import time

import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String
from std_srvs.srv import Trigger


def main():
    root = Path(tempfile.mkdtemp(prefix='m2_gz_smoke_'))
    # Keep this test away from a user's running robot or simulation.
    os.environ['ROS_DOMAIN_ID'] = '168'
    os.environ['GZ_PARTITION'] = f'm2_gz_smoke_{os.getpid()}'
    os.environ['ROS_LOG_DIR'] = str(root / 'ros_logs')
    for key in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT'):
        os.environ.pop(key, None)
    log = (root / 'launch.log').open('w')
    process = subprocess.Popen(
        ['ros2', 'launch', 'm2_gz_simulation', 'simulation.launch.py', 'gui:=false'],
        stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
    )
    rclpy.init()
    node = rclpy.create_node('simulation_smoke', parameter_overrides=[
        Parameter('use_sim_time', value=True)])
    state = {}
    statuses = []
    velocities = []
    report = {}

    def receive(msg):
        for name, position, velocity in zip(msg.name, msg.position, msg.velocity):
            state[name] = (position, velocity)

    subscriptions = [
        node.create_subscription(JointState, '/m2_gz_simulation/joint_states',
                                 receive, qos_profile_sensor_data),
        node.create_subscription(String, '/m2_gz_simulation/command_status',
                                 lambda msg: statuses.append(msg.data), 10),
        node.create_subscription(Float64, '/m2_gz_simulation/lift/cmd_vel',
                                 lambda msg: velocities.append(msg.data), 10),
    ]
    lift = node.create_publisher(Float64, '/m2_gz_simulation/lift/target_position', 1)
    gripper = node.create_publisher(Float64, '/m2_gz_simulation/gripper/target_displacement', 1)

    def wait(predicate, timeout=15.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if process.poll() is not None:
                raise RuntimeError(f'Launch exited with {process.returncode}')
            rclpy.spin_once(node, timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError(f'Timed out; state={state}; statuses={statuses[-10:]}')

    def spin_for(seconds):
        end = time.monotonic() + seconds
        wait(lambda: time.monotonic() >= end, seconds + 2)

    def send(publisher, value, mechanism):
        index = len(statuses)
        publisher.publish(Float64(data=value))
        wait(lambda: any(s.startswith(f'{mechanism}: running') for s in statuses[index:]), 5)
        return index

    def succeeded(index, mechanism):
        wait(lambda: f'{mechanism}: succeeded' in statuses[index:])

    def world_control(request):
        result = subprocess.run([
            'gz', 'service', '-s', '/world/arm_world/control',
            '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '3000', '--req', request,
        ], capture_output=True, text=True, timeout=5)
        if result.returncode or 'data: true' not in result.stdout:
            raise AssertionError(f'World control failed: {result.stdout} {result.stderr}')

    try:
        wait(lambda: len(state) >= 3 and lift.get_subscription_count() > 0 and
             gripper.get_subscription_count() > 0 and node.get_clock().now().nanoseconds > 0, 25)
        spin_for(0.3)
        lift_index = send(lift, 0.2, 'lift')
        gripper_index = send(gripper, 0.05, 'gripper')
        succeeded(lift_index, 'lift'); succeeded(gripper_index, 'gripper')
        assert abs(state['vertical_to_horizontal'][0] - 0.2) < 0.004
        assert abs(state['gripper_left_joint'][0] - 0.05) < 0.003
        assert abs(state['gripper_right_joint'][0] + 0.05) < 0.003
        report['parallel_lift_and_gripper'] = dict(state)

        index = send(lift, -0.2, 'lift'); succeeded(index, 'lift')
        assert abs(state['vertical_to_horizontal'][0] + 0.2) < 0.004
        report['negative_lift_target'] = state['vertical_to_horizontal']

        index = send(lift, 0.35, 'lift')
        spin_for(0.2)
        latest = send(lift, -0.1, 'lift'); succeeded(latest, 'lift')
        assert 'lift: interrupted' in statuses[index:]
        assert abs(state['vertical_to_horizontal'][0] + 0.1) < 0.004
        report['preemption'] = state['vertical_to_horizontal']

        index = len(statuses)
        lift.publish(Float64(data=0.5))
        wait(lambda: any('lift: rejected' in s for s in statuses[index:]))
        report['out_of_range_rejected'] = True

        index = send(lift, 0.3, 'lift')
        world_control('pause: true')
        spin_for(0.2)
        paused_clock = node.get_clock().now().nanoseconds
        spin_for(0.5)
        assert node.get_clock().now().nanoseconds == paused_clock
        assert 'lift: succeeded' not in statuses[index:]
        assert abs(velocities[-1]) < 1e-12
        world_control('pause: false')
        succeeded(index, 'lift')
        report['pause_resume'] = True

        index = send(lift, -0.3, 'lift')
        spin_for(0.2)
        stop = node.create_client(Trigger, '/m2_gz_simulation/stop')
        assert stop.wait_for_service(timeout_sec=3)
        response = stop.call_async(Trigger.Request())
        wait(response.done)
        assert response.result().success
        wait(lambda: 'lift: interrupted' in statuses[index:])
        spin_for(0.8)
        held = state['vertical_to_horizontal'][0]
        spin_for(0.5)
        assert abs(state['vertical_to_horizontal'][0] - held) < 0.005
        report['stop_hold_position'] = held

        old_time = node.get_clock().now().nanoseconds
        world_control('reset: {all: true}')
        wait(lambda: node.get_clock().now().nanoseconds < old_time, 5)
        spin_for(0.5)
        index = send(lift, 0.1, 'lift'); succeeded(index, 'lift')
        report['reset_recovery'] = state['vertical_to_horizontal']
        assert abs(state['vertical_to_horizontal'][0] - 0.1) < 0.004
        assert velocities and max(abs(value) for value in velocities) <= 0.250001
        report['maximum_commanded_velocity'] = max(abs(value) for value in velocities)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        if process.poll() is None:
            process.send_signal(signal.SIGINT)  # launch forwards the signal to its children
            try:
                process.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
        log.close()
        (root / 'report.json').write_text(json.dumps(report, indent=2))
        print(f'Integration artifacts: {root}', flush=True)

    launch_output = (root / 'launch.log').read_text()
    assert process.returncode == 0, f'Launch shutdown failed: {process.returncode}'
    unexpected_exits = [line for line in launch_output.splitlines() if 'process has died' in line]
    assert not unexpected_exits, f'A launched process crashed: {unexpected_exits}'
    assert re.search(r'\[gazebo-\d+\]: process has finished cleanly', launch_output)
    assert re.search(r'\[fsm_node-\d+\]: process has finished cleanly', launch_output)
    assert re.search(r'\[parameter_bridge-\d+\]: process has finished cleanly', launch_output)
    assert 'escalating to' not in launch_output, 'Shutdown required forced termination'
    report['clean_shutdown'] = True
    report['passed'] = True
    (root / 'report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2), flush=True)


if __name__ == '__main__':
    main()
