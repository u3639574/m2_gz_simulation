#!/usr/bin/env python3
"""Exercise simlib registration, the real move_base node and the physical swerve."""
import json
from pathlib import Path
import tempfile
import time
from ament_index_python.packages import get_package_share_directory
from m2_interfaces.srv import GetCmdTrajectory, RunTrajectory
from simulation_smoke import Simulation


def main():
    root = Path(tempfile.mkdtemp(prefix='m2_trajectory_test_'))
    print(f'Artifacts: {root}', flush=True)
    registry = Path(get_package_share_directory('m2_simlib')) / 'config/example_trajectories.json'
    sim = Simulation('fsm', root, [f'trajectory_config:={registry}'])
    report = {}
    try:
        sim.wait(lambda: sim.odom is not None and len(sim.state) == 12, 60)
        get = sim.node.create_client(GetCmdTrajectory, '/m2_gz_simulation/move_base/get_cmd_trajectory')
        run = sim.node.create_client(RunTrajectory, '/m2_gz_simulation/move_base/run_trajectory')
        assert get.wait_for_service(timeout_sec=10) and run.wait_for_service(timeout_sec=10)
        deadline = time.monotonic() + 10
        while True:
            future = get.call_async(GetCmdTrajectory.Request(id='demo_reverse'))
            sim.wait(future.done)
            if future.result().success:
                assert future.result().chassis_cmd.linear.x < 0
                break
            assert time.monotonic() < deadline, 'simlib did not register the JSON trajectory'
            sim.spin(.1)
        future = run.call_async(RunTrajectory.Request(id='demo_reverse'))
        sim.wait(future.done); assert future.result().success
        sim.wait(lambda: abs(sim.odom.pose.pose.position.x-3.4) < .02 and
                 abs(sim.odom.twist.twist.linear.x) < .03, 20)
        report['registered_planner_endpoint'] = [sim.odom.pose.pose.position.x, sim.odom.pose.pose.position.y]
        assert abs(sim.odom.pose.pose.position.y-3.85) < .02
        sim.key(88)  # Stops the planner and clears selector caches.
        sim.wait(lambda: abs(sim.odom.twist.twist.linear.x) < .005)
        sim.key(86); sim.wait(lambda: sim.odom.twist.twist.linear.x > .05)
        sim.key(16777237); sim.wait(lambda: sim.odom.twist.twist.linear.x < -.05)
        sim.key(88); sim.spin(2.5)
        assert abs(sim.odom.twist.twist.linear.x) < .005
        report['manual_override_cancels_forward_without_resuming'] = True
        sim.key(16777235); sim.wait(lambda: sim.odom.twist.twist.linear.x > .05)
        sim.world_control('reset: {all: true}'); sim.spin(.8)
        assert abs(sim.odom.pose.pose.position.x-3.7) < .02
        assert abs(sim.odom.twist.twist.linear.x) < .005
        report['reset_clears_latched_keyboard'] = True
    finally:
        sim.close()
    report['clean_shutdown'] = True
    (root/'report.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report), flush=True)


if __name__ == '__main__':
    main()
