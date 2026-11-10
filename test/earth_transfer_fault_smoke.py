#!/usr/bin/env python3
"""Physical regressions for empty grasp and X cancelling the entire autopath."""
import argparse
import json
from pathlib import Path
import yaml
from ament_index_python.packages import get_package_share_directory
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from simulation_smoke import Simulation


def run(root):
    root.mkdir(parents=True, exist_ok=True)
    config=yaml.safe_load((Path(get_package_share_directory('m2_gz_simulation'))/'config/fsm_node_params.yaml').read_text())
    params=config['/m2_gz_simulation/fsm_node']['ros__parameters']
    params['pid_pose']['pickup_x']['target']=3.70  # Deliberately stop short: empty grip.
    params['earth_transfer.stage_timeout']=5.0
    path=root/'missing_block.yaml'; path.write_text(yaml.safe_dump(config))
    sim=Simulation('fsm',root,[f'fsm_config:={path}']); stages=[]; report={}
    sub=sim.node.create_subscription(String,'/m2_gz_simulation/chassis/autopath_status',
        lambda msg: stages.append(msg.data),QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    try:
        sim.wait(lambda: len(sim.state)==12 and sim.odom is not None and 'earth_block_1' in sim.poses,60)
        sim.spin(1); sim.key(ord('T'))
        sim.wait(lambda: any(s.startswith('failed:') for s in stages),15)
        assert 'confirm_grasp' in stages and 'raise' not in stages and 'approach_red' not in stages, stages
        assert abs(sim.state['vertical_to_horizontal'][0])<.01
        assert abs(sim.poses['earth_block_1'][2]-.175)<.01
        sim.spin(.5); assert abs(sim.odom.twist.twist.linear.x)<.005
        report['empty_grasp_stopped_without_raising_or_driving']=True
        sim.key(ord('X'))
        client=sim.node.create_client(SetParameters,'/m2_gz_simulation/fsm_node/set_parameters')
        assert client.wait_for_service(timeout_sec=3)
        future=client.call_async(SetParameters.Request(parameters=[Parameter(name='pid_pose.pickup_x.target',
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,double_value=4.05))]))
        sim.wait(future.done); assert future.result().results[0].successful
        stages.clear(); sim.key(ord('T'))
        sim.wait(lambda: 'pickup' in stages and sim.odom.twist.twist.linear.x>.08,10)
        sim.key(ord('X')); sim.wait(lambda: 'cancelled' in stages,3)
        sim.wait(lambda: abs(sim.odom.twist.twist.linear.x)<.005,3)
        sim.spin(1)
        assert 'close_grip' not in stages and 'raise' not in stages, stages
        assert abs(sim.odom.twist.twist.linear.x)<.005
        report['X_cancelled_entire_sequence_during_pickup']=True
    finally:
        report['stages']=stages
        (root/'report.json').write_text(json.dumps(report,indent=2))
        sim.close()
    print(json.dumps(report),flush=True)

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--output',type=Path,required=True)
    run(parser.parse_args().output.resolve())
