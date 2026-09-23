# m2_gz_simulation

Gazebo Harmonic simulation of a lift + two-finger gripper robot, with cascaded ROS 2 position control via the `m2_fsm_cpp` command framework.

## Architecture

Two ROS nodes coordinate through an FSM command scheduler:

```
fsm_node (command/mode scheduling)
  ├── lift_subsystem ──→ /m2_gz_simulation/lift/p_setpoint (Float64)
  └── gripper_subsystem ──→ /m2_gz_simulation/gripper/{left,right}/cmd_pos (Float64)

control_node (outer-loop PID)
  ├── joint_feedback ← /m2_gz_simulation/joint_states (from Gazebo bridge)
  └── lift_controller ← /m2_gz_simulation/lift/p_setpoint
                     ──→ /m2_gz_simulation/lift/cmd_vel (Float64)

ros_gz_bridge (bidirectional)
  ├── /clock, /joint_states  ← Gazebo
  └── /lift/cmd_vel, /gripper/{left,right}/cmd_pos → Gazebo
```

- **Lift** uses an outer ROS position PID producing a velocity command; Gazebo's `JointController` closes the inner velocity loop.
- **Gripper** maps a Boolean grip state to two fixed position targets; Gazebo's `JointPositionControllers` handle the motion directly.
- Both `set_lift_pos_command` and `gripper_toggle_grip_command` return immediately — they mean "target issued," not "motion completed."

## Dependencies

| Dependency | Type |
|---|---|
| `rclcpp`, `std_msgs`, `std_srvs`, `sensor_msgs` | ROS 2 |
| `trolly` | M2 utility library |
| `m2_fsm_cpp` | FSM framework |
| `m2_interfaces` | M2 message definitions |
| `ros_gz_sim`, `ros_gz_bridge` | Gazebo integration (runtime) |

## Build

From the workspace root:

```bash
colcon build --packages-select m2_gz_simulation
```

The build produces:
- `libm2-gz-simulation.so` — shared library (subsystems, commands, control, modes)
- `fsm_node` — FSM executable
- `control_node` — control loop executable
- `control_test` — gtest binary

## Launch

Start Gazebo, the bridge, and both ROS nodes:

```bash
ros2 launch m2_gz_simulation simulation.launch.py
```

Launch arguments:

| Argument | Default | Description |
|---|---|---|
| `gui` | `true` | Show the Gazebo GUI (`false` for headless) |
| `paused` | `false` | Start simulation paused |
| `world` | `model/two_arm_robot.sdf` | SDF world file |
| `bridge_config` | `config/ros_gz_bridge.yaml` | Gazebo bridge topic mappings |
| `fsm_config` | `config/fsm_node_params.yaml` | FSM node parameters |
| `control_config` | `config/control_node_params.yaml` | Control node parameters |

Examples:

```bash
# Headless with custom world
ros2 launch m2_gz_simulation simulation.launch.py gui:=false world:=path/to/custom.sdf

# Paused start (inspect before running)
ros2 launch m2_gz_simulation simulation.launch.py paused:=true
```

If the bridge, `fsm_node`, or `control_node` exits, the entire launch shuts down to prevent Gazebo from retaining stale commands.

## Configuration

Two parameter files, each keyed to its node namespace (`/m2_gz_simulation/{fsm,control}_node`).

### fsm_node_params.yaml

| Parameter | Default | Description |
|---|---|---|
| `loop_freq` | `100` | FSM tick rate (Hz) |
| `lift.set_pos_topic` | `/m2_gz_simulation/lift/p_setpoint` | Lift position setpoint topic |
| `lift.min_pos` | `-0.4` | Minimum lift position (m) |
| `lift.max_pos` | `0.4` | Maximum lift position (m) |
| `gripper.left_set_pos_topic` | `/m2_gz_simulation/gripper/left/cmd_pos` | Left finger position topic |
| `gripper.right_set_pos_topic` | `/m2_gz_simulation/gripper/right/cmd_pos` | Right finger position topic |
| `gripper.closed_displacement` | `0.0` | Displacement per finger when closed (m) |
| `gripper.open_displacement` | `0.08` | Displacement per finger when open (m) |

### control_node_params.yaml

| Parameter | Default | Description |
|---|---|---|
| `loop_freq` | `100` | Control loop rate (Hz) |
| `joint_states_topic` | `/m2_gz_simulation/joint_states` | Gazebo joint state feedback |
| `feedback_timeout` | `0.25` | Max age of valid feedback (s) |
| `max_control_dt` | `0.1` | Max allowed simulation time step (s) |
| `lift.joint_name` | `vertical_to_horizontal` | Lift joint to track |
| `lift.p_setpoint_topic` | `/m2_gz_simulation/lift/p_setpoint` | Incoming position setpoint |
| `lift.cmd_vel_topic` | `/m2_gz_simulation/lift/cmd_vel` | Outgoing velocity command |
| `lift.min_position` | `-0.4` | Lower joint limit (m) |
| `lift.max_position` | `0.4` | Upper joint limit (m) |
| `lift.kp` | `3.0` | Outer-loop proportional gain |
| `lift.ki` | `0.1` | Outer-loop integral gain |
| `lift.kd` | `0.1` | Outer-loop derivative gain |
| `lift.integral_limit` | `0.2` | Integral windup clamp |
| `lift.max_velocity` | `0.25` | Velocity limit (m/s) |
| `lift.max_acceleration` | `0.8` | Acceleration limit (m/s²) |

### Gazebo bridge topics

Defined in `config/ros_gz_bridge.yaml`:

| Direction | ROS topic | Gazebo topic | Type |
|---|---|---|---|
| GZ → ROS | `/clock` | `/world/arm_world/clock` | Clock |
| GZ → ROS | `/m2_gz_simulation/joint_states` | `/world/arm_world/model/two_arm_robot/joint_state` | JointState |
| ROS → GZ | `/m2_gz_simulation/lift/cmd_vel` | `/vertical_to_horizontal/cmd_vel` | Float64 |
| ROS → GZ | `/m2_gz_simulation/gripper/left/cmd_pos` | `/gripper_left_joint/cmd_pos` | Float64 |
| ROS → GZ | `/m2_gz_simulation/gripper/right/cmd_pos` | `/gripper_right_joint/cmd_pos` | Float64 |

## Interacting with the simulation

The `fsm_node` runs `test_mode`, which currently consumes PS4 controller input on the `input_topic` parameter (default `/keyboard/keypress`). To issue commands programmatically, publish directly to the FSM topics:

```bash
# Move lift to 0.2 m
ros2 topic pub -1 /m2_gz_simulation/lift/p_setpoint std_msgs/msg/Float64 "data: 0.2"

# Move lift down
ros2 topic pub -1 /m2_gz_simulation/lift/p_setpoint std_msgs/msg/Float64 "data: -0.3"

# Open gripper (left → +0.08, right → -0.08)
ros2 topic pub -1 /m2_gz_simulation/gripper/left/cmd_pos std_msgs/msg/Float64 "data: 0.08"
ros2 topic pub -1 /m2_gz_simulation/gripper/right/cmd_pos std_msgs/msg/Float64 "data: -0.08"

# Close gripper (left → 0.0, right → 0.0)
ros2 topic pub -1 /m2_gz_simulation/gripper/left/cmd_pos std_msgs/msg/Float64 "data: 0.0"
ros2 topic pub -1 /m2_gz_simulation/gripper/right/cmd_pos std_msgs/msg/Float64 "data: 0.0"
```

Monitor joint states:
```bash
ros2 topic echo /m2_gz_simulation/joint_states
```

## Model

The SDF model (`model/two_arm_robot.sdf`) defines:
- **`vertical_to_horizontal`** — prismatic joint (Z-axis, ±0.4 m)
- **`gripper_left_joint`** — prismatic (Y-axis, 0–0.08 m)
- **`gripper_right_joint`** — prismatic (Y-axis, −0.08–0 m)

Gazebo plugins: `JointStatePublisher` (100 Hz feedback), `JointController` (lift velocity PID), `JointPositionController` ×2 (gripper position).

## Known limitations

1. **No wait-for-completion:** `set_lift_pos_command` and `gripper_toggle_grip_command` finish immediately. Sequential command groups advance without waiting for motion to settle.
2. **Initial lift target is 0.0:** The lift subsystem publishes `0.0` at startup. If the physical lift starts elsewhere, this can cause an unintended move. The control node only addresses this after receiving a valid setpoint.
3. **Repeated PID reset:** `lift_controller` resets its PID integrator on every setpoint message. Since the lift subsystem republishes every tick, integral windup protection is partially defeated during steady motion.
4. **Keyboard input not wired:** `test_mode` inherits from `ps_mode` and expects PS4 controller input on an `input_topic`. A keyboard bridge to the FSM is not yet implemented.
5. **`noexcept` on fallible methods:** Constructors and `init()` methods that create ROS entities are marked `noexcept`. If parameter declaration or topic creation fails, the process terminates rather than recovering.
6. **Tests are stubs:** `test/control_test.cpp` and `test/simulation_smoke.py` contain no test logic.

## Directory layout

```
m2_gz_simulation/
├── CMakeLists.txt
├── package.xml
├── cmake/                   # M2Build cmake modules
├── code/
│   ├── CMakeLists.txt
│   ├── include/m2_gz_simulation/
│   │   ├── base/            # simulation_robot, subsystem_container, define
│   │   ├── commands/        # lift/set_lift_pos, gripper/toggle_grip
│   │   ├── control/         # joint_feedback, lift_controller, position_pid
│   │   ├── modes/           # test_mode
│   │   ├── nodes/           # fsm_node, control_node
│   │   └── subsystems/      # lift_subsystem, gripper_subsystem
│   └── src/
│       ├── base/
│       ├── commands/
│       ├── control/
│       ├── modes/
│       ├── nodes/{fsm,control}/
│       └── subsystems/
├── config/
│   ├── fsm_node_params.yaml
│   ├── control_node_params.yaml
│   └── ros_gz_bridge.yaml
├── launch/
│   ├── simulation.launch.py
│   └── ros_gz_bridge.launch.py
├── model/
│   └── two_arm_robot.sdf
└── test/
    ├── control_test.cpp
    └── simulation_smoke.py
```