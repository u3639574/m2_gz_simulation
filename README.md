# m2_gz_simulation

Gazebo Harmonic simulation of a lift + two-finger gripper robot, with cascaded ROS 2 position control via the `m2_fsm_cpp` command framework.

## Architecture

The FSM scheduler runs inside `fsm_node`. The independent `control_node` receives
position targets over ROS topics; it does not run the command scheduler:

```
/keyboard/keypress (Gazebo Int32) → bridge → test_mode → command / group

fsm_node (command/mode scheduling)
  ├── lift_subsystem ──→ /m2_gz_simulation/lift/p_setpoint (Float64)
  └── gripper_subsystem ──→ /m2_gz_simulation/gripper/{left,right}/cmd_pos (Float64)

control_node (outer-loop PID)
  ├── joint_feedback ← /m2_gz_simulation/joint_states (from Gazebo bridge)
  └── lift_controller ← /m2_gz_simulation/lift/p_setpoint
                     ──→ /m2_gz_simulation/lift/cmd_vel (Float64)

ros_gz_bridge (directed mappings)
  ├── /clock, /joint_states, /keyboard/keypress ← Gazebo
  └── /lift/cmd_vel, /gripper/{left,right}/cmd_pos → Gazebo
```

- **Lift** uses an outer ROS position PID producing a velocity command; Gazebo's `JointController` closes the inner velocity loop.
- **Gripper** maps a Boolean grip state to two fixed position targets; Gazebo's `JointPositionControllers` handle the motion directly.
- The four keyboard motion commands finish using fresh joint position/velocity feedback; the FSM contains no PID.
- Both legacy `set_lift_pos_command` and `gripper_toggle_grip_command` return immediately — they mean "target issued," not "motion completed."

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
- `control_test`, `keyboard_command_test` — gtest binaries (when testing is enabled)

## Launch

Start Gazebo, the bridge, and both ROS nodes:

```bash
ros2 launch m2_gz_simulation simulation.launch.py
```

Launch arguments:

| Argument | Default | Description |
|---|---|---|
| `command_source` | `fsm` | `fsm`: commands + controller; `position`: controller only; `velocity`: bridge/Gazebo only |
| `gui` | `true` | Show the Gazebo GUI (`false` for headless) |
| `gui_config` | `config/gazebo_gui.config` | Scene controls and automatic KeyPublisher loading |
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
| `input_topic` | `/keyboard/keypress` | Gazebo key events bridged as ROS Int32 |
| `lift.bottom_position` | `-0.06` | Keyboard bottom preset: finger tips at base top (m) |
| `completion.position_tolerance` | `0.003` | Command completion position tolerance (m) |
| `completion.velocity_tolerance` | `0.01` | Command completion speed tolerance (m/s) |
| `completion.feedback_timeout` | `0.25` | Maximum simulation and wall age of completion feedback (s) |
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
| GZ → ROS | `/keyboard/keypress` | `/keyboard/keypress` | Int32 |
| GZ → ROS | `/clock` | `/world/arm_world/clock` | Clock |
| GZ → ROS | `/m2_gz_simulation/joint_states` | `/world/arm_world/model/two_arm_robot/joint_state` | JointState |
| ROS → GZ | `/m2_gz_simulation/lift/cmd_vel` | `/vertical_to_horizontal/cmd_vel` | Float64 |
| ROS → GZ | `/m2_gz_simulation/gripper/left/cmd_pos` | `/gripper_left_joint/cmd_pos` | Float64 |
| ROS → GZ | `/m2_gz_simulation/gripper/right/cmd_pos` | `/gripper_right_joint/cmd_pos` | Float64 |

## Interacting with the simulation

Choose one command source when launching. Stop the previous launch before switching
modes. Do not publish onto another active node's actuator output: ROS topics do
not arbitrate between two publishers, so their messages overwrite each other.
The ROS velocity message is `std_msgs/msg/Float64` in m/s, not `Twist`.

### FSM mode: teach the command/subsystem architecture

```bash
ros2 launch m2_gz_simulation simulation.launch.py command_source:=fsm
# In another sourced terminal: this schedules set_lift_pos_command.
ros2 topic pub --once /m2_gz_simulation/lift/target_position std_msgs/msg/Float64 '{data: 0.2}'
# This schedules gripper_toggle_grip_command.
ros2 service call /m2_gz_simulation/gripper/toggle std_srvs/srv/Trigger '{}'
```

`lift/target_position` is a request to the FSM. `lift/p_setpoint` is the subsystem's
output to the control node. The lift subsystem does not publish a default zero
before a command is issued. After a command, it continuously republishes that
target, as in `m2_nezha26`; identical setpoints preserve PID and acceleration state.
The gripper starts open as before. No PID or Gazebo feedback loop was added to the
FSM/subsystems.

The default GUI configuration loads **KeyPublisher**. Click the Gazebo scene to
focus its window, then use these keys (the simulation must be running):

| Key | Enum / value | Command |
|---|---|---|
| ↑ | `keyboard_key::UP` / 16777235 | Raise lift to +0.4 m |
| ↓ | `keyboard_key::DOWN` / 16777237 | Lower assembly to base top, -0.06 m |
| ← | `keyboard_key::LEFT` / 16777234 | Open grip |
| → | `keyboard_key::RIGHT` / 16777236 | Close grip |
| Q | `keyboard_key::Q` / 81 | Sequential: close grip, then raise lift |
| A | `keyboard_key::A` / 65 | Parallel: open grip and lower lift |

All keys from the supplied screenshot are declared in `base/keyboard_key.h`;
unassigned keys are ignored. Repeated events for an active action do not restart
it. Once it finishes, another event can run it again. KeyPublisher sends press
events without release state, so holding a key may run it again after completion.

The bottom preset follows the current SDF geometry: finger-tip height is
`0.75 + joint_position - 0.10 - 0.09`; the base top is at `0.50 m`, giving
`joint_position = -0.06 m`. The SDF lower limit of -0.4 m is still available to
explicit position requests, but is below the base top. Change
`lift.bottom_position` if your model or desired lower pose changes.

For a headless keyboard-path check, publish a Gazebo event:

```bash
gz topic -t /keyboard/keypress -m gz.msgs.Int32 -p 'data: 81'
# Or inject the equivalent event directly on the ROS side:
ros2 topic pub --once /keyboard/keypress std_msgs/msg/Int32 '{data: 81}'
```

The new motion commands wait for fresh feedback within 3 mm and below 0.01 m/s.
Sequential composition waits for both fingers before starting the raise command;
parallel composition starts both commands in one scheduler tick and waits for
both to finish. See [the keyboard/command implementation review](docs/keyboard_commands.md).

### Position mode: test the control node directly

```bash
ros2 launch m2_gz_simulation simulation.launch.py command_source:=position
ros2 topic pub --once /m2_gz_simulation/lift/p_setpoint std_msgs/msg/Float64 '{data: 0.2}'
```

This starts Gazebo, the bridge, and `control_node`, without an FSM that can
replace your setpoint. Lift positions are joint-relative metres in [-0.4, 0.4].
Both the subsystem and control node retain their existing clamping behavior.

With no FSM running, direct gripper tests also have a single producer:

```bash
ros2 topic pub --once /m2_gz_simulation/gripper/left/cmd_pos std_msgs/msg/Float64 '{data: 0.08}'
ros2 topic pub --once /m2_gz_simulation/gripper/right/cmd_pos std_msgs/msg/Float64 '{data: -0.08}'
```

### Velocity mode: test the bridge and Gazebo actuator

```bash
ros2 launch m2_gz_simulation simulation.launch.py command_source:=velocity
ros2 topic pub --once /m2_gz_simulation/lift/cmd_vel std_msgs/msg/Float64 '{data: 0.1}'
# Stop the motion explicitly; Gazebo retains its last velocity command.
ros2 topic pub --once /m2_gz_simulation/lift/cmd_vel std_msgs/msg/Float64 '{data: 0.0}'
```

This starts neither the FSM nor the ROS PID. Gazebo's inner velocity controller
still runs. Manual velocity commands bypass the ROS position/velocity limits;
use them for short, supervised actuator tests within the SDF joint limits.

To inspect the active command source and feedback:

```bash
ros2 topic info -v /m2_gz_simulation/lift/p_setpoint
ros2 topic info -v /m2_gz_simulation/lift/cmd_vel
ros2 topic echo /m2_gz_simulation/joint_states
```

## Tests and architecture review

```bash
colcon build --packages-select m2_gz_simulation --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select m2_gz_simulation
colcon test-result --test-result-base build/m2_gz_simulation --verbose
python3 src/m2_gz_simulation/test/simulation_smoke.py
```

Six controller C++ tests cover repeated setpoints, integral persistence, acceleration limits,
feedback loss/reset, invalid measurements and command-gated subsystem output.
Five additional C++ tests cover Int32 keyboard routing, sequential/parallel completion, interruption and stale feedback.
The opt-in Gazebo test runs all three launch modes in an isolated test domain and
partition, checking positive/negative goals, continuous FSM setpoints, gripper
commands, pause/reset recovery, manual velocity and clean shutdown. It also injects
all six keyboard actions on Gazebo Transport and verifies sequential ordering and
concurrent motion through the actual bridge.
See [the architecture/debug review](docs/sim_robot_review.md) for the comparison
with `main`, reproduced failure, fixes, and remaining teaching tradeoffs.

## Model

The SDF model (`model/two_arm_robot.sdf`) defines:
- **`vertical_to_horizontal`** — prismatic joint (Z-axis, ±0.4 m)
- **`gripper_left_joint`** — prismatic (Y-axis, 0–0.08 m)
- **`gripper_right_joint`** — prismatic (Y-axis, −0.08–0 m)

Gazebo plugins: `JointStatePublisher` (100 Hz feedback), `JointController` (lift velocity PID), `JointPositionController` ×2 (gripper position).

## Known limitations

1. **Legacy versus motion commands:** `set_lift_pos_command` and
   `gripper_toggle_grip_command` still finish when issued. The new keyboard
   commands finish on measured motion completion. `is_gripped()` remains the
   requested state; `at_target()` checks joint feedback, not contact with an object.
2. **Blocked motion:** missing/stale feedback or an obstructed joint keeps a new
   motion command pending; it does not silently advance the sequence. There is no
   automatic timeout/failed-command state yet. Conflicting actions can interrupt
   commands/groups, but cancellation leaves actuator targets latched until replaced.
3. **Discrete position goals:** changed position targets reset the PID, while
   identical targets preserve it. Continuous trajectory streaming would need a
   separate review of reset policy and acceleration continuity.
4. **Keyboard mode:** a single `test_mode` is active; no PS message conversion,
   controller LED service or multi-mode selection is used. The GUI must have focus.
5. **Fallible initialization is noexcept:** several inherited/template setup paths
   terminate on exceptions. Loop rates are now restricted to [1, 1000] Hz, but
   further parameter-validation/error-handling improvements remain possible.
6. **Transport and reset policy:** launch supervises process exits, not frozen
   processes. Invalid feedback inhibits velocity; the latest target resumes after
   fresh feedback returns. A world reset does not cancel the FSM's desired target.

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