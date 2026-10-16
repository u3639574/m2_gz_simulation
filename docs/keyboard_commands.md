# Keyboard FSM and composite commands

Historical review from the earlier stationary-arm implementation. For the current
competition robot, controls, units and test results, see [swerve_sensors.md](swerve_sensors.md)
and the package README.


## Architecture

The structure follows `m2_nezha26`: input belongs to a mode, modes schedule commands,
commands declare their subsystem requirements, and subsystems publish desired
actuator state. The keyboard transport uses Gazebo `gz.msgs.Int32` bridged to ROS
`std_msgs/msg/Int32`, instead of PS `JoyData`.

`simulation_robot` derives from the existing generic `fsm::ros2_robot`. It owns one
`test_mode`, forwards Int32 events, enters the mode on initialization, ticks the
scheduler/mode, and exits the mode at shutdown. There is no joystick LED discovery
or fake joystick-message conversion. This change is local to `m2_gz_simulation`;
`m2_fsm_cpp` and `m2_nezha26` remain unchanged.

```text
Gazebo KeyPublisher
  → /keyboard/keypress (gz.msgs.Int32)
  → ros_gz_bridge (GZ_TO_ROS)
  → /keyboard/keypress (std_msgs/msg/Int32)
  → simulation_robot → test_mode::on_input(keyboard_key)
  → m2_fsm_cpp command scheduler
  → basic command or sequential/parallel group
  → lift/gripper subsystem → actuator target topics

Gazebo JointState → motion_completion → subsystem.at_target() → command.is_finished()

Lift p_setpoint → independent control_node PID → cmd_vel → Gazebo velocity PID
```

The new feedback in the FSM only answers whether motion has completed. PID remains
in the separate control node and Gazebo, preserving the teaching separation.

## File map

| File | Responsibility |
|---|---|
| `base/keyboard_key.h` | Typed enum for every screenshot key: Q/A/Z/W/S/X/E/D/C/R/F/V and arrows. Numeric codes occur here, not in mode logic. |
| `base/simulation_robot.h`, `code/src/base/simulation_robot.cpp` | Generic ROS robot lifecycle, Int32 subscription, single active keyboard mode, subsystem registration and existing terminal request interfaces. |
| `modes/test_mode.h`, `code/src/modes/test_mode.cpp` | Six bindings, scheduling and active-command tracking to suppress repeat events while the same action is pending/running. Unknown keys have no effect. |
| `commands/lift/move_lift.h`, `code/src/commands/move_lift.cpp` | `raise_lift_command` and `lower_lift_command`, sharing endpoint selection and completion behavior. Own the lift requirement. |
| `commands/gripper/set_grip.h`, `code/src/commands/set_grip.cpp` | `open_grip_command` and `close_grip_command`, sharing explicit grip-state setting and completion behavior. Own the gripper requirement. |
| `commands/composite/grip_and_lift.h`, `code/src/commands/grip_and_lift.cpp` | Named subclasses of the actual framework sequential and parallel groups. Compose the basic commands with `add_commands`, just as the real robot does. |
| `subsystems/motion_completion.h` | Read-only feedback helper: matches joints by name, rejects incomplete/nonfinite measurements, checks position/speed and simulation/wall freshness. New requests invalidate old completion data. |
| Lift/gripper subsystem headers and sources | Expose `at_target()` and endpoint accessors; retain target publication. Destroy feedback subscriptions during cleanup. |
| `config/fsm_node_params.yaml` | Input topic, completion thresholds, bottom preset. |
| `config/ros_gz_bridge.yaml` | Adds the directional Int32 keyboard bridge. |
| `config/gazebo_gui.config`, `launch/simulation.launch.py` | Loads the scene, world controls and KeyPublisher automatically in GUI mode; custom GUI config can be supplied. Headless mode omits it. |
| `test/keyboard_command_test.cpp`, `CMakeLists.txt` | Five additional C++ integration tests using the real ROS input subscription, scheduler and command groups. |
| `test/simulation_smoke.py` | Publishes all six Gazebo keyboard events and verifies motion in addition to the existing three-mode tests. |

Header paths in the table are relative to `code/include/m2_gz_simulation/` unless
otherwise stated. Source and configuration paths are relative to the package root.

## Bindings and endpoints

| Key | Action |
|---|---|
| Up | Raise to `lift.max_pos` (+0.4 m) |
| Down | Lower to `lift.bottom_position` (-0.06 m) |
| Left | Open grip (left +0.08 m, right -0.08 m) |
| Right | Close grip (both 0 m) |
| Q | Close grip completely, then raise |
| A | Open grip and lower concurrently |

The screenshot codes map to uppercase letter values even for lowercase typing.
The enum preserves those observed values, including Qt's extended arrow codes.

“Bottom on the base” is interpreted as the lowest gripper tips at the base's top
surface. In the supplied SDF, `0.75 + q - 0.10 - 0.09 = 0.50`, hence `q = -0.06 m`.
This avoids using the lower joint limit (-0.4 m), which would place the assembly
below that surface. The preset is clamped to the configured joint range and can be
changed without editing the command class.

## Completion, concurrency and interruption

Basic motion commands set their target in `initialize()`, leave control to the
actuator, and return `subsystem.at_target()` from `is_finished()`. Completion needs
fresh feedback after the request, position error at most 0.003 m and speed at most
0.01 m/s. Gripper completion checks both fingers. Invalid or stale feedback cannot
advance a sequence.

The sequential group owns both subsystem requirements throughout. Only its close
child starts initially. When that child finishes, the framework starts the raise
child; the group finishes when raising completes. The parallel group initializes
both children in one scheduler tick and finishes after both report completion.

The children retain the framework's `CANCEL_SELF` policy, inherited by their groups.
A conflicting incoming command can replace the whole group. It prevents remaining
sequence stages from starting. `test_mode` tracks command IDs (including pending
commands) so repeated events for an active key do not recreate its group. Different
keys can schedule independent actions; subsystem requirements resolve conflicts.

Cancellation is orchestration cancellation, not an emergency stop: published
actuator targets remain latched until replaced. Likewise, starting a sequence does
not automatically stop motion toward a previously issued, latched target. Start the
teaching sequence from a settled pose. There is no automatic motion-failure timeout;
blocked joints leave a command pending until replaced. Contact-based grasp success
and multi-mode switching are outside this implementation.

The existing terminal `lift/target_position` and `gripper/toggle` interfaces retain
their instant-command semantics. They can also replace a conflicting keyboard group.

## Validation

All eleven C++ cases passed (six controller regressions plus five keyboard/group
cases). The headless Gazebo test passed all three command-source modes. In FSM mode
it injected Up, Down, Left, Right, Q and A through Gazebo Transport, verified that
both fingers were closed when Q first issued the raise target, and observed opening
while the lift descended for A. Existing pause/reset recovery and clean-shutdown
checks also passed. These tests exercise the bridge and physics; physical desktop
keypress delivery and visual layout still require a focused GUI window.
