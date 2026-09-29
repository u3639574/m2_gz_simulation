# Review of `sim_robot` and the setpoint failure

This records the earlier setpoint-debugging change. The subsequent keyboard work
implements motion completion and replaces the PS input scaffold; see
[keyboard_commands.md](keyboard_commands.md) for the current implementation.

Reviewed against local `main` at `23f41b0` and `sim_robot` at `c66f06a`.
Fixes are working-tree changes on `sim_robot`; `main`, `m2_fsm_cpp` and
`m2_nezha26` were not modified.

## Architecture judgment

The separation is appropriate for teaching the same command/subsystem design used
on the real robot. A subsystem publishes desired actuator state; a separate motor
controller or driver closes the control loop. Students can write commands and
modes without learning PID implementation first.

```text
fsm_node
  mode or ROS intent request
    -> m2_fsm_cpp command scheduler
      -> set_lift_pos_command
        -> lift_subsystem (stores/publishes desired position)
             |
             | Float64 lift/p_setpoint
             v
control_node
  joint_feedback -> lift_controller / outer position PID
                         |
                         | Float64 lift/cmd_vel
                         v
ros_gz_bridge -> Gazebo JointController / inner velocity PID -> physics
     ^                                                          |
     +---------------------- JointState feedback ----------------+
```

The nodes coordinate through topics, not a shared scheduler. `control_node` is the
simulation equivalent of a motor-controller interface. Keeping it in the same
package/shared library is acceptable for this teaching project: runtime ownership
is still separate, and the code is divided into control, commands and subsystems.

This is structurally similar to `m2_nezha26`'s lift subsystem publishing position
setpoints to a lower-level device/controller. It is not a drop-in hardware binary:
the current topics, Float64 types and metre units differ from hardware interfaces.
Use an adapter/remapping layer if exact hardware interchangeability becomes a goal.

The gripper's Boolean-to-two-joint-position mapping remains in its subsystem and
its position loops remain in Gazebo. This is reasonable for modeling a pneumatic
actuator. For complete simulator independence, move the Gazebo-specific finger
mapping into an actuator adapter later.

## Confirmed failure and comparison with main

The SDF and bridge YAML mappings were identical between the two branches. No model
or Gazebo-plugin change was needed to solve this failure.

1. `lift_subsystem::tick()` published its stored target at 100 Hz, initialized to
   `0.0`. A terminal publishing directly to `lift/p_setpoint` introduced a second
   publisher on the subsystem's output. Its `0.2` target was replaced about one
   FSM period later by `0.0`.
2. `lift_controller::receive_setpoint()` reset the PID for every message, including
   repeated identical values. `position_pid::reset()` clears both the integral and
   previous output. At 100 Hz and 0.8 m/s^2 acceleration, the ramp repeatedly starts
   at roughly `0.8 * 0.01 = 0.008 m/s`, instead of rising toward `0.25 m/s`.
3. Direct `cmd_vel` publishing with the control node running creates the same
   competing-publisher problem one layer farther downstream.
4. `test_mode::on_input()` is empty, so the existing mode never schedules a new lift
   target. Its PS input type is `m2_interfaces/msg/JoyData`; the keyboard-like topic
   name does not make it accept Gazebo keyboard `Int32` messages.

In the reproduction before the fix:

- After publishing `p_setpoint=0.2`, joint position stayed around `0.000036 m` and
  the most recent setpoints were all `0.0`.
- Publishing `cmd_vel=0.1` at 20 Hz with the ROS controller running moved the lift
  only `0.00454 m` over one second. Topic inspection found both the terminal test
  publisher and `control_node`; the controller output repeatedly returned to about
  `-0.008 m/s`.

In `main`, the normal request was `lift/target_position`, handled by the FSM before
updating its subsystem's target. Its PID was called in subsystem ticks and reset
when a command changed the goal, rather than on every transport heartbeat.
Publishing directly to an actively controlled velocity output is not exclusive
control on either branch; it should always be tested without a competing producer.

## Changes

| File | Change and purpose |
|---|---|
| `launch/simulation.launch.py` | Adds mutually exclusive `command_source` modes: `fsm`, `position`, `velocity`. Supervises only the processes actually started. |
| `code/src/control/lift_controller.cpp` | Identical setpoints no longer reset PID state. Changed goals retain the existing reset policy. The control node remains the owner of the outer loop. |
| `code/src/subsystems/lift_subsystem.cpp` and its header | Waits for an explicit command before publishing; retains the real-robot-style continuous target publication afterward. Adds ROS resource cleanup. |
| `code/src/base/simulation_robot.cpp` and its header | Adds `lift/target_position` and `gripper/toggle` request interfaces that schedule existing command classes. Adds explicit cleanup for singleton-owned subsystem resources. |
| `code/src/subsystems/gripper_subsystem.cpp` and its header | Releases publishers/node references during cleanup and avoids publishing after ROS shutdown. Control semantics remain unchanged. |
| `code/src/nodes/fsm/fsm_node.cpp` | Restricts loop rate to [1, 1000] Hz and closes/resets the robot after spinning to break node ownership cycles. |
| `code/src/nodes/control/control_node.cpp` | Restricts loop rate to [1, 1000] Hz and avoids integer-millisecond truncation of its timer period. |
| `code/include/m2_gz_simulation/control/{joint_feedback,lift_controller}.h` and constructors | Helpers borrow their owning node instead of storing `shared_from_this()` back into it, eliminating ownership cycles. |
| `config/ros_gz_bridge.yaml` | Corrects ownership/configuration comments. Actual bridge topics and types are unchanged. |
| `test/control_test.cpp` and `CMakeLists.txt` | Replaces the empty test stub with six regressions; initializes and links the same Trolly ROS logger the application uses. |
| `test/simulation_smoke.py` | Replaces the empty smoke-test stub with real tests for all three modes and clean shutdown. |
| `README.md` | Documents the layer boundaries, correct publish commands, and remaining limitations. |
| `docs/sim_robot_review.md` and `cmake/modules/m2_gz_simulation_publish.cmake` | Records this review and installs it alongside the README. |

No PID, joint-state subscription or Gazebo control code was added to the FSM
subsystems. The new ROS entry points are another source of scheduled commands,
just like a mode's input handler.

## Command ownership

| Launch mode | Running ROS application nodes | Publish lift requests to |
|---|---|---|
| `fsm` (default) | FSM and control node | `/m2_gz_simulation/lift/target_position` |
| `position` | Control node only | `/m2_gz_simulation/lift/p_setpoint` |
| `velocity` | Neither FSM nor control node | `/m2_gz_simulation/lift/cmd_vel` |

Gazebo and the bridge run in every mode. Stop the previous launch before changing
modes. Terminal gripper position tests should use `position` or `velocity` mode so
the FSM does not continuously overwrite those joint targets. There is no topic
multiplexer or automatic priority arbitration between external publishers.

## What should remain separate from this bug fix

- **Completion feedback:** an instantaneous setpoint command means “request issued,”
  not “motion settled.” That can be intentional, but sequential motion exercises
  need explicit waits or actuator feedback/status. Reading completion feedback in
  a subsystem does not require putting the PID there.
- **Input curriculum:** keep or implement PS button bindings in `test_mode` as a
  student exercise. Keyboard events require a real message adapter or a separate
  input handler.
- **Requested versus measured state:** `is_gripped()` means the requested gripper
  state. It does not confirm contact with an object.
- **Trajectory streaming:** changing goals still reset the PID. Before introducing
  continuously changing trajectories, revisit integral reset and acceleration
  continuity across goal updates.
- **Fault policy:** stale feedback stops velocity until valid data returns. The
  latest desired target then resumes. World reset does not clear the FSM's target;
  change that deliberately if the desired teaching behavior is cancel-on-reset.
- **Initialization:** the existing noexcept setup paths can terminate on invalid
  configuration. Broader parameter-validation changes are outside this fix.

## Validation

Use the build/test commands in the README. The C++ regressions exercise actual ROS
subscriptions in the control helper, rather than duplicating the setpoint-reset
logic in a test. The headless test checks an initial idle FSM, continuous target
heartbeats, positive/negative goals, gripper command scheduling, pause/resume,
reset recovery, isolated position input, isolated velocity input, and process
cleanup. GUI rendering and contact/payload grasping are not part of these checks.

The completed headless run passed all three modes, and all six C++ regressions
passed. Measured results:

| Check | Observed result |
|---|---|
| FSM request of +0.2 m | +0.1998705 m |
| Direct position request of +0.2 m | +0.1998728 m |
| Negative goal of -0.2 m | Within 0.003 m in both position paths |
| Maximum commanded velocity in position paths | 0.25 m/s |
| Direct 0.1 m/s velocity for one second | 0.0993718 m travel |
| Gripper, pause/resume and reset checks | Passed |
| Shutdown of all three launch modes | Clean |

Local test reports and logs are saved under
`/home/thunder/ros2_ws/log/m2_sim_robot_review/`.
