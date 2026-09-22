# m2_gz_simulation

Gazebo Harmonic / ROS 2 Jazzy simulation of the arm in `model/two_arm_robot.sdf`.
The package uses the existing `m2_fsm_cpp` command scheduler and the workspace's
`m2b` build conventions. It has one lift subsystem and one two-finger gripper
subsystem. It does not depend on the hardware interfaces in `m2_nezha26`.

## Build and run

The workspace must already contain `m2b`, `trolly`, and `m2_fsm_cpp` and their
dependencies. Install the ROS/Gazebo runtime if it is missing:

```bash
sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
colcon build --packages-select m2_gz_simulation --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 launch m2_gz_simulation simulation.launch.py
```

`--packages-select` assumes the workspace dependencies have already been built,
as they were on the development machine. For a fresh checkout, build the
workspace dependencies first (for example with `--packages-up-to m2_gz_simulation`).

Launch options:

```bash
# Headless server, bridge and FSM
ros2 launch m2_gz_simulation simulation.launch.py gui:=false
# Start with physics paused
ros2 launch m2_gz_simulation simulation.launch.py paused:=true
# Only the bridge, for an already-running world
ros2 launch m2_gz_simulation ros_gz_bridge.launch.py
```

The full launch also accepts `world`, `bridge_config`, and `controller_config`.
If you change world/model names or joint interfaces, update the bridge config
and controller consistently. The existing SDF is a complete world: it is opened
directly, without spawning a second copy of the robot. Gazebo is launched as an
argument-vector process without an intermediate shell so shutdown signals reach it. Do not run the standalone
bridge at the same time as the full launch. The full launch terminates the world
if the bridge or controller exits.

## Command the mechanisms

In another sourced terminal, wait until `/m2_gz_simulation/joint_states` contains
all three joints. Targets are ROS-only topics; the node converts them into FSM
commands. Publish once per requested movement:

```bash
# Lift joint position in metres, valid range [-0.4, 0.4]
ros2 topic pub --once /m2_gz_simulation/lift/target_position std_msgs/msg/Float64 '{data: 0.2}'
# Per-finger displacement in metres, valid range [0.0, 0.08]
ros2 topic pub --once /m2_gz_simulation/gripper/target_displacement std_msgs/msg/Float64 '{data: 0.05}'
# Cancel all movements and hold the measured positions
ros2 service call /m2_gz_simulation/stop std_srvs/srv/Trigger '{}'
# Watch lifecycle events (running, succeeded, interrupted, rejected, timed out, aborted)
ros2 topic echo /m2_gz_simulation/command_status
```

Gripper displacement `d` means left joint `+d` and right joint `-d`. It is not
an absolute world position or total aperture. With the corrected finger geometry,
the ideal clear gap is `0.02 + 2*d` metres. Lift position is relative to the SDF
joint zero, not the height above ground. All movable joints are prismatic, so
positions are metres and velocities are metres/second.

A new command replaces an active command for the same subsystem. Lift and gripper
commands can run concurrently because they claim different subsystem requirements.
Successful commands leave their target active, so the subsystem continues holding
position. Startup captures the measured position instead of issuing a motion goal.
Invalid/out-of-range/nonfinite goals and goals without valid feedback are rejected;
there is no silent target clamping. Parameter-defined operating limits may be
narrower than the SDF limits but cannot exceed them.

`command_status` is a diagnostic string stream, not an action result API with
request IDs or durable history. Command classes are reusable with the sequential
and parallel command groups supplied by `m2_fsm_cpp`. No motion sequence starts
automatically. A future task-specific sequence must define how to propagate a
child command's failure before proceeding to the next step.

## Control and feedback flow

```text
ROS lift/target_position
  -> simulation_robot request callback
  -> m2_fsm_cpp scheduler: move_lift_command
  -> lift_subsystem target
  -> position PID, velocity and acceleration limits
  -> Float64 lift/cmd_vel -> ros_gz_bridge -> gz.msgs.Double
  -> Gazebo JointController velocity PID -> joint force -> physics
  -> Gazebo JointStatePublisher -> ros_gz_bridge -> sensor_msgs/JointState
  -> joint_feedback -> lift_subsystem and command completion checks

ROS gripper/target_displacement
  -> scheduler: set_gripper_command
  -> gripper_subsystem -> left/right Float64 cmd_pos
  -> ros_gz_bridge -> Gazebo JointPositionController -> physics
  -> the same JointState feedback -> both-finger completion check
```

`m2_fsm_cpp` supplies command scheduling, requirement arbitration and subsystem
`ticks`; this package supplies the robot-specific feedback and outer PID. The
inner velocity feedback loop runs inside Gazebo. This is a cascaded position /
velocity controller for the lift. The gripper retains its two position controllers.

### Position PID

`position_pid::compute()` uses:

```text
error = target_position - measured_position
raw_velocity = kp * error + ki * integral(error * dt) - kd * measured_velocity
```

Derivative uses measured velocity to avoid a derivative spike when targets change.
The candidate integral is clamped; integration is accepted only when the output is
unsaturated or the error is unwinding saturation. The output is limited by maximum
velocity and by acceleration relative to the previous command. PID state persists
across control ticks and resets on a new target, hold, invalid feedback or time
reset. Movement farther out of a joint limit is suppressed.

The outer defaults are `kp=3.0`, `ki=0.1`, `kd=0.1`, maximum velocity `0.25 m/s`,
and maximum acceleration `0.8 m/s^2`. These live in `fsm_node_params.yaml`.
The SDF contains inner velocity gains `P=200`, `I=80`, `D=0`, a force limit of
`+/-200 N`, and `54.936 N` of feed-forward gravity compensation for the nominal
`5.6 kg` moving mass. Gazebo's integral limits use Gazebo PID conventions; the
ROS outer integral limit is the accumulated position error in `m*s`.
The two gripper position controllers use `P=120`, `I=0`, `D=12` and `+/-20 N`.
These are starting gains verified for this unloaded model, not arbitrary payloads.

### Timing and failure behavior

The node has a 100 Hz wall timer and a single-threaded executor. Each robot tick
first advances the simulation-time feedback context, then calls the existing FSM
scheduler. PID `dt`, command timeouts and settling durations use ROS simulation
time from `/clock`, not wall time. The scheduler updates subsystems before commands,
so a newly initialized target begins driving output on the following tick.

A feedback frame is accepted atomically only if all three required joint names
appear exactly once with finite positions and velocities. Name order is irrelevant.
Older samples are discarded. A small lead over `/clock` is tolerated because clock
and feedback are independent transport streams. Samples older than 0.25 simulation
seconds invalidate feedback; lift output becomes zero, grippers receive their last
measured positions, and active commands abort. On recovery, the lift captures its
current position and a new command is needed to resume the old movement.

Duplicate simulation timestamps do not advance PID or command completion. A clock
stall longer than 0.25 wall seconds sends zero lift velocity and resets PID state;
it retains the target so a paused simulation can resume. A rewind or a simulation
time gap over 0.1 s invalidates feedback, cancels commands and resets control. Fresh
feedback is required afterward. Faster-than-real-time runs may need a higher loop
frequency or a reviewed `max_control_dt`.

A command finishes only when position and velocity remain within tolerance for
0.2 simulation seconds. Default tolerances: lift 3 mm, gripper 2 mm per finger,
and 0.01 m/s velocity. Timeout is 10 simulation seconds. Timeout and interruption
hold measured positions rather than continuing the previous movement. Normal
completion keeps the requested target. Cancellation uses owner IDs because this
FSM initializes a replacement before ending the previous command; an old command
must not overwrite a new owner's target.

The scheduler singleton retains subsystems to process exit. `simulation_robot`'s
destructor releases their ROS publishers while the node still exists and drains
commands, avoiding ROS entity destruction during static teardown. One robot per
process is supported. No modifications to `m2_fsm_cpp` or `m2_nezha26` are required.

The stop service is a controlled hold, not an emergency-stop or power-off interface.
Direct low-level topic publishers bypass FSM arbitration. The built-in Gazebo
controller retains its last command if transport disappears, so launch supervises
bridge/controller exits by shutting down the world. A frozen process or arbitrary
external transport outage is not covered by a Gazebo-side command watchdog.

## File-by-file review map

| File(s) | Responsibility and change |
|---|---|
| `model/two_arm_robot.sdf` | World and robot geometry; added world-to-base fixed mount and 100 Hz named joint feedback; enabled lift force-mode velocity PID and gravity compensation; tuned finger position controllers; made command topic names explicit; removed unsupported controller `publish_clock` tags. Finger centres changed from coincident `y=0.4` to `0.425` and `0.375` to prevent overlap at zero. |
| `launch/simulation.launch.py` | New complete launch: SDF, YAML bridge, FSM, GUI/headless and pause options, configurable paths, exit supervision. |
| `launch/ros_gz_bridge.launch.py` | Existing bridge-only launcher retained; now defaults to this package's bridge name and mapping file. |
| `config/ros_gz_bridge.yaml` | Five directed Gazebo/ROS mappings: clock, joint feedback, lift velocity, two finger positions. Updated comments to reflect implemented plugins. No bidirectional command loops. |
| `config/fsm_node_params.yaml` | New node timing, outer PID gains, motion limits, tolerances and configurable feedback/output topics. Restart to apply parameter-file changes. |
| `code/src/nodes/fsm_node.cpp` | Executable entry point; validates frequency, configures logging, creates robot and wall timer, spins serial callbacks, reports startup configuration errors. |
| `code/include/m2_gz_simulation/base/simulation_robot.h` and `code/src/base/simulation_robot.cpp` | Registers both subsystems, owns feedback context, converts ROS requests into scheduled commands, provides stop service, handles time discontinuities and ROS resource teardown. |
| `code/include/m2_gz_simulation/control/joint_feedback.h` and `code/src/control/joint_feedback.cpp` | Complete named-joint sample validation, cached position/velocity, timestamp freshness, simulation `dt`, rewind/gap and wall-stall detection. |
| `code/include/m2_gz_simulation/control/position_pid.h` | Header-only stateful outer PID with derivative on velocity, anti-windup, velocity clamp, acceleration limit and reset. |
| `code/include/m2_gz_simulation/subsystems/position_subsystem.h` | Shared command-facing subsystem contract and owner-aware release; holds no duplicate control loop. |
| `code/include/m2_gz_simulation/subsystems/lift_subsystem.h` and `code/src/subsystems/lift_subsystem.cpp` | Lift target, outer PID, velocity publication, measured-position hold, limits and settled checks. |
| `code/include/m2_gz_simulation/subsystems/gripper_subsystem.h` and `code/src/subsystems/gripper_subsystem.cpp` | Paired finger targets, signed position commands, measured-position hold and both-finger settled checks. |
| `code/include/m2_gz_simulation/commands/position_command.h` and `code/src/commands/position_command.cpp` | Shared command lifecycle with concrete `move_lift_command` and `set_gripper_command`; requirements, readiness, settling, timeout, cancellation and status. |
| `code/CMakeLists.txt` | New explicit source lists and target selector assignments for the shared control library and FSM executable. |
| `CMakeLists.txt` | Existing `m2b` entry point retained; adds the controller unit-test target. |
| `cmake/ros/m2_gz_simulation_ros_2_build_library.cmake` | Fixes invalid local interface target, removes unused geometry/nav/hardware interface dependencies and self-dependency, adds sensor messages, links the existing FSM core. |
| `cmake/modules/m2_gz_simulation_publish.cmake` | Builds the control library with ROS dependencies; installs launch, config, model and this guide. |
| `cmake/ros/m2_gz_simulation_ros_2_publish.cmake` | Publishes `fsm_node` without duplicating control sources and exports actual runtime/build dependencies. |
| `package.xml` | Declares sensor messages, Gazebo launch/bridge, launch tools and tests; removes unused generated-interface declarations. |
| `test/control_test.cpp` | Ten regression cases covering PID bounds/state, invalid inputs, feedback ordering/staleness/reset, limits, ownership, settling, timeout and lost feedback. |
| `test/simulation_smoke.py` | Opt-in real Gazebo integration test; starts an isolated headless launch, exercises commands/pause/reset/stop and verifies clean shutdown; writes measurements and logs to a temporary directory. |

Existing build helper files not listed as changed retain their template behavior.

## Validation

```bash
colcon test --packages-select m2_gz_simulation
colcon test-result --test-result-base build/m2_gz_simulation --verbose
gz sdf -k src/m2_gz_simulation/model/two_arm_robot.sdf
python3 src/m2_gz_simulation/test/simulation_smoke.py
```

The integration test uses ROS domain 168 and a unique Gazebo partition, starts
and cleans up its own headless launch, and requires local UDP/topic networking.
It checks simultaneous lift/gripper movement, negative lift targets, same-subsystem
preemption, invalid target rejection, paused-clock behavior, stop-and-hold, world
reset recovery, the 0.25 m/s command bound, and process shutdown. It is deliberately
opt-in rather than starting Gazebo during every `colcon test`.

Validation on 2026-09-23 passed: package build, ten C++ test cases, SDF validation,
launch argument loading and the full headless smoke test, including clean exits
of Gazebo, bridge and FSM. Measured lift positions at command completion were
`0.199138 m` for a `0.2 m` target and `-0.198989 m` for a `-0.2 m` target. Finger
positions reached `+/-0.05 m`; maximum commanded lift velocity was `0.25 m/s`.
Workspace review artifacts are saved under `log/m2_gz_simulation_review/`.

The implementation was built against the installed workspace dependencies. Gazebo
8.15.0 was used for headless validation. The development machine lacked the two
ROS/Gazebo runtime packages and sudo required a password, so testing used downloaded
Debian packages extracted under `/tmp/m2_gz_runtime`, without system installation.
For ordinary use, install the dependencies using the command above. GUI rendering
and payload/contact grasping are not covered by the headless test.

References: [ROS/Gazebo topic bridge](https://gazebosim.org/docs/latest/ros2_integration/),
[Gazebo launch](https://gazebosim.org/docs/harmonic/ros2_launch_gazebo/),
[JointController](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1JointController.html),
[JointStatePublisher](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1JointStatePublisher.html).
