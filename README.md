# m2_gz_simulation

ROS 2 Jazzy / Gazebo Harmonic teaching simulation: a mobile lift/gripper robot on
the supplied 2027 Robocon venue, with three dynamic Earth Blocks in red storage.
Commands and subsystems remain in `fsm_node`; lift/yaw control remains in the
separate `control_node`.

## Build and run

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select m2_gz_simulation --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 launch m2_gz_simulation simulation.launch.py
```

Focus the Gazebo GUI window. It loads KeyPublisher automatically. The robot starts
at `(3.70, 3.85)` facing world +X, close to the first storage block. Simulation
starts running by default.

| Key | Action |
|---|---|
| ↑ / ↓ | Drive forwards / backwards at 0.2 m/s |
| ← / → | Turn chassis left / right at 0.4 rad/s |
| X | Stop chassis |
| W / S | Raise lift to 1.1 m / lower to 0 m |
| E / D | Open / close gripper |
| Q | Sequential: close around a block (or close empty), then raise |
| A | Parallel: open and lower |
| R / F | Turntable to 0° / 180° relative to chassis |

Arrow and X keys use the tutorial's Gazebo `TriggeredPublisher -> DiffDrive`
connection. A press latches its velocity; releasing the key does not stop the robot.
Press X to stop. The FSM handles the letter-key arm/yaw commands using the key enum.
Do not drive from a second publisher while using arrow keys.

For a pickup, keep the arm at zero yaw and bottom height, open the fingers, drive
about 0.35 m forward until the cube is between the pads, press X, then Q. S lowers
while retaining the grip; E releases it. A opens immediately while lowering, so a
held block is released at the current height.

## Architecture

```text
Keyboard Int32 -> test_mode -> m2_fsm_cpp command / group -> subsystems
  lift_subsystem -> lift/p_setpoint (m) ---+
  yaw_subsystem  -> yaw/p_setpoint (rad) --+--> independent control_node:
                                               position PID -> velocity target
                                               velocity PID -> effort + feedforward
                                               -> Gazebo ApplyJointForce -> physics
  gripper_subsystem -> finger position targets -> Gazebo position controllers

Gazebo JointState -> control_node feedback
                 -> FSM motion completion (no PID)
Gazebo finger contacts -> gripper grasp completion

Arrow/X Int32 -> Gazebo TriggeredPublisher -> Twist -> four-wheel DiffDrive
```

Both lift and yaw now run an explicit two-loop cascade in C++. The old lift Gazebo
velocity PID is removed to avoid two competing inner controllers. Effort units are
newtons (lift) and newton-metres (yaw). The topology follows the supplied
`~/Downloads/foc_math.c`; firmware ERPM/current scaling is replaced with SI units.

`lift/velocity_target` and `yaw/velocity_target` are diagnostic outputs. They are
not bridged to another controller. Identical or changed setpoints preserve loop
state and acceleration continuity; invalid feedback or a clock discontinuity resets
both loops. The lift holds its measured startup position using gravity feedforward.

## Yaw interface

Zero yaw points the arm along chassis +X; positive yaw is counterclockwise viewed
from above. This is relative to the chassis, not a world compass heading. Commands
use the shortest angular path, including across 359°/0°.

```bash
ros2 topic pub --once /m2_gz_simulation/yaw/target_degrees \
  std_msgs/msg/Float64 '{data: 90.0}'
```

The request schedules `set_yaw_angle_command` and owns the yaw subsystem until
completion. The public API accepts finite angles `0 <= theta < 360` degrees;
invalid values are rejected. Internally the subsystem publishes radians.

Your removal of the old `lift/target_position` subscription and `gripper/toggle`
service is preserved. Those interfaces no longer exist.

## Launch modes

| `command_source` | Nodes | Direct test interface |
|---|---|---|
| `fsm` (default) | FSM and controller | Keyboard / `yaw/target_degrees` |
| `position` | Controller only | `lift/p_setpoint` (m), `yaw/p_setpoint` (rad) |
| `effort` | Neither application node | `lift/cmd_effort` (N), `yaw/cmd_effort` (Nm) |

Gazebo, bridge, gripper plugins and chassis controls exist in all modes. Stop the
previous launch before switching. The old `velocity` launch mode and lift
`cmd_vel` actuator interface have been replaced by effort control.

```bash
ros2 launch m2_gz_simulation simulation.launch.py command_source:=position gui:=false
# Another sourced terminal:
ros2 topic pub --once /m2_gz_simulation/lift/p_setpoint std_msgs/msg/Float64 '{data: 0.5}'
```

Other arguments: `gui` (true), `paused` (false), `world`, `gui_config`,
`bridge_config`, `fsm_config`, and `control_config`. Launch adds the installed model
directory to `GZ_SIM_RESOURCE_PATH`; no user-specific model search path is needed.

## Physical model

- Earth Blocks: 0.35 × 0.35 × 0.35 m, nominal mass 0.6 kg; rigid boxes with uniform
  inertia and friction. Source: supplied rulebook v1.1, page 9. No Sky Blocks.
- Three block centres: `(4.95, 3.85)`, `(4.95, 4.45)`, `(4.95, 5.05)`, all inside
  the red 1 × 2 m storage area. Resting centre height is 0.175 m.
- Vertical mast: 1.6 m; horizontal beam: 0.9 m; lift travel: 0–1.1 m. At bottom,
  fingers span roughly 0.06–0.38 m above the floor, overlapping an Earth Block.
- Open clear gripper gap: 0.54 m, greater than even the rulebook's +5% block width.
  Pads are 0.18 × 0.04 × 0.32 m with friction `mu = mu2 = 2.0` and force limit 25 N.
- Closing against a cube stops the fingers near ±0.125 m. Completion accepts fresh
  contact from both pads on the same Earth Block for at least 0.1 s. Empty closure
  still uses position/velocity completion. There is no artificial fixed attachment.
- Four wheels: 0.12 m radius, 0.68 m separation, two joints per side in DiffDrive.
  Base is dynamic and no longer attached to the world.

The model is a teaching robot with enlarged arms, not a competition-size-compliance
model. The field is the requested community model, not a certified digital twin.
Imported visual Sky Blocks are removed; collision geometry follows the venue solids
rather than the upstream coarse, mismatched floor/platform collision boxes.

## Configuration and topic contract

- `config/control_node_params.yaml`: separate position/velocity gains, speed/ramp
  limits and force/torque limits for lift/yaw. Lift gravity feedforward is 39.24 N
  (4 kg moving assembly); the inner integral supplies the additional payload force.
- `config/fsm_node_params.yaml`: input, completion thresholds, lift endpoints and
  gripper displacements. `at_target()` tolerances use metres for lift/fingers and
  radians for yaw.
- `config/ros_gz_bridge.yaml`: all ROS/Gazebo mappings and directions, including
  contacts, chassis Twist/odometry and model poses used by regression tests.

Position requests use metres for lift and radians for the low-level yaw target.
The degree API is only `/yaw/target_degrees`. In effort mode there is no ROS
position/speed limiting; Gazebo retains the last force until changed or shut down.
On feedback loss the ROS controller sends zero effort; this is not a mechanical
brake, so an unsupported lift can descend under gravity.

## Tests and model generation

```bash
colcon test --packages-select m2_gz_simulation
colcon test-result --test-result-base build/m2_gz_simulation --verbose
python3 src/m2_gz_simulation/test/simulation_smoke.py
# A shorter individual mode run:
python3 src/m2_gz_simulation/test/simulation_smoke.py --mode fsm
```

The C++ tests cover cascaded dynamics, payload compensation, feedback faults,
commands, groups, key ownership, yaw bounds and contact completion. Static tests
check dimensions, storage containment, four-wheel/actuator ownership and the bridge.
Headless tests check real grasp/lift/rotation/release, degree wrap, chassis keys,
position/effort modes, pause/reset recovery and clean shutdown.

`model/two_arm_robot.sdf` is generated by `scripts/generate_world.py`; edit that
script and regenerate when changing mechanics. The imported field and its source
commit are under `model/robocon_ground/`. `scripts/import_venue.py --source PATH`
reproduces the import from the cloned repository. Models are installed with the
package; launching needs no network connection.

See [the implementation review](docs/robocon_2027.md) for a file map, design decisions,
source references and measured test results. Earlier review documents are historical.
