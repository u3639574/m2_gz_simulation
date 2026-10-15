# 2027 competition simulation implementation review

This change extends the keyboard command/subsystem design with an explicit C++
position/velocity cascade, an imported venue, three Earth Blocks, a turntable, and
four-wheel chassis control. It preserves the user's removal of the old lift request
subscription and gripper toggle service. `m2_fsm_cpp` and `m2_nezha26` are unchanged.

## Controller design

Previously the ROS code calculated a velocity target and Gazebo's JointController
implemented the velocity loop. Now **both loops live in `control_node`**, for lift
and yaw independently:

```text
position target - measured position
  -> outer position PID
  -> velocity clamp and acceleration ramp
  -> velocity target - measured velocity
  -> inner velocity PID + feedforward
  -> force/torque clamp
  -> Float64 cmd_effort
  -> ros_gz_bridge -> Gazebo ApplyJointForce -> physical joint
```

The reference is `foc_run_cascaded_pid_control_pos()` in the user-supplied
`/home/thunder/Downloads/foc_math.c` (starting at line 407). That code computes a
velocity command and then an effort-related current command. This implementation
uses the same two-loop topology but is independently written for SI-unit physics;
it does not copy VESC firmware code or its 3200 ERPM scaling/current conversion.

The outer loop has proportional/integral position terms, derivative on measured
velocity, velocity saturation and an acceleration limit. The inner loop integrates
velocity error with clamping and conditional anti-windup, and uses a filtered
measured-velocity derivative to avoid setpoint derivative kicks. An invalid sample
or time discontinuity resets both loops. Setpoint heartbeats and new valid goals do
not reset them, so the velocity ramp remains continuous.

For the lift, effort is force in N. The moving assembly has mass 4 kg, so the initial
feedforward is `4 * 9.81 = 39.24 N`. The speed-loop integral compensates for the added
0.6 kg payload. For yaw, effort is torque in Nm and feedforward is zero. The controller
uses the shortest angular error with `std::remainder`, so crossing 359° to 1° does
not command nearly a full revolution. At exactly 180°, either shortest direction
is valid and floating-point initial error can select the direction.

The old Gazebo lift JointController is removed; each controlled axis has exactly one
ApplyJointForce plugin. The published `velocity_target` topics are diagnostics only.
On stale feedback, zero effort is sent; without a mechanical brake, a lift can fall
under gravity. Launch shuts the world down if a required process exits, but it does
not detect every possible frozen-process failure.

## Venue and blocks

The source repository was cloned to `/home/thunder/ros2_ws/vendor/gazebo_models`:
[upstream repository](https://github.com/jimi4-lei/gazebo_models), commit
`1391b36e4707cdfc7071d6ad5509bc9a8b828603`. A small adapted copy of the field is installed
inside the package. No launch-time clone or download is required.

The original mesh contains Sky Block visuals. The importer removes scene nodes
`Cube.020` through `Cube.043` and `Plane.015`; it also builds a static collision mesh
from the actual solid venue geometry, including the ramps/stairs/platforms. A flat
floor box supplies reliable ground contact. This replaces the upstream Gazebo
Classic model's coarse collision boxes, whose pillar/platform locations and heights
do not consistently follow its visual mesh. Attribution and the upstream README are
retained in `model/robocon_ground/`.

The supplied [rulebook v1.1](/home/thunder/Downloads/Robocon_2027_Rulebook_v1-1.pdf),
page 9, specifies a nominal Earth Block side of 350 mm (±5%) and nominal mass of
600 g (±20%). The three implemented blocks use the nominal dimensions and mass,
rigid collision boxes, and uniform-body inertia. Cardboard deformation and stickers'
actual contact properties are approximated, not simulated.

The imported red storage rectangle covers x=[4.5,5.5], y=[3.5,5.5] m. Block centres
are `(4.95,3.85)`, `(4.95,4.45)`, `(4.95,5.05)`; all three full footprints fit inside.
The robot starts near storage at `(3.70,3.85)`, facing +X. This is a convenient pickup
practice location rather than a rulebook match-start placement. No other game objects
are added. The imported venue remains a community approximation of the field.

## Robot mechanics and grasp completion

| Component | New model |
|---|---|
| Base | Dynamic 30 kg chassis, 0.65 × 0.60 × 0.16 m; world anchor removed |
| Vertical mast | Length 1.6 m, rotates about its base |
| Horizontal beam | Length 0.9 m; lift range 0–1.1 m |
| Turntable | Unbounded revolute joint around local Z, chassis-relative zero along +X |
| Fingers | 0.18 × 0.04 × 0.32 m pads; centres at ±(0.07 + displacement) m |
| Open grip | Displacements ±0.22 m, clear gap 0.54 m > 0.35 m block width |
| Closed grip | Empty targets 0 m; with a block, physical contact stops them near ±0.125 m |
| Pad contact | Coulomb coefficients `mu=mu2=2.0`, per-finger actuator effort capped at 25 N |
| Wheels | Four 0.12 m-radius wheels, 0.68 m separation, two driven joints per side |

The initial beam centre is approximately 0.47 m above ground and the finger centres
are 0.25 m below it. Thus the bottom position puts the pads over a floor-resting cube
while keeping the tips above ground. Finger and lift axes are defined in the moving
arm frames, so they rotate with the turntable instead of remaining world-aligned.

Closing around a cube cannot reach the empty gripper's zero displacement. Each pad
therefore has a contact sensor. `gripper_subsystem::at_target()` accepts either empty
position completion or contacts from **both pads on the same Earth Block**, fresh
within 0.25 s and sustained for 0.1 s. New grip requests invalidate this contact
history. No artificial fixed joint or teleportation attaches the block.

This preserves meaningful sequential composition: Q waits until the grip finishes
before starting the lift. A retains its parallel semantics and opens immediately
while lowering. Contact detection is a completion signal, not a continuous grasp
recovery controller; an object can still be lost under untested motions/collisions.

## Yaw and keyboard ownership

`yaw_subsystem::set_angle(double degrees)` accepts finite values in [0,360), returns
false for invalid values, converts to radians and continuously publishes its target.
`set_yaw_angle_command` owns the subsystem requirement and finishes on fresh angular
feedback. `yaw_zero_command` and `yaw_half_turn_command` provide the requested presets.
The `yaw/target_degrees` ROS request schedules the same generic command, so it
participates in scheduler interruption/ownership instead of bypassing the subsystem.

| Input | Owner | Effect |
|---|---|---|
| Up / Down | Gazebo TriggeredPublisher | Chassis forwards/backwards |
| Left / Right | Gazebo TriggeredPublisher | Chassis counterclockwise/clockwise |
| X | Gazebo TriggeredPublisher | Chassis stop |
| W / S | test_mode | Lift top/bottom |
| E / D | test_mode | Grip open/close |
| Q / A | test_mode | Sequential close/raise; parallel open/lower |
| R / F | test_mode | Turntable 0°/180° |
| `yaw/target_degrees` | simulation_robot → scheduler | Arbitrary valid degree target |

This follows the requested [Gazebo moving-robot tutorial](https://gazebosim.org/docs/latest/moving_robot/).
The Harmonic [DiffDrive API](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)
allows repeated left/right joint entries, so it drives all four wheels with no
additional ROS wheel PID. Arrows are ignored by test_mode to avoid commanding the arm
at the same time. KeyPublisher supplies press events only, so drive commands latch
until another direction or X. Letter-key repeats are suppressed while their command
is active. Generated numeric trigger codes come from the existing key enum.

## File map

Paths below are relative to the package root.

| File | Responsibility |
|---|---|
| `code/include/m2_gz_simulation/control/cascaded_pid.h` | Two-loop controller and inner-loop state, limits, filter and feedforward |
| `code/include/m2_gz_simulation/control/position_pid.h` | Existing outer PID and velocity/acceleration limiting, reused by the cascade |
| `code/{include/m2_gz_simulation,src}/control/joint_controller.*` | ROS target/effort interface; shared lift/yaw control, shortest-angle error and fault handling |
| `code/{include/m2_gz_simulation,src}/control/lift_controller.*` | Compatibility alias to the shared joint controller; no duplicate controller implementation |
| `code/{include/m2_gz_simulation,src}/control/joint_feedback.*` | Configurable named-axis feedback; two instances share clock settings |
| `code/{include/m2_gz_simulation,src}/nodes/control/control_node.*` | Owns both axis feedback/controller pairs; default configuration runs at 200 Hz |
| `code/{include/m2_gz_simulation,src}/subsystems/yaw_subsystem.*` | Degree API, radian target publication and yaw completion |
| `code/include/m2_gz_simulation/commands/yaw/set_angle.h`, `code/src/commands/set_yaw_angle.cpp` | Arbitrary-angle command plus 0°/180° commands |
| `code/{include/m2_gz_simulation,src}/subsystems/gripper_subsystem.*` | Larger open travel, two contact subscriptions and grasp completion |
| `code/include/m2_gz_simulation/subsystems/motion_completion.h` | Adds wrap-aware angular completion while preserving linear checks |
| `code/{include/m2_gz_simulation,src}/subsystems/lift_subsystem.*` | New 0–1.1 m travel and floor pickup bottom preset |
| `code/{include/m2_gz_simulation,src}/base/simulation_robot.*`, `base/subsystem_container.h` | Registers yaw, routes degree requests and releases resources |
| `code/src/modes/test_mode.cpp` | New letter bindings; arrows delegated to Gazebo |
| `scripts/generate_world.py` | Reproducible robot, three blocks, physics and chassis keyboard mapping |
| `scripts/import_venue.py` | Reproducible pinned-venue adaptation |
| `model/two_arm_robot.sdf`, `model/robocon_ground/` | Generated world/robot and installed field assets/provenance |
| `config/*.yaml`, `config/gazebo_gui.config` | New gains, units, bridge routes, geometry presets and field camera |
| `launch/simulation.launch.py` | Resource search path and fsm/position/effort process selection |
| `package.xml`, CMake files | Contact-message dependencies; install assets/scripts; register tests |
| `test/control_test.cpp`, `test/keyboard_command_test.cpp` | Controller, scheduler, yaw validation and contact regressions |
| `test/world_contract_test.py`, `test/simulation_smoke.py` | Structural checks and real physics tests |

## Validation results

- 14 C++ cases passed: six controller cases and eight keyboard/command/contact cases.
- Five structural model checks passed (reported by CTest as one Python test).
- SDF validation passed using the package model search path.
- All three headless launch modes passed, including clean shutdown.
- FSM grasp: cube centre rose from about 0.175 m to **1.27407 m**.
- Loaded lift holding effort measured **45.0701 N**, close to the expected 45.126 N
  for a 4.6 kg combined mass.
- The real cube remained above 1.20 m and contacted both pads through 180° and back.
- Verified lowering/release, parallel open/lower, degree wrap (359°→1°), invalid 360°
  rejection, all four chassis key directions and X stop.
- Standalone position mode passed lift/yaw control, pause/resume and world-reset
  recovery. Direct effort mode verified force actuation with no competing ROS PID.

Logs and JSON measurements are saved in `/home/thunder/ros2_ws/log/m2_robocon_review/`.
The automated tests exercise headless physics, world-space chassis displacement and the actual Gazebo keyboard topic;
desktop key focus and GUI rendering were not automatically inspected. Competition
size compliance, stair-climbing capability and cardboard deformation are not claimed.