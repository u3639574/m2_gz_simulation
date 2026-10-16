# Swerve, IMU, lidar and restored venue review

The chassis now uses four independent steering/rolling modules. Arrow keys
translate in chassis X/Y; Z/C rotate the chassis. The IMU and lidar publish raw
measurements through the bridge. The FSM and separate lift/yaw controller retain
their existing roles; no sensor-processing ROS node was added.

## Venue inspection and import

Fetched [jimi4-lei/gazebo_models](https://github.com/jimi4-lei/gazebo_models) and
confirmed `origin/main` still points to `1391b36e4707cdfc7071d6ad5509bc9a8b828603`.
Its Gazebo world includes only `robocon_ground`, a static model. The visual DAE
contains 24 coloured Sky Block halves (`Cube.020`–`Cube.043`, forming 12 blocks),
storage markings and venue geometry. It does **not** supply separate dynamic
Earth Block models. Consequently the condition “both types are already available”
is not satisfied: we still need our three 0.35 m, 0.6 kg Earth Blocks.

The full upstream `Untitled.dae` is now copied byte for byte to `venue.dae`.
Previously removed Sky Blocks and `Plane.015` are restored. Sky Blocks retain their
upstream static nature; making them graspable would require splitting them into
dynamic models. The adapted collision mesh now includes those blocks too.

The original Classic SDF has coarse platform boxes and omits visible ramps/stairs
from its collisions. We keep our Harmonic world and matching static triangle
collisions plus the floor box. Thus this is the full original visual scene with
corrected physical surfaces, not a verbatim launch of the upstream Classic world.
Robot spawn and the three Earth Block locations are unchanged.

## File map

| File | Responsibility / change |
|---|---|
| `scripts/import_venue.py` | Reproducible pinned import; restores all visual nodes and includes static Sky Blocks in collision geometry. |
| `model/robocon_ground/` | Installed field meshes, model description and source attribution. |
| `scripts/generate_world.py` | Canonical model definition: sensor systems/sensors, four two-joint modules, plugin parameters, feedback joints and key bindings. |
| `model/two_arm_robot.sdf` | Generated world consumed by Gazebo. Regenerate after editing the script. |
| `gazebo/swerve_kinematics.h` | Pure wheel-vector calculation and nearest equivalent steering angle. |
| `gazebo/swerve_drive.cpp` | Gazebo system subscribing to chassis Twist; reads steering positions and sets eight physical joint velocities. |
| `CMakeLists.txt`, `package.xml` | Build/install the Harmonic plugin and declare Gazebo development dependencies. |
| `launch/simulation.launch.py` | Locates the installed plugin; enables headless EGL rendering when GUI is disabled. |
| `config/ros_gz_bridge.yaml` | Adds raw `Imu` and `LaserScan` mappings; existing chassis Twist topic remains valid. |
| `code/src/modes/test_mode.cpp` | Updates the displayed key help; chassis keys still do not schedule arm commands. |
| `code/src/commands/set_grip.cpp` | Restores actual gripper completion instead of unconditional success, fixing the pre-existing sequential-command regression. |
| `test/swerve_test.cpp` | Checks inverse kinematics, reversed rolling, angle optimization and combined motion at all modules. |
| `test/world_contract_test.py` | Checks joint hierarchy, actuator ownership, complete static Sky Block scene and sensor bridges. |
| `test/keyboard_command_test.cpp` | Ensures arrows, Z/C and X do not command the arm; existing composite-command tests remain active. |
| `test/simulation_smoke.py` | Real physics/sensor regression, including payload retention, six motion directions and reset. |

## Swerve mechanics and control

Reference: [dawan0111/Gazebo-Harmonic-Swerve-Robot](https://github.com/dawan0111/Gazebo-Harmonic-Swerve-Robot),
commit `8f0ef9500cad866b7cd77c6ea79b97f34e23dea3`, especially
`amr_description/controller.py`. That project uses ROS publishers and steering/
velocity controllers. Our implementation independently expresses the same rigid-body
kinematics in a Gazebo system, which avoids bringing a second ROS control framework
into the command-architecture lesson. No upstream robot meshes or source files are
copied into this package. The reference clone remains under workspace `vendor/`.

Each module has this physical chain:

```text
base -> revolute steering joint (local +Z)
     -> small steering carrier
     -> revolute wheel joint (local -Z after the wheel's 90° roll)
     -> cylindrical wheel
```

The wheel axis is therefore carrier +Y; positive wheel rotation drives along
carrier +X. Steering rotates that direction around chassis +Z. Wheel radius is
0.12 m; module centres are `(±0.23, ±0.34)` m. There is no DiffDrive plugin.

For body velocity `(vx, vy, wz)` and module location `(xi, yi)`:

```text
wheel_vx = vx - wz * yi
wheel_vy = vy + wz * xi
angle    = atan2(wheel_vy, wheel_vx)
speed    = hypot(wheel_vx, wheel_vy)
```

The equivalent `(angle + pi, -speed)` is selected when it reduces the turn from
the **measured** steering angle. This caps the required steering change at 90°.
A zero command holds current steering angles and stops the wheels.

`SwerveDrive::PreUpdate()` runs in simulation time. It reads joint feedback,
computes the four targets, caps steering rates, waits for all steering errors to
fall below 0.10 rad, then ramps wheel speeds. Wheel speeds are scaled together
when any exceeds the maximum, preserving the combined motion direction.
Missing steering feedback stops the modules; nonfinite Twist inputs become stop
commands. Reset clears the latched Twist and all wheel targets.

| SDF parameter | Default | Meaning |
|---|---|---|
| `wheel_radius` | 0.12 m | Perimeter speed to wheel angular velocity conversion |
| `max_wheel_speed` | 0.6 m/s | Maximum wheel perimeter speed |
| `wheel_acceleration` | 0.5 m/s² | Driving speed ramp; stop/alignment commands stop immediately |
| `steering_rate` | 3 rad/s | Maximum steering velocity |
| `steering_gain` | 8 s⁻¹ | Position error to steering velocity |

This uses Gazebo velocity actuation (`Joint::SetVelocity`), allowing physics to
apply joint motion and wheel-ground contact forces. It never resets joint poses
or writes a chassis world pose. It is an ideal actuator simulation, without motor
current/torque dynamics. The lift and arm turntable continue to use their separate
cascaded PID effort control.

The chassis command is **body-relative**, not world-relative. After turning 90°,
Up moves along the robot's new forward direction. All seven key mappings use the
existing enum values from `keyboard_key.h`; the generator reads those values so
no numeric key codes are duplicated in the model-generation logic.

| Key | Chassis Twist |
|---|---|
| Up / Down | `vx = +0.2 / -0.2 m/s` |
| Left / Right | `vy = +0.2 / -0.2 m/s` |
| Z / C | `wz = +0.4 / -0.4 rad/s` |
| X | All zero |

A key replaces the entire Twist. KeyPublisher sends keypresses, not key-release
events, so commands latch until another key or X. Simultaneous keyboard axes are
not combined. Combined motion can be requested on the existing ROS topic:

```bash
ros2 topic pub --once /m2_gz_simulation/chassis/cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.15, y: 0.10}, angular: {z: 0.20}}'
ros2 topic pub --once /m2_gz_simulation/chassis/cmd_vel geometry_msgs/msg/Twist '{}'
```

Use one command source at a time. The bridge still sends this topic ROS → Gazebo.
JointState feedback now contains 12 named joints: the original four arm joints
plus four steering and four rolling joints. Arm feedback looks up names, so it
does not depend on the expanded array order.

Chassis odometry now comes from Gazebo `OdometryPublisher` at 50 Hz. It is
**ground truth**, not a wheel encoder estimate or an IMU estimator. The topic is
unchanged, but this semantic difference matters when evaluating future localization.

## Raw sensors

The setup follows the [Gazebo sensors tutorial](https://gazebosim.org/docs/harmonic/sensors/),
using the Harmonic equivalent of the user's latest-version link.

* World `Imu` system + base-link `imu` sensor: 100 Hz, topic `/imu`.
* World `Sensors` system with `ogre2` + base-link `gpu_lidar`: 10 Hz, topic `/lidar`,
  720 horizontal samples over `[-pi, pi]`, one vertical layer, range 0.08–15 m.

The IMU is centred on the base link, aligned with chassis X/Y/Z. The lidar offset
relative to that link is `(0.34, 0, 0.10)` m, with no rotation. Its scan plane is
0.34 m above ground: above the 0.32 m chassis and below the Earth Block's 0.35 m
top. Arm/gripper occlusion remains physically visible; it is not filtered away.
No random noise model is configured. The simulated stationary IMU reports about
`+9.81 m/s²` along Z and zero angular rate.

| ROS topic | Type | Header frame |
|---|---|---|
| `/m2_gz_simulation/imu` | `sensor_msgs/msg/Imu` | `two_arm_robot/base/imu` |
| `/m2_gz_simulation/scan` | `sensor_msgs/msg/LaserScan` | `two_arm_robot/base/lidar` |

Both bridges are Gazebo → ROS and non-lazy. The sensor frame names are Gazebo's
scoped defaults. The SDF contains the fixed extrinsics; this change does not add
a ROS TF tree, state estimator, SLAM, navigation, or obstacle-avoidance node.
Later sensor code should use simulation time and account for those extrinsics.
The lidar may publish infinity for rays without a return within its configured range.

`gui:=false` adds `--headless-rendering`, enabling Ogre2/EGL scans without a GUI.
This still requires a working rendering driver. Both sensors are active in all
three command-source modes.

## Gripper regression found during verification

At the start of this change, `set_grip_command::is_finished()` returned `true`.
That advanced Q's sequence as soon as the close target was issued; the existing
`SequentialWaitsForBothFingersBeforeRaising` test failed. It now delegates to
`gripper_->at_target()`, using the already-implemented fresh position/velocity or
two-pad contact completion. No new gripper controller or grasp attachment was added.

## Validation

Validated on ROS 2 Jazzy / Gazebo Sim 8.15.0 on 2026-10-16:

* Package build and `gz sdf -k`: passed.
* 17 C++ test cases and six static world checks: passed (`colcon test-result`
  reports 21 entries, including the CTest wrappers).
* Headless `fsm`, `position` and `effort` modes: passed, with clean shutdown.
* Regenerating the world/import produced identical file hashes. The visual mesh
  hash also matches upstream `Untitled.dae` exactly.

| Physical measurement | Result |
|---|---|
| Initial front lidar return in FSM mode | 0.735338 m; expected 0.735327 m from block/robot poses |
| Earth Block centre after Q lift | 1.274077 m, retained through turntable 180° and back |
| Loaded lift effort | 45.06995 N |
| Left/right key body-Y travel during sampled intervals | +0.07426 / −0.09710 m |
| Z/C body yaw during sampled intervals | +0.16717 / −0.21769 rad; matching IMU rate signs |

Motion samples include steering and acceleration transients, so their durations
and distances are not equal. The first sideways steering transition produced
about 0.028 m of cross-axis drift; steady sideways travel stayed aligned. This is
a contact-physics teaching model, not an exact trajectory tracker.

Full reports and process logs are saved in
`~/ros2_ws/log/m2_swerve_sensors_review_20261016/`.

Reproduce from a sourced workspace:

```bash
colcon build --packages-select m2_gz_simulation --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select m2_gz_simulation
colcon test-result --test-result-base build/m2_gz_simulation --verbose
python3 src/m2_gz_simulation/test/simulation_smoke.py
```

The physics test launches isolated headless worlds and checks actual model poses,
not only commanded joint values. It verifies all four translations, both chassis
yaw directions, stop, a mixed Twist, reset during motion, raw sensor timestamps,
the front lidar ray against the known Earth Block surface, IMU yaw-rate signs,
and the existing arm pickup/turn/release sequence. GUI keypress handling is exercised
by publishing the same Gazebo Int32 events as KeyPublisher; manual GUI focus and
interactive appearance are not covered by the automated test.

Observed Harmonic 8.15 limitation: the pre-existing `/poses/*` PosePublisher
diagnostic topics can retain their last pose after a world reset. In the reset
probe, fresh ground-truth odometry returned to `(3.70, 3.85)` and both sensors
continued publishing, while the diagnostic robot pose remained stale. Reset
checks therefore use OdometryPublisher for chassis position. Restart the launch
before relying on the block `/poses/*` diagnostics after a reset; this change
does not patch Gazebo's PosePublisher implementation.
