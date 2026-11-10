# Earth Block pickup and transfer demo

This demo starts from the supplied world's spawn `(3.70, 3.85, yaw=0)` and transports
`earth_block_1` from red storage to the red Transfer Area. Start with a fresh world
for each full run. It uses ground-truth odometry and the existing FSM/subsystem
architecture. The robot, venue, wheel plugin and mechanism controllers are unchanged
by this feature.

## Run and keys

```bash
cd ~/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select m2_simlib m2_gz_simulation --cmake-force-configure --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
ros2 launch m2_gz_simulation simulation.launch.py
```

Focus Gazebo and press **T** once. For headless testing:

```bash
gz topic -t /keyboard/keypress -m gz.msgs.Int32 -p 'data: 84'
```

| Key | Command | Use |
|---|---|---|
| T | `earth_block_transfer_command` | Whole pickup/transfer sequence from the default spawn |
| G | `move_to_earth_block_command` | PID to gripping pose; open grip, lower lift and set arm yaw 0 first |
| B | `move_to_nearest_slope_command` | Curved PID approach to the closest configured ramp entrance |
| H | `move_up_slope_command` | PID ascent; first align at B's target and set arm yaw 180 |
| X | Cancel chassis actions, including the entire T sequence | Stops chassis; an interrupted lift reference holds its measured position |
| Arrows / Z / C | Cancel chassis actions and take manual control | Existing latched keyboard movement |

The enum in `keyboard_key.h` defines these codes. Other mechanism keys are ignored
while T owns the mechanisms; press X before taking over. X does not open the grip.
The arm controller retains its last yaw target if interrupted.

Read the current stage with:

```bash
ros2 topic echo /m2_gz_simulation/chassis/autopath_status --qos-durability transient_local
```

## Sequence and geometry

The sequence is implemented with `m2_fsm_cpp` sequential and parallel command groups:

1. **Prepare:** open grip and lower lift in parallel; also reset the arm to 0°.
2. **Pickup:** drive to `(4.05, 3.85, yaw=0)`. The fingers reach approximately 0.90 m
   ahead of the chassis, centering them on the block at `(4.95, 3.85)`.
3. **Close:** close the fingers and require both pads to contact the same Earth Block.
4. **Raise:** lift to the existing top position, 1.1 m joint displacement.
5. **Approach:** PID to `(3.25, 3.50, yaw=0)` in front of the red ramp.
6. **Turn arm:** rotate the arm to 180° relative to the chassis.
7. **Ascend:** PID to `(-0.60, 3.50, yaw=0)`, travelling toward world −X.
8. **Release:** open the grip and lower the lift in parallel.

As agreed, the chassis keeps yaw 0°, so ascent is **backward in the body frame**.
The arm points uphill after its 180° turn. Ramp pitch changes the horizontal reach
of the raised arm; the final chassis target accounts for this in the measured world.

The red ramp occupies X `[-0.8, 2.7]`, Y `[3, 4]`. The transfer square at its top is
X `[-1.8, -0.8]`, Y `[3, 4]`, with floor Z `0.6`. These coordinates come from the
imported mesh and the supplied rulebook's field diagrams (pages 8 and 12).
The 0.35 m block must settle with its centre inside X `[-1.625, -0.975]`,
Y `[3.175, 3.825]`, Z approximately `0.775`.

B/H select between the red and blue entrance poses by planar distance. T chooses
one side when constructed and passes it to both commands, so they cannot disagree.
Only the red-storage demo is physically validated. These are configured field moves,
not general obstacle-avoiding navigation: do not run B/H from arbitrary positions
across the central platform, or T again after the first block has been moved.

Opening and lowering in parallel releases the block from the raised arm, as requested.
The block falls onto the transfer floor. This is not a gentle place-down routine; that
would require a separate lower-while-gripping stage before opening.

## Files and responsibilities

| File (relative to package) | Responsibility |
|---|---|
| `code/include/.../commands/chassis/earth_block_motion.h` | Public declarations of the three chassis actions |
| `code/src/commands/chassis/earth_block_motion.cpp` | Nezha-style wrappers: fetch X/Y/yaw PID parameters and add a simlib PID pose child |
| `code/include/.../commands/composite/earth_block_transfer.h` | Full sequence interface and monitoring state |
| `code/src/commands/composite/earth_block_transfer.cpp` | Ordered groups, stage publication, grasp gate, payload monitoring and fault handling |
| `code/src/subsystems/chassis_subsystem.cpp` | ROS parameter facade, nearest-ramp choice and stage/fault reporting |
| `code/src/modes/test_mode.cpp` | Keyboard scheduling, autorepeat rejection and whole-sequence cancellation |
| `code/include/.../base/keyboard_key.h` | Named Gazebo key codes |
| `config/fsm_node_params.yaml` | Waypoints, PID gains, speed caps, acceleration/deceleration limits and timeouts |
| `test/earth_transfer_smoke.py` | Real Gazebo pickup, curve, ascent and full-block landing assertions |
| `test/earth_transfer_fault_smoke.py` | Real Gazebo empty-grasp rejection and cancellation during pickup |
| `test/keyboard_command_test.cpp` | PID slew, settling, stale-feedback and scheduler/ownership regressions |

Here `...` means `m2_gz_simulation`. The shared controller is in sibling package
`m2_simlib/code/src/ros2/commands/move_base/pid_pose.cpp`, with its API in
`code/include/m2_simlib/ros2/commands/move_base/pid_pose.h`.
The existing per-folder CMake helper discovers these new `.cpp` files on configure;
`--cmake-force-configure` handles an already configured build tree.

## PID logic and curve shaping

The wrappers follow Nezha's `move_to_staff_rack.cpp`: a named command reads parameters
from the chassis subsystem and contains a reusable PID pose command. No path is
registered with `m2_move_base` for these moves.

On each FSM update, the PID command:

1. Reads fresh ground-truth X/Y/yaw and body velocity.
2. Computes independent world-X, world-Y and yaw corrections, wrapping yaw error.
3. Caps each velocity and limits changes using that axis's acceleration/deceleration.
4. Rotates world XY velocity into the chassis frame and sends a standard ROS Twist
   (`linear.x`, `linear.y`, `angular.z`) through simlib's FSM velocity input.
5. Requires position/yaw tolerances **and low measured speed** for 20 successive
   updates before completing. At the configured 100 Hz this is approximately 0.2 s.

The approach initially accelerates X faster than Y; Y continues correcting as X
slows near its target. This changing velocity ratio bends the route. It is a feedback
motion with bounded velocity changes, not a time-parameterized sine/trapezoid path or
an exact geometric spline. The output is acceleration-limited, not jerk-limited.

The configured gains use the existing PID controller with **Ki=Kd=0**. P control was
sufficient in the measured simulation; increasing gain alone caused oscillation near
the approach endpoint. Different gains and slew limits remove that oscillation while
retaining speed. Simlib's inherited Ki/Kd are per-update gains, so retune them if you
change the 100 Hz loop frequency. The acceleration limiter uses ROS simulation time.

| Move | X gain | Y gain | X/Y speed caps (m/s) | X/Y acceleration (m/s²) | X/Y deceleration (m/s²) |
|---|---:|---:|---|---|---|
| Pickup | 1.4 | 2.0 | 0.28 / 0.18 | 0.30 / 0.12 | 0.40 / 0.50 |
| Approach | 1.1 | 1.0 | 0.50 / 0.16 | 0.35 / 0.12 | 0.60 / 0.50 |
| Ascent | 1.2 | 2.0 | 0.55 / 0.18 | 0.35 / 0.12 | 0.50 / 0.50 |

The swerve plugin still enforces its own wheel limits (0.6 m/s and 0.5 m/s²), so
command limits are not a guarantee of identical physical chassis acceleration.
The arm's existing sine/cascaded control settings are retained.

Targets/gains/limits are read when a command is constructed. ROS parameter changes
apply on the next keypress, not midway through a running T sequence. For example:

```bash
ros2 param set /m2_gz_simulation/fsm_node pid_pose.red_ascent_x.max_abs_output 0.45
ros2 param set /m2_gz_simulation/fsm_node motion.red_approach.y.acceleration 0.10
```

The path through the software is:

```text
T/G/B/H → FSM commands → simlib PID pose → move_base_subsystem FSM input
       → chassis/cmd_vel → bridge → Gazebo swerve plugin → wheel joints
       ← localization_subsystem ← ground-truth chassis/odometry
```

`m2_move_base` remains launched for the existing trajectory interface. PID initialization
stops its trajectory and suppresses its output, so it cannot override this sequence.

## Failure behaviour and validation

Stale/invalid odometry, a clock reset, invalid PID limits, or an expired PID target
watchdog zero the chassis command and latch failure. Failure **does not return success**
to a sequential group. The T sequence also watches stage timeouts and loss of both-pad
payload contact for more than 0.6 simulation seconds. It interrupts the active child,
retains the grip target, and waits for X/manual cancellation. An empty closed gripper
cannot advance to lift/transport. Restored odometry alone cannot resume a failed move.

The tuned headless run completed in about 37 simulation seconds. Its final block
centre was approximately `(-1.306, 3.453, 0.775)` m, fully inside the transfer square.
Measured peak planar speeds were approximately 0.28 m/s for pickup, 0.51 m/s for
approach and 0.55 m/s for ascent. The other two storage blocks were undisturbed.
Artifacts are under `/home/thunder/ros2_ws/log/transfer_validation/`.

Reproduce:

```bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws
python3 src/m2_gz_simulation/test/earth_transfer_smoke.py --output log/transfer_validation/manual_success
python3 src/m2_gz_simulation/test/earth_transfer_fault_smoke.py --output log/transfer_validation/manual_faults
ROS_DOMAIN_ID=176 colcon test --packages-select m2_gz_simulation
colcon test-result --test-result-base build/m2_gz_simulation/test_results
```

The physics scripts own and stop isolated launches; run them sequentially because
both use ROS domain 175. They are opt-in tests, not part of the normal colcon test run.
