# Ultra-Fast LiDAR MPC — Design

## Objective

Build a new, independent ROS 1 controller that completes the fixed obstacle
course as quickly as the stock simulator dynamics allow while avoiding all
contacts. The controller must make runtime navigation decisions from `/scan`
only. It must not subscribe to odometry, IMU, ground-truth pose, map, camera, or
Gazebo model-state topics.

The first performance target is a successful run below 10 seconds. The broader
acceptable band is 8–12 seconds. The controller may command 5–6 m/s in open
space, but speed at every instant is selected from collision risk and vehicle
dynamics rather than held constant.

## Simulator Constraints

- The robot starts near `(0, 9.5)` facing down the corridor.
- The Burger footprint has a nominal radius of 0.105 m.
- The LiDAR supplies 360 samples at 10 Hz, with 10 m maximum range and
  approximately 0.01 m Gaussian noise.
- The differential-drive plugin updates at 30 Hz and limits wheel linear
  acceleration to 1 m/s² through `wheelAcceleration`.
- The final staggered openings are approximately 0.365–0.446 m wide. Their
  clearance is too small for deepest-ray or single-scan steering.
- With 1 m/s² acceleration, five seconds is below the physical lower bound for
  19.5 m. A straight-line accelerate-and-stop profile has an ideal lower bound
  near 8.8 seconds before obstacle detours and control error.

The solution will not modify the simulator, robot URDF, wheel acceleration,
collision geometry, or world file. Simulator modifications can be explored
later as a separate experiment.

## Package Boundary

Create a standalone catkin package:

`itürover_Answer_AhmetEfeŞensoy/ros1/solution_ultrafast_lidar_mpc/`

The package contains one production C++ node, a launch file, configuration,
geometry/planning unit tests, and an optional benchmark runner. It does not
import or modify any previous solution.

## Runtime Architecture

The production pipeline is:

`LaserScan -> scan conditioning -> LiDAR motion estimate -> local obstacle model -> structural features -> trajectory lattice -> safety filter -> cmd_vel`

### 1. Scan conditioning

Each scan is converted once into Cartesian points. Invalid values are removed;
isolated one-ray returns are retained when close because thin structures exist
in the course. A light three-sample angular median is used only for feature
fitting; collision checking always uses the conservative minimum of raw and
filtered range.

### 2. LiDAR-only motion estimation

A short-horizon correlative scan matcher estimates `(dx, dy, dyaw)` between
successive scans. The commanded differential-drive motion supplies only a
search prior; it is not treated as measured motion. Stable wall segments receive
more weight than isolated obstacle points.

The estimate maintains a robot-relative local frame and a confidence value. No
global position is required. When confidence drops, accumulated map history is
discarded and the planner falls back to the current raw scan with a reduced
speed bound.

### 3. Rolling local obstacle model

The latest scans are transformed into a short-lived rolling occupancy grid.
History is age-limited so scan-matching drift cannot leave phantom obstacles.
Both raw scan points and occupied grid cells participate in collision checks.

Obstacle inflation contains:

- the 0.105 m robot radius;
- a small static clearance margin;
- directional uncertainty derived from scan-matching covariance;
- a latency term in the forward direction.

The narrow-gate controller uses explicit post geometry instead of blindly
inflating until a real gate disappears.

### 4. Course-direction estimator

The initial robot heading defines the desired forward hemisphere. Parallel
corridor walls reinforce this direction through robust line fitting. Direction
is continuous modulo 180 degrees, but the controller permanently rejects the
backward branch unless executing an emergency escape.

This supplies global progress without a map or odometry and prevents a locally
wide opening behind the robot from becoming the target.

### 5. Structural perception

The perception layer emits two kinds of navigable structure:

- **free corridors:** collision-free angular/curvature intervals suitable for
  ordinary high-speed motion;
- **gates:** two compatible wall endpoints forming a traversable opening in a
  barrier.

A gate track stores its fitted plane, left and right posts, physical width,
center, normal, confidence, and age. Tracks are associated across scans in the
LiDAR local frame.

When a gate is selected, the target is its geometric center and plane normal,
not the deepest LiDAR ray. The track stays locked until the estimated robot
center crosses the gate plane with both posts moving into the rear hemisphere.
This directly addresses the staggered holes at the end of the course.

### 6. Minimum-time trajectory lattice

At every scan, the planner evaluates dynamically reachable differential-drive
trajectories over a 1.5–2.5 second horizon. Candidates combine:

- maximum acceleration, hold, and maximum braking profiles;
- smooth curvature polynomials rather than constant angular velocity alone;
- ordinary corridor targets and any active gate target;
- a high-resolution refinement around the best coarse candidate.

The evaluator uses swept-circle collision checks at fine time steps. Candidate
selection is lexicographic:

1. reject any predicted collision or non-stoppable state;
2. reject backward course progress;
3. maximize safe forward progress per unit time;
4. maximize clearance and gate-centering robustness;
5. minimize control discontinuity.

This prevents a weighted score from trading a small collision penalty for a
large speed reward.

### 7. Speed policy

The controller publishes immediately after the first valid scan. In open space
it commands a 5–6 m/s target so the stock drive reaches its 1 m/s² acceleration
limit. There is no artificial cruise-speed ceiling below the reachable safe
speed.

The admissible speed is bounded by:

- distance needed to brake before the nearest swept-path obstruction;
- curvature and lateral-clearance demand;
- localization and gate-fit uncertainty;
- the ability to remain inside the active gate aperture for the whole crossing;
- one missed LiDAR frame plus control latency.

Braking begins at the latest provably safe point. A gate is not assigned a
fixed slow speed: high-confidence, well-centered approaches may pass much
faster than uncertain approaches.

### 8. Between-scan control and safety

A 30 Hz control timer follows the latest planned trajectory between 10 Hz
LiDAR updates. It may continue the planned command, but it cannot increase the
last scan's certified speed bound.

An independent safety filter checks time-to-contact in the commanded swept
sector. If the current state is no longer stoppable, it replaces the planner
output with maximum braking and the lowest-risk steering direction. Stale scan,
NaN-heavy scan, lost motion estimate, or no valid trajectory always reduces
speed; none of these conditions permit blind acceleration.

## Behavioral States

The node exposes explicit states for telemetry without splitting navigation
into unrelated controllers:

- `LAUNCH`: first scan validation and immediate acceleration;
- `RACE`: ordinary minimum-time planning;
- `GATE_APPROACH`: gate plane and center locked;
- `GATE_CROSS`: plane-normal crossing without target switching;
- `BRAKE`: end wall or emergency stopping;
- `ESCAPE`: bounded recovery used only when no forward trajectory exists.

State changes do not reset localization or trajectory history. In-place spinning
is not part of normal navigation.

## End Detection

Completion is detected using LiDAR structure only: after the sequence of narrow
gate-like barriers, a persistent transverse end wall closes the forward
corridor and no passable opening exists. The robot performs maximum safe braking
and stops before contact. End detection is deliberately structural rather than
based on elapsed time, odometry, or a hard-coded coordinate.

## Parameters

Launch parameters cover physical assumptions and tuning knobs, including:

- footprint radius and clearance margins;
- acceleration/deceleration and angular limits;
- open-space command ceiling;
- scan-matcher search bounds and confidence thresholds;
- grid resolution and history lifetime;
- trajectory horizon, integration step, and lattice resolution;
- gate width limits, association thresholds, and crossing confidence;
- stale-scan and emergency time-to-contact thresholds.

Defaults target the supplied Burger world. Parameters are grouped by subsystem
and validated at startup; invalid safety-critical values terminate the node.

## Implementation Shape

Keep units independently testable:

- `scan_geometry`: scan conversion, discontinuities, line fits, and gates;
- `scan_matcher`: relative motion and confidence;
- `rolling_grid`: short-lived obstacle fusion;
- `trajectory`: differential-drive propagation and swept footprint;
- `planner`: lattice generation, refinement, and lexicographic selection;
- `safety_filter`: stopping and time-to-contact invariants;
- `race_controller`: ROS integration and state ownership.

The hot path performs no unbounded allocation. Scan buffers, grids, and
candidate arrays are preallocated. Timing for each stage is measured; a planning
cycle exceeding its 100 ms budget is treated as a safety event and reduces the
next command.

## Verification Strategy

### Unit and deterministic tests

- scan sanitization preserves close thin obstacles;
- scan matching recovers known translation/rotation under noise;
- gate extraction returns the correct center and width for all final openings;
- a one-ray obstacle invalidates every intersecting trajectory;
- swept collision checking catches impacts between integration samples;
- braking bounds remain safe with one missed scan;
- gate lock does not jump to a deeper side ray;
- backward openings cannot win ordinary planning;
- stale or invalid scans never produce acceleration;
- end-wall detection stops before contact.

Synthetic scans cover straight corridor, box/cylinder clutter, staggered gates,
partial post occlusion, grazing approaches, and noisy/dropout cases.

### Gazebo validation

Validation proceeds in measured stages:

1. low-speed collision correctness;
2. gate-center accuracy across repeated approaches;
3. progressive command ceilings of 2, 3, 4, 5, then 6 m/s;
4. repeated full-course runs at the fastest successful configuration;
5. a final batch reporting completion rate, mean time, variance, minimum
   clearance, planner latency, peak command, and peak estimated speed.

Speed is raised only after a configuration completes without contact. The final
result is the fastest configuration that remains repeatably collision-free, not
the run with the single luckiest time.

## Acceptance Criteria

- A new independent ROS 1 package builds with catkin.
- The production node subscribes to `/scan` and publishes `/cmd_vel`; it has no
  runtime pose, map, IMU, camera, or Gazebo-state dependency.
- Motion begins on the first valid scan.
- The robot crosses every staggered opening and stops before the terminal wall
  without contact.
- Unit tests pass and planner timing stays within the 10 Hz scan budget.
- Gazebo batch results identify the fastest repeatably successful setting.
- The initial performance goal is a successful run below 10 seconds; if stock
  physics or sensing prevents that result, the measured limiting factor is
  reported rather than bypassed by changing the simulator.
