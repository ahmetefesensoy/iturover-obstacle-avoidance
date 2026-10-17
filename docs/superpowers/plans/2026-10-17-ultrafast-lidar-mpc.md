# Ultra-Fast LiDAR MPC Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build a ROS 1 C++ controller that uses only 10 Hz LiDAR at runtime to complete the supplied obstacle corridor at the fastest repeatedly collision-free time, targeting less than 10 seconds.

**Architecture:** A scan-conditioned, LiDAR-localized rolling obstacle model feeds structural corridor/gate perception and a dynamically reachable trajectory lattice. A lexicographic minimum-time planner chooses the fastest collision-free trajectory, while an independent 30 Hz safety filter prevents commands that cannot stop before contact.

**Tech Stack:** ROS 1 Noetic, C++14, roscpp, sensor_msgs, geometry_msgs, Eigen3, catkin, GoogleTest, Python 3 benchmark tooling

**Spec:** `docs/superpowers/specs/2026-10-17-ultrafast-lidar-mpc-design.md`

## Global Constraints

- Production navigation decisions use `/scan` only; the node must not subscribe to odometry, IMU, camera, maps, Gazebo state, or ground truth.
- Do not modify the simulator world, robot URDF, collision geometry, or `wheelAcceleration=1` setting.
- Publish motion immediately after the first valid scan.
- Reject predicted collisions before optimizing speed; no weighted score may exchange clearance for speed.
- Open-space commands may reach 6 m/s, but every command must satisfy braking, localization-confidence, latency, curvature, and gate-clearance bounds.
- The 0.365–0.446 m staggered gates must be crossed by tracking both posts, the geometric center, and the gate-plane normal.
- The production control path must finish inside the 100 ms LiDAR period and avoid unbounded allocation after initialization.
- Runtime code is C++14; measurement-only batch tooling may use Python 3.
- Existing solutions and the simulator package remain unchanged.

---

## File Map

Create the catkin package at `itürover_Answer_AhmetEfeŞensoy/ros1/solution_ultrafast_lidar_mpc/` with these responsibilities:

- `CMakeLists.txt`: library, node, tests, installation, and compiler warnings.
- `package.xml`: ROS and Eigen dependencies.
- `include/ultrafast_lidar_mpc/types.hpp`: shared value types and numeric helpers.
- `include/ultrafast_lidar_mpc/config.hpp`: validated controller parameters.
- `src/config.cpp`: parameter validation.
- `include/ultrafast_lidar_mpc/scan_geometry.hpp`, `src/scan_geometry.cpp`: scan conditioning, line fitting, discontinuities, and synthetic-scan-independent geometry.
- `include/ultrafast_lidar_mpc/scan_matcher.hpp`, `src/scan_matcher.cpp`: correlative relative-pose estimate and confidence.
- `include/ultrafast_lidar_mpc/rolling_grid.hpp`, `src/rolling_grid.cpp`: age-limited local occupancy and clearance field.
- `include/ultrafast_lidar_mpc/structural_perception.hpp`, `src/structural_perception.cpp`: course direction, gate tracks, gate crossing, and terminal wall.
- `include/ultrafast_lidar_mpc/trajectory.hpp`, `src/trajectory.cpp`: acceleration-limited propagation and continuous swept-footprint collision checks.
- `include/ultrafast_lidar_mpc/planner.hpp`, `src/planner.cpp`: coarse-to-fine minimum-time lattice and lexicographic candidate selection.
- `include/ultrafast_lidar_mpc/safety_filter.hpp`, `src/safety_filter.cpp`: braking and stale-scan command veto.
- `include/ultrafast_lidar_mpc/race_controller.hpp`, `src/race_controller.cpp`: non-ROS state machine and subsystem ownership.
- `src/race_controller_node.cpp`: ROS subscriber, publisher, timer, parameters, and telemetry.
- `config/race.yaml`: supplied-world defaults.
- `launch/reactive_robot.launch`: simulator plus controller startup.
- `test/synthetic_scan.hpp`: deterministic scan generator shared by tests.
- `test/test_config.cpp`, `test/test_scan_geometry.cpp`, `test/test_scan_matcher.cpp`, `test/test_rolling_grid.cpp`, `test/test_structural_perception.cpp`, `test/test_trajectory.cpp`, `test/test_planner.cpp`, `test/test_safety_filter.cpp`, `test/test_race_controller.cpp`: focused unit tests.
- `scripts/batch_run.py`: measurement-only repeated Gazebo runner.
- `scripts/test_batch_run.py`: deterministic benchmark parser tests.
- `README.md`: build, run, tune, and benchmark instructions.

## Build Harness Used by Every Task

From the repository root on Ubuntu 20.04 with ROS Noetic:

```bash
source /opt/ros/noetic/setup.bash
rm -rf /tmp/ultrafast_ws
mkdir -p /tmp/ultrafast_ws/src
ln -s "$PWD/itürover_Answer_AhmetEfeŞensoy/ros1/solution_ultrafast_lidar_mpc" /tmp/ultrafast_ws/src/solution_ultrafast_lidar_mpc
ln -s "$PWD/itürover_Answer_AhmetEfeŞensoy/ros1/iturover_obstacle_avoidance_assignment-main" /tmp/ultrafast_ws/src/reactive_robot_sim
cd /tmp/ultrafast_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make run_tests_ultrafast_lidar_mpc
catkin_test_results build/test_results
```

Expected final result: build succeeds with no warnings from package code and `catkin_test_results` reports zero failures.

---

### Task 1: Catkin Package, Shared Types, and Configuration Validation

**Files:**
- Create: `itürover_Answer_AhmetEfeŞensoy/ros1/solution_ultrafast_lidar_mpc/CMakeLists.txt`
- Create: `itürover_Answer_AhmetEfeŞensoy/ros1/solution_ultrafast_lidar_mpc/package.xml`
- Create: `itürover_Answer_AhmetEfeŞensoy/ros1/solution_ultrafast_lidar_mpc/include/ultrafast_lidar_mpc/types.hpp`
- Create: `itürover_Answer_AhmetEfeŞensoy/ros1/solution_ultrafast_lidar_mpc/include/ultrafast_lidar_mpc/config.hpp`
- Create: `itürover_Answer_AhmetEfeŞensoy/ros1/solution_ultrafast_lidar_mpc/src/config.cpp`
- Test: `itürover_Answer_AhmetEfeŞensoy/ros1/solution_ultrafast_lidar_mpc/test/test_config.cpp`

**Interfaces:**
- Consumes: none.
- Produces: `Vec2`, `PoseDelta`, `MotionState`, `Command`, `ScanPoint`, `ScanFrame`, `Gate`, `RaceState`, `ControllerConfig`, `wrapAngle(double)`, and `ControllerConfig::validate()` in namespace `ultrafast_lidar_mpc`.

- [ ] **Step 1: Write configuration tests before package implementation**

```cpp
TEST(Config, AcceptsRaceDefaults) {
  ControllerConfig c;
  EXPECT_NO_THROW(c.validate());
  EXPECT_DOUBLE_EQ(c.robot_radius, 0.105);
  EXPECT_DOUBLE_EQ(c.max_linear_accel, 1.0);
  EXPECT_DOUBLE_EQ(c.open_speed_command, 6.0);
}

TEST(Config, RejectsUnsafeOrImpossibleRanges) {
  ControllerConfig c;
  c.robot_radius = 0.0;
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c = ControllerConfig{};
  c.control_rate_hz = 5.0;
  EXPECT_THROW(c.validate(), std::invalid_argument);
  c = ControllerConfig{};
  c.max_linear_decel = -1.0;
  EXPECT_THROW(c.validate(), std::invalid_argument);
}
```

- [ ] **Step 2: Create the catkin manifest and test target, then confirm the missing headers fail the build**

Use package name `ultrafast_lidar_mpc`. Require `roscpp`, `sensor_msgs`, `geometry_msgs`, and `eigen`; compile with `-std=c++14 -Wall -Wextra -Wpedantic -Werror=return-type`. Add `catkin_add_gtest(test_config test/test_config.cpp)` linked to `ultrafast_lidar_core`.

Run:

```bash
cd /tmp/ultrafast_ws
catkin_make run_tests_ultrafast_lidar_mpc_gtest_test_config
```

Expected: compilation fails because `config.hpp` and its types do not exist.

- [ ] **Step 3: Define the shared types with stable names**

```cpp
struct Vec2 { double x{0.0}; double y{0.0}; };
struct PoseDelta { double dx{0.0}; double dy{0.0}; double dyaw{0.0}; double confidence{0.0}; };
struct MotionState { double v{0.0}; double w{0.0}; double confidence{0.0}; };
struct Command { double linear{0.0}; double angular{0.0}; };
struct ScanPoint { Vec2 p; double range{0.0}; double bearing{0.0}; bool raw_valid{false}; };
struct ScanFrame {
  double stamp{0.0};
  std::vector<double> conservative_ranges;
  std::vector<ScanPoint> collision_points;
  std::vector<ScanPoint> fit_points;
};
struct Gate {
  std::uint64_t id{0};
  Vec2 left_post;
  Vec2 right_post;
  Vec2 center;
  Vec2 normal;
  double width{0.0};
  double confidence{0.0};
  double age{0.0};
};
enum class RaceState { LAUNCH, RACE, GATE_APPROACH, GATE_CROSS, BRAKE, ESCAPE };
```

Reserve scan vectors to 360 elements at construction sites. Implement `wrapAngle` with `std::remainder(angle, 2.0 * M_PI)` and do not depend on ROS angles utilities.

- [ ] **Step 4: Implement exact configuration defaults and invariants**

Set defaults: radius `0.105`, static margin `0.018`, maximum forward latency `0.12`, acceleration/deceleration `1.0`, maximum angular acceleration `8.0`, open command `6.0`, control rate `30`, scan timeout `0.22`, grid `8.0 m` at `0.025 m`, grid history `0.8 s`, trajectory horizon `2.0 s`, integration step `0.025 s`, gate width `[0.30, 0.60] m`, gate minimum confidence `0.65`, and end-wall persistence `3` scans.

`validate()` must throw `std::invalid_argument` when a length/rate/acceleration is non-positive, `control_rate_hz < 20`, `scan_timeout < 0.15`, `grid_resolution > 0.05`, `integration_dt > 0.05`, the gate interval is inverted, or `open_speed_command < 1.0`.

- [ ] **Step 5: Build and run the configuration test**

```bash
cd /tmp/ultrafast_ws
catkin_make run_tests_ultrafast_lidar_mpc_gtest_test_config
catkin_test_results build/test_results/ultrafast_lidar_mpc
```

Expected: all configuration tests pass.

- [ ] **Step 6: Commit the independently buildable foundation**

```bash
git add itürover_Answer_AhmetEfeŞensoy/ros1/solution_ultrafast_lidar_mpc
git commit -m "feat: scaffold ultrafast lidar controller"
```

---

### Task 2: Conservative Scan Conditioning and Geometry Primitives

**Files:**
- Create: `include/ultrafast_lidar_mpc/scan_geometry.hpp`
- Create: `src/scan_geometry.cpp`
- Create: `test/synthetic_scan.hpp`
- Create: `test/test_scan_geometry.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `ScanFrame`, `ScanPoint`, `Vec2`, and `ControllerConfig` from Task 1.
- Produces: `ScanGeometry::condition(const sensor_msgs::LaserScan&)`, `ScanGeometry::fitLine(const std::vector<Vec2>&)`, `ScanGeometry::rangeDiscontinuities(const ScanFrame&, double)`, `LineModel`, and deterministic `SyntheticScan` helpers.

- [ ] **Step 1: Write failing tests for thin returns, noise, and line fitting**

```cpp
TEST(ScanGeometry, PreservesCloseSingleRayForCollision) {
  auto msg = SyntheticScan::constant(5.0);
  msg.ranges[180] = 0.40;
  ScanGeometry g(ControllerConfig{});
  const ScanFrame f = g.condition(msg);
  EXPECT_NEAR(f.conservative_ranges[180], 0.40, 1e-9);
  EXPECT_TRUE(std::any_of(f.collision_points.begin(), f.collision_points.end(),
      [](const ScanPoint& p) { return p.range < 0.41; }));
}

TEST(ScanGeometry, MedianOnlyAffectsFitStream) {
  auto msg = SyntheticScan::constant(2.0);
  msg.ranges[100] = 0.30;
  const ScanFrame f = ScanGeometry(ControllerConfig{}).condition(msg);
  EXPECT_NEAR(f.conservative_ranges[100], 0.30, 1e-9);
  EXPECT_GT(f.fit_points[100].range, 1.9);
}

TEST(ScanGeometry, FitsNoisyWall) {
  const auto points = SyntheticScan::linePoints(-1.0, 0.01, 80);
  const LineModel line = ScanGeometry::fitLine(points);
  EXPECT_GT(line.confidence, 0.9);
  EXPECT_NEAR(std::abs(line.normal.x), 1.0, 0.02);
  EXPECT_NEAR(std::abs(line.offset), 1.0, 0.02);
}
```

- [ ] **Step 2: Run the scan geometry tests and observe missing symbols**

```bash
cd /tmp/ultrafast_ws
catkin_make run_tests_ultrafast_lidar_mpc_gtest_test_scan_geometry
```

Expected: build fails on undefined `ScanGeometry` and `SyntheticScan`.

- [ ] **Step 3: Implement scan conversion without erasing thin obstacles**

For each ray, accept finite ranges inside `[range_min, range_max]`. Store the raw range in `conservative_ranges`. Build the fit range from the median of the ray and its immediate angular neighbors, but set the collision range to `min(raw, median)`. Convert bearing with `angle_min + index * angle_increment`; store points as `(range*cos(bearing), range*sin(bearing))`.

For invalid neighbors, compute the median from the remaining valid values. If no value is valid, store infinity and emit no point.

- [ ] **Step 4: Implement orthogonal least-squares line fitting and discontinuities**

Compute centroid and the 2×2 covariance matrix. The unit eigenvector of the smallest eigenvalue is the line normal; `offset = normal dot centroid`. Confidence is `1 - lambda_min / max(lambda_max, 1e-9)`. A discontinuity is an adjacent valid-ray range jump above the requested threshold; preserve wraparound between ray 359 and ray 0.

- [ ] **Step 5: Run focused tests and the full package suite**

```bash
cd /tmp/ultrafast_ws
catkin_make run_tests_ultrafast_lidar_mpc_gtest_test_scan_geometry
catkin_make run_tests_ultrafast_lidar_mpc
catkin_test_results build/test_results
```

Expected: all tests pass.

- [ ] **Step 6: Commit scan geometry**

```bash
git add itürover_Answer_AhmetEfeŞensoy/ros1/solution_ultrafast_lidar_mpc
git commit -m "feat: add conservative lidar geometry"
```

---

### Task 3: LiDAR-Only Correlative Motion Estimation

**Files:**
- Create: `include/ultrafast_lidar_mpc/scan_matcher.hpp`
- Create: `src/scan_matcher.cpp`
- Create: `test/test_scan_matcher.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: consecutive `ScanFrame` objects, the last `Command`, and elapsed seconds.
- Produces: `PoseDelta ScanMatcher::match(const ScanFrame&, const ScanFrame&, const Command&, double)` and `ScanMatcher::reset()`.

- [ ] **Step 1: Write tests for translation, rotation, ambiguity, and reset**

```cpp
TEST(ScanMatcher, RecoversNoisyForwardMotion) {
  const auto previous = SyntheticScan::corridorFrame(1.4, 8.0, 0.01, 11);
  const auto current = SyntheticScan::transformFrame(previous, -0.42, 0.01, -0.03, 0.01, 12);
  ScanMatcher matcher(ControllerConfig{});
  const PoseDelta d = matcher.match(previous, current, Command{4.2, 0.3}, 0.1);
  EXPECT_NEAR(d.dx, 0.42, 0.04);
  EXPECT_NEAR(d.dy, -0.01, 0.04);
  EXPECT_NEAR(d.dyaw, 0.03, 0.025);
  EXPECT_GT(d.confidence, 0.65);
}

TEST(ScanMatcher, LowersConfidenceInFeaturelessCircle) {
  const auto a = SyntheticScan::constantFrame(3.0);
  const auto b = SyntheticScan::constantFrame(3.0);
  const PoseDelta d = ScanMatcher(ControllerConfig{}).match(a, b, Command{}, 0.1);
  EXPECT_LT(d.confidence, 0.4);
}
```

- [ ] **Step 2: Confirm the matcher tests fail before implementation**

Run `catkin_make run_tests_ultrafast_lidar_mpc_gtest_test_scan_matcher`.

Expected: missing `ScanMatcher` symbols.

- [ ] **Step 3: Implement bounded coarse-to-fine correlation**

Use the command prior only to center the search:

```cpp
const double prior_dx = last_command.linear * dt;
const double prior_yaw = last_command.angular * dt;
```

Search `dx` over `prior_dx ± max(0.20, 0.45*abs(prior_dx))`, `dy` over `±0.20 m`, and `dyaw` over `prior_yaw ±0.30 rad`. Coarse steps are `(0.04 m, 0.04 m, 0.035 rad)`; refine twice around the winner with steps divided by four.

Transform current fit points into the previous frame. Project each transformed point to the corresponding previous angular bin and compare predicted range with the previous conservative range. Use Huber loss with `delta=0.08 m`, ignore points outside the previous sensor range, and require at least 60 inliers. Weight points belonging to a line fit above 0.8 confidence by 1.5.

- [ ] **Step 4: Compute confidence and reject implausible jumps**

Set confidence from inlier fraction, normalized residual, and best-to-second-best separation:

```cpp
confidence = clamp(0.45*inlier_fraction
                 + 0.35*std::exp(-rms/0.06)
                 + 0.20*clamp((second - best)/std::max(second, 1e-6), 0.0, 1.0),
                   0.0, 1.0);
```

Reject estimates exceeding `0.9 m` translation or `0.7 rad` rotation per scan by returning zero motion with confidence zero.

- [ ] **Step 5: Run matcher and regression tests**

```bash
cd /tmp/ultrafast_ws
catkin_make run_tests_ultrafast_lidar_mpc_gtest_test_scan_matcher
catkin_make run_tests_ultrafast_lidar_mpc
catkin_test_results build/test_results
```

Expected: translation and rotation tolerances pass; the ambiguous scene remains below 0.4 confidence.

- [ ] **Step 6: Commit motion estimation**

```bash
git add itürover_Answer_AhmetEfeŞensoy/ros1/solution_ultrafast_lidar_mpc
git commit -m "feat: estimate motion from lidar scans"
```

---

### Task 4: Age-Limited Rolling Occupancy and Clearance Field

**Files:**
- Create: `include/ultrafast_lidar_mpc/rolling_grid.hpp`
- Create: `src/rolling_grid.cpp`
- Create: `test/test_rolling_grid.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: conditioned scan frames and `PoseDelta`.
- Produces: `RollingGrid::update(const ScanFrame&, const PoseDelta&, double)`, `occupied(const Vec2&)`, `clearance(const Vec2&)`, `reset()`, and grid dimensions fixed at construction.

- [ ] **Step 1: Write tests for motion compensation, expiry, and clearance**

```cpp
TEST(RollingGrid, WarpsOldHitsIntoCurrentRobotFrame) {
  RollingGrid grid(ControllerConfig{});
  grid.update(SyntheticScan::singleHitFrame(2.0, 0.0), PoseDelta{}, 0.0);
  grid.update(SyntheticScan::emptyFrame(), PoseDelta{0.5, 0.0, 0.0, 1.0}, 0.1);
  EXPECT_TRUE(grid.occupied(Vec2{1.5, 0.0}));
}

TEST(RollingGrid, ExpiresHistoryAfterConfiguredLifetime) {
  RollingGrid grid(ControllerConfig{});
  grid.update(SyntheticScan::singleHitFrame(1.0, 0.0), PoseDelta{}, 0.0);
  grid.update(SyntheticScan::emptyFrame(), PoseDelta{}, 0.81);
  EXPECT_FALSE(grid.occupied(Vec2{1.0, 0.0}));
}

TEST(RollingGrid, ClearanceMatchesNearestOccupiedCell) {
  RollingGrid grid(ControllerConfig{});
  grid.update(SyntheticScan::singleHitFrame(1.0, 0.0), PoseDelta{}, 0.0);
  EXPECT_NEAR(grid.clearance(Vec2{0.5, 0.0}), 0.5, 0.04);
}
```

- [ ] **Step 2: Run the rolling-grid target and verify missing implementation failure**

Run `catkin_make run_tests_ultrafast_lidar_mpc_gtest_test_rolling_grid`.

- [ ] **Step 3: Implement a preallocated robot-centered grid**

Allocate `ceil(8.0/0.025)^2` cells once. Each cell stores last-hit timestamp and occupied byte. On update, inverse-transform active occupied cell centers by the new `PoseDelta` into the current robot frame, write them into a spare preallocated buffer, swap buffers, then insert current collision points. Discard cells outside the grid and timestamps older than `grid_history_seconds`.

- [ ] **Step 4: Compute a conservative distance field every scan**

Initialize occupied cells to zero and free cells to infinity. Run forward and backward 8-neighbor chamfer passes using costs `1` and `sqrt(2)`, then multiply by resolution. `clearance()` returns zero outside the grid so candidates cannot escape unobserved space.

- [ ] **Step 5: Run all tests and confirm no allocation growth**

Add a test that records `grid.capacitySignature()` before and after 100 updates and expects equality. Run the full harness and expect zero failures.

- [ ] **Step 6: Commit rolling occupancy**

```bash
git add itürover_Answer_AhmetEfeŞensoy/ros1/solution_ultrafast_lidar_mpc
git commit -m "feat: add rolling lidar occupancy grid"
```

---

### Task 5: Course Direction, Gate Tracking, and End-Wall Detection

**Files:**
- Create: `include/ultrafast_lidar_mpc/structural_perception.hpp`
- Create: `src/structural_perception.cpp`
- Create: `test/test_structural_perception.cpp`
- Modify: `test/synthetic_scan.hpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `ScanFrame`, `PoseDelta`, and previous structural state.
- Produces: `NavigationStructure StructuralPerception::update(const ScanFrame&, const PoseDelta&)`, where `NavigationStructure` contains `course_heading`, `course_confidence`, `boost::optional<Gate> active_gate`, `gate_crossed`, and `terminal_wall`.

- [ ] **Step 1: Write exact synthetic gate and direction tests**

```cpp
TEST(StructuralPerception, FindsNarrowGateCenterNotDeepestRay) {
  const ScanFrame f = SyntheticScan::wallWithGap(
      1.20, -0.18, 0.20, 6.0, -0.10);  // plane, edges, depth, side-ray bias
  StructuralPerception p(ControllerConfig{});
  const auto s = p.update(f, PoseDelta{});
  ASSERT_TRUE(static_cast<bool>(s.active_gate));
  EXPECT_NEAR(s.active_gate->width, 0.38, 0.025);
  EXPECT_NEAR(s.active_gate->center.y, 0.01, 0.02);
  EXPECT_NEAR(s.active_gate->normal.y, 0.0, 0.08);
}

TEST(StructuralPerception, KeepsGateIdentityThroughPartialOcclusion) {
  StructuralPerception p(ControllerConfig{});
  const auto a = p.update(SyntheticScan::wallWithGap(1.2, -0.2, 0.2, 5.0, 0.0), PoseDelta{});
  const auto b = p.update(SyntheticScan::partiallyOccludedGate(1.0, -0.2, 0.2), PoseDelta{0.2, 0.0, 0.0, 0.9});
  ASSERT_TRUE(a.active_gate && b.active_gate);
  EXPECT_EQ(a.active_gate->id, b.active_gate->id);
}

TEST(StructuralPerception, RejectsRearOpeningAsCourseDirection) {
  StructuralPerception p(ControllerConfig{});
  const auto s = p.update(SyntheticScan::frontAndRearCorridors(), PoseDelta{});
  EXPECT_LT(std::abs(wrapAngle(s.course_heading)), 0.35);
}
```

- [ ] **Step 2: Verify the structural tests fail**

Run `catkin_make run_tests_ultrafast_lidar_mpc_gtest_test_structural_perception`.

- [ ] **Step 3: Implement corridor direction from robust wall lines**

Fit line hypotheses to contiguous fit-point arcs of at least 12 rays. Keep lines longer than 0.5 m and confidence above 0.75. Convert line tangents to headings modulo π. Select the cluster nearest the prior course heading and update with a confidence-weighted circular mean. Initialize prior heading to zero, the robot's starting forward direction. Never switch by π unless `allow_reverse_escape` is explicitly supplied by the controller.

- [ ] **Step 4: Detect and associate gate posts**

Generate post candidates from paired range discontinuities that lie on a common barrier plane. Accept physical width `[0.30, 0.60] m`, behind-gap depth at least `0.70 m` beyond the post plane, and gate normal within 70° of course direction. Score with line support, width stability, depth, and temporal agreement. Associate to the previous gate when center distance is below 0.25 m, normal change below 20°, and width change below 0.10 m; preserve ID and exponentially smooth endpoints with alpha 0.55.

Set `gate_crossed` only when the locally integrated robot center changes side of the locked plane and both posts remain in the rear hemisphere for two scans. Do not clear the gate from a single wide forward ray.

- [ ] **Step 5: Detect the terminal wall structurally**

Require a transverse line spanning at least 1.8 m, centered within 0.35 m of the forward axis, closer than 1.8 m, with no gap wider than `2*robot_radius + 2*static_margin`. Assert `terminal_wall` after three consecutive scans and reset the streak on any passable opening.

- [ ] **Step 6: Run structural and full tests**

Run the full build harness. Expected: biased deepest rays do not move the gate center, partial occlusion preserves gate ID, and terminal-wall persistence requires exactly three scans.

- [ ] **Step 7: Commit structural perception**

```bash
git add itürover_Answer_AhmetEfeŞensoy/ros1/solution_ultrafast_lidar_mpc
git commit -m "feat: track corridor gates from lidar"
```

---

### Task 6: Dynamics, Trajectory Generation, and Continuous Collision Checking

**Files:**
- Create: `include/ultrafast_lidar_mpc/trajectory.hpp`
- Create: `src/trajectory.cpp`
- Create: `test/test_trajectory.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `MotionState`, `ControllerConfig`, `RollingGrid`, and a `ControlProfile` containing target velocity and start/end curvature.
- Produces: `Trajectory propagate(const MotionState&, const ControlProfile&, double)`, `CollisionResult checkSwept(const Trajectory&, const RollingGrid&, double)`, and `double stoppingDistance(double, double, double)`.

- [ ] **Step 1: Write dynamics and between-sample collision tests**

```cpp
TEST(Trajectory, HonorsOneMeterPerSecondSquaredAcceleration) {
  TrajectoryModel m(ControllerConfig{});
  const auto t = m.propagate(MotionState{0.0, 0.0, 1.0}, ControlProfile{6.0, 0.0, 0.0}, 2.0);
  EXPECT_NEAR(t.samples.back().v, 2.0, 0.03);
  EXPECT_NEAR(t.samples.back().x, 2.0, 0.06);
}

TEST(Trajectory, DetectsObstacleBetweenIntegrationSamples) {
  RollingGrid grid = SyntheticScan::gridWithPoint(Vec2{0.50, 0.0});
  Trajectory t = SyntheticScan::twoSampleTrajectory(Vec2{0.0, 0.0}, Vec2{1.0, 0.0});
  const auto hit = SweptCollisionChecker(ControllerConfig{}).check(t, grid, 0.13);
  EXPECT_FALSE(hit.collision_free);
}

TEST(Trajectory, ComputesLatencyAwareStoppingDistance) {
  EXPECT_NEAR(stoppingDistance(4.0, 1.0, 0.12), 8.48, 1e-6);
}
```

- [ ] **Step 2: Confirm missing trajectory implementation fails**

Run `catkin_make run_tests_ultrafast_lidar_mpc_gtest_test_trajectory`.

- [ ] **Step 3: Implement acceleration-limited differential-drive propagation**

At each `integration_dt`, move `v` toward `target_v` by at most `accel*dt` or `decel*dt`. Interpolate curvature cubically with zero endpoint slope, set `w = curvature*v`, clamp angular acceleration to `max_angular_accel`, and integrate the exact constant-twist arc when `abs(w)>1e-6`; otherwise integrate straight motion.

- [ ] **Step 4: Implement swept-circle safety**

For every adjacent pair of trajectory samples, sample the center segment at spacing no larger than half a grid cell. Required radius is:

```cpp
robot_radius + static_margin
+ localization_sigma
+ std::max(0.0, v) * forward_latency_seconds
```

Apply the latency term only ahead of the center using an oriented capsule, not symmetrically at a gate. Return collision, first collision time, and minimum clearance. Treat outside-grid queries as collision.

- [ ] **Step 5: Run tests and add a narrow-gate capsule regression**

Add a 0.365 m aperture test with the robot exactly centered; it must remain valid at low uncertainty. Shift the trajectory center by 0.09 m and expect rejection. Run all tests.

- [ ] **Step 6: Commit trajectory mechanics**

```bash
git add itürover_Answer_AhmetEfeŞensoy/ros1/solution_ultrafast_lidar_mpc
git commit -m "feat: model fast collision-safe trajectories"
```

---

### Task 7: Lexicographic Minimum-Time Planner

**Files:**
- Create: `include/ultrafast_lidar_mpc/planner.hpp`
- Create: `src/planner.cpp`
- Create: `test/test_planner.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: `PlannerInput{MotionState, RollingGrid reference, NavigationStructure, localization_sigma, certified_speed_cap}`.
- Produces: `PlanResult MinimumTimePlanner::plan(const PlannerInput&)`, containing `Command command`, `Trajectory trajectory`, `double certified_speed_cap`, `double minimum_clearance`, `bool valid`, and diagnostic counters.

- [ ] **Step 1: Write tests proving safety outranks speed and gates outrank deep rays**

```cpp
TEST(Planner, RejectsFasterCollidingCandidate) {
  PlannerInput in = SyntheticScan::straightWithCloseObstacle();
  const auto out = MinimumTimePlanner(ControllerConfig{}).plan(in);
  ASSERT_TRUE(out.valid);
  EXPECT_LT(out.command.linear, 2.0);
  EXPECT_TRUE(out.trajectory.collision_free);
}

TEST(Planner, AcceleratesAtPhysicalLimitInOpenCorridor) {
  PlannerInput in = SyntheticScan::openPlannerInput(MotionState{1.0, 0.0, 1.0});
  const auto out = MinimumTimePlanner(ControllerConfig{}).plan(in);
  EXPECT_NEAR(out.command.linear, 6.0, 1e-9);
  EXPECT_GT(out.trajectory.samples.back().v, 2.9);
}

TEST(Planner, AimsThroughLockedGateNormal) {
  PlannerInput in = SyntheticScan::biasedGatePlannerInput();
  const auto out = MinimumTimePlanner(ControllerConfig{}).plan(in);
  EXPECT_LT(std::abs(out.trajectory.samples.back().y - in.structure.active_gate->center.y), 0.03);
}
```

- [ ] **Step 2: Run the planner test and observe missing planner symbols**

Run `catkin_make run_tests_ultrafast_lidar_mpc_gtest_test_planner`.

- [ ] **Step 3: Generate dynamically reachable coarse candidates**

Use target velocities `{0, current_v, min(open_speed_command, certified_speed_cap)}` and 31 terminal curvatures spanning the reachable angular-acceleration interval. Include five curvature centers around the direct course or gate-center solution. Generate three curvature-change shapes: hold, ease-in, and ease-out. Reserve all candidate/sample storage during planner construction.

- [ ] **Step 4: Apply hard rejection in exact order**

For each candidate:

1. reject swept collision;
2. reject stopping distance greater than clear distance along the swept path;
3. reject terminal course progress below `-0.02 m` outside `ESCAPE`;
4. for an active gate, reject any crossing point outside `gate_width/2 - robot_radius - margin`;
5. reject gate-plane crossing with heading more than 18° from the fitted normal at speeds above 1.5 m/s.

Never turn these conditions into additive costs.

- [ ] **Step 5: Rank safe candidates and refine the winner**

Rank by projected forward progress divided by horizon, then gate-center error, minimum clearance, command change, and absolute terminal curvature. Generate an 11×11 refinement around the winning target velocity and terminal curvature with quarter spacing. Return the first command of the refined winner and its certified maximum velocity.

- [ ] **Step 6: Add deterministic tie and no-solution tests**

Verify identical candidates always choose the smaller command change. In a fully blocked scan, require `valid=false` and zero linear command. Run the full suite.

- [ ] **Step 7: Commit the planner**

```bash
git add itürover_Answer_AhmetEfeŞensoy/ros1/solution_ultrafast_lidar_mpc
git commit -m "feat: plan lexicographic minimum-time motion"
```

---

### Task 8: Independent Safety Filter

**Files:**
- Create: `include/ultrafast_lidar_mpc/safety_filter.hpp`
- Create: `src/safety_filter.cpp`
- Create: `test/test_safety_filter.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: planned command, current motion estimate, current scan, current time, last scan time, and planner-certified cap.
- Produces: `SafetyDecision SafetyFilter::filter(...)` with final `Command`, `vetoed`, and `SafetyReason { NONE, STALE_SCAN, INVALID_SCAN, UNSTOPPABLE, CAP_EXCEEDED }`.

- [ ] **Step 1: Write stale-scan, missed-frame, and braking tests**

```cpp
TEST(SafetyFilter, NeverAcceleratesOnStaleScan) {
  SafetyFilter f(ControllerConfig{});
  const auto d = f.filter(Command{6.0, 0.0}, MotionState{2.0, 0.0, 0.9},
                          SyntheticScan::openFrame(), 1.0, 0.70, 6.0);
  EXPECT_TRUE(d.vetoed);
  EXPECT_EQ(d.reason, SafetyReason::STALE_SCAN);
  EXPECT_LE(d.command.linear, 2.0);
}

TEST(SafetyFilter, BrakesWhenOneMissedFrameMakesStateUnstoppable) {
  const auto scan = SyntheticScan::singleHitFrame(2.0, 0.0);
  const auto d = SafetyFilter(ControllerConfig{}).filter(
      Command{6.0, 0.0}, MotionState{2.1, 0.0, 0.9}, scan, 1.0, 0.90, 6.0);
  EXPECT_EQ(d.command.linear, 0.0);
  EXPECT_EQ(d.reason, SafetyReason::UNSTOPPABLE);
}
```

- [ ] **Step 2: Confirm the tests fail before safety-filter code exists**

Run `catkin_make run_tests_ultrafast_lidar_mpc_gtest_test_safety_filter`.

- [ ] **Step 3: Implement direct time-to-contact and command-cap checks**

Compute minimum projected distance inside the command's swept angular sector. Subtract robot radius, static margin, and localization uncertainty. Compare remaining distance with `v*scan_timeout + v²/(2*max_linear_decel)`. Veto to linear zero when the state is already unstoppable; otherwise clamp requested velocity to `min(certified_cap, sqrt(2*a*remaining_distance))`.

For stale or invalid scans, never increase speed. Command maximum braking by setting the target to zero; preserve a bounded angular command only when it increases clearance in both the raw scan and rolling grid.

- [ ] **Step 4: Run safety and full tests**

Expected: stale scans cannot accelerate, close obstacles force braking, and an open fresh scan preserves the planner command exactly.

- [ ] **Step 5: Commit safety enforcement**

```bash
git add itürover_Answer_AhmetEfeŞensoy/ros1/solution_ultrafast_lidar_mpc
git commit -m "feat: enforce lidar stopping safety"
```

---

### Task 9: Race Controller State Machine

**Files:**
- Create: `include/ultrafast_lidar_mpc/race_controller.hpp`
- Create: `src/race_controller.cpp`
- Create: `test/test_race_controller.cpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: conditioned scans and monotonic timestamps.
- Produces: `ControllerOutput RaceController::onScan(const sensor_msgs::LaserScan&)`, `ControllerOutput RaceController::onControlTick(double)`, state telemetry, and `reset()`.

- [ ] **Step 1: Write state-transition tests**

```cpp
TEST(RaceController, MovesOnFirstValidScan) {
  RaceController c(ControllerConfig{});
  const auto out = c.onScan(SyntheticScan::openRosScan(0.1));
  EXPECT_EQ(out.state, RaceState::RACE);
  EXPECT_GT(out.command.linear, 0.0);
}

TEST(RaceController, HoldsGateLockUntilPlaneCrossing) {
  RaceController c(ControllerConfig{});
  c.onScan(SyntheticScan::gateRosScan(0.1, 1.2));
  EXPECT_EQ(c.state(), RaceState::GATE_APPROACH);
  c.onScan(SyntheticScan::biasedGateRosScan(0.2, 0.8));
  EXPECT_EQ(c.state(), RaceState::GATE_CROSS);
  c.onScan(SyntheticScan::postsBehindRosScan(0.3));
  EXPECT_EQ(c.state(), RaceState::RACE);
}

TEST(RaceController, BrakesAtPersistentEndWall) {
  RaceController c(ControllerConfig{});
  c.onScan(SyntheticScan::endWallRosScan(0.1));
  c.onScan(SyntheticScan::endWallRosScan(0.2));
  const auto out = c.onScan(SyntheticScan::endWallRosScan(0.3));
  EXPECT_EQ(out.state, RaceState::BRAKE);
  EXPECT_DOUBLE_EQ(out.command.linear, 0.0);
}
```

- [ ] **Step 2: Run the state-machine test and confirm missing symbols**

Run `catkin_make run_tests_ultrafast_lidar_mpc_gtest_test_race_controller`.

- [ ] **Step 3: Wire subsystems in one deterministic scan transaction**

On each scan: condition scan; match against previous scan; derive motion from pose delta and scan interval; update grid; update structure; choose state; plan; filter; cache the certified command and cap. Reset grid and gate history when match confidence stays below 0.35 for two scans.

State rules:

- `LAUNCH -> RACE` on first valid scan and publish positive command immediately.
- `RACE -> GATE_APPROACH` when gate confidence reaches 0.65.
- `GATE_APPROACH -> GATE_CROSS` when the planned path intersects the locked plane and center error is below 0.04 m.
- `GATE_CROSS -> RACE` only on structural `gate_crossed`.
- any state `-> BRAKE` on terminal wall or safety `UNSTOPPABLE`.
- `RACE -> ESCAPE` only after three consecutive scans with no forward-safe trajectory; leave after a forward candidate exists for two scans.

- [ ] **Step 4: Implement 30 Hz between-scan behavior**

`onControlTick` follows the cached first 100 ms of the trajectory. It may reduce linear speed or apply braking but must never exceed the last scan's certified cap and must never increase speed after `scan_timeout`. Gate state and target may change only in `onScan`, preventing timer/scan controller races.

- [ ] **Step 5: Run all unit tests**

Run the full harness. Expected: state transitions are deterministic and all prior geometry, localization, planning, and safety tests remain green.

- [ ] **Step 6: Commit the controller core**

```bash
git add itürover_Answer_AhmetEfeŞensoy/ros1/solution_ultrafast_lidar_mpc
git commit -m "feat: orchestrate ultrafast race states"
```

---

### Task 10: ROS Node, Parameters, Launch, and Telemetry

**Files:**
- Create: `src/race_controller_node.cpp`
- Create: `config/race.yaml`
- Create: `launch/reactive_robot.launch`
- Modify: `CMakeLists.txt`
- Modify: `package.xml`

**Interfaces:**
- Consumes: ROS `/scan` and private parameters.
- Produces: ROS `/cmd_vel`, throttled human-readable logs, and `/ultrafast_lidar_mpc/telemetry` as `std_msgs/Float64MultiArray` for non-control measurement.

- [ ] **Step 1: Add a source-level dependency test for forbidden subscriptions**

Create a CMake script test that reads `race_controller_node.cpp`, requires the literal subscription to `"/scan"`, and fails if it contains `odom`, `imu`, `camera`, `model_states`, `ground_truth`, or `OccupancyGrid`.

Run `ctest -R production_topic_contract -V` and expect failure because the node file does not exist.

- [ ] **Step 2: Implement the ROS node with one subscriber and one command publisher**

Use a `/scan` subscriber queue of one with `tcpNoDelay()`, a `/cmd_vel` publisher queue of one, and a 30 Hz timer. Protect the controller with one mutex because ROS callbacks may run concurrently. Convert `ControllerOutput.command` directly into `geometry_msgs::Twist`; set every unused component to zero.

Load every `ControllerConfig` value from private parameters, validate once, and exit with `ROS_FATAL` on invalid values. Publish zero velocity in the destructor and on ROS shutdown.

- [ ] **Step 3: Add exact supplied-world configuration**

Write `race.yaml` using Task 1 defaults plus coarse planner counts `31`, refinement counts `11`, gate center tolerance `0.04`, high-speed gate normal tolerance `0.314159`, scan matcher minimum inliers `60`, and open command `6.0`.

- [ ] **Step 4: Create a single-command launch file**

`reactive_robot.launch` must include `reactive_robot_sim/reactive_robot_sim.launch` with `model=burger_rgbd`, `ground_truth=false`, configurable `gui` and `rviz`, then load `race.yaml` under the controller node. Starting this launch file must start both simulator and controller without a second command.

- [ ] **Step 5: Add bounded telemetry**

At 2 Hz log state, estimated velocity, match confidence, gate ID/confidence, selected command, certified cap, minimum clearance, candidate counts, planner microseconds, and safety reason. The numeric telemetry array uses the same fixed field order documented in `README.md`; logging must not allocate inside the 30 Hz timer beyond ROS message storage.

- [ ] **Step 6: Build and verify the production topic contract**

```bash
cd /tmp/ultrafast_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
ctest --test-dir build/ultrafast_lidar_mpc -R production_topic_contract -V
catkin_make run_tests_ultrafast_lidar_mpc
catkin_test_results build/test_results
```

Expected: contract passes and all unit tests pass.

- [ ] **Step 7: Commit ROS integration**

```bash
git add itürover_Answer_AhmetEfeŞensoy/ros1/solution_ultrafast_lidar_mpc
git commit -m "feat: launch lidar-only race controller"
```

---

### Task 11: Closed-Loop Synthetic Course and Timing Tests

**Files:**
- Create: `test/test_closed_loop_course.cpp`
- Create: `test/fixtures/final_gate_geometry.yaml`
- Modify: `test/synthetic_scan.hpp`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: complete `RaceController` and exact final-gate geometry from the supplied world.
- Produces: deterministic closed-loop collision/success regressions and a release-mode 100 ms planning-budget test.

- [ ] **Step 1: Encode the three final narrow barriers as test-only geometry**

Store barrier centers and openings derived from the world:

```yaml
barriers:
  - {y: -5.553, left_edge: 0.312, right_edge: 0.758}
  - {y: -6.942, left_edge: -0.915, right_edge: -0.550}
  - {y: -8.339, left_edge: -0.142, right_edge: 0.264}
terminal_wall_y: -10.042
```

Use this file only in tests; production code must not install or read it.

- [ ] **Step 2: Write a closed-loop gate-sequence regression**

Simulate 10 Hz ray casts from the test geometry and 30 Hz differential-drive motion with 1 m/s² acceleration. Feed only generated `LaserScan` messages to `RaceController`. Assert every barrier plane is crossed through its opening, minimum obstacle clearance is positive, the controller reaches `BRAKE`, and no target jump exceeds 0.12 m between consecutive locked-gate scans.

- [ ] **Step 3: Write scan-noise and dropout repetitions**

Run the gate sequence for seeds 1–50 with 0.01 m Gaussian range noise and one dropped scan at each of five different approach phases. Require 50/50 collision-free stops. Verify dropped-scan cases brake or hold; none may increase velocity during the gap.

- [ ] **Step 4: Add release-mode timing and allocation enforcement**

Build `Release`, run 1,000 planner calls on the full 360-ray clutter fixture, discard the first 50 warmups, and require p99 below 80 ms and maximum below 100 ms on the validation machine. Print coarse candidates, refined candidates, and p50/p95/p99 microseconds on failure.

Expose a test-only `capacitySignature()` that returns the capacities of planner candidate, refinement, and trajectory-sample buffers. Capture it before and after the 1,000 calls and require exact equality so the hot path cannot silently grow its storage.

- [ ] **Step 5: Run the complete deterministic suite**

```bash
cd /tmp/ultrafast_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
catkin_make run_tests_ultrafast_lidar_mpc
catkin_test_results build/test_results
```

Expected: 50/50 noisy synthetic runs finish without collision and the planner remains inside budget.

- [ ] **Step 6: Commit end-to-end regressions**

```bash
git add itürover_Answer_AhmetEfeŞensoy/ros1/solution_ultrafast_lidar_mpc
git commit -m "test: validate closed-loop gate racing"
```

---

### Task 12: Measurement-Only Gazebo Batch Runner

**Files:**
- Create: `scripts/batch_run.py`
- Create: `scripts/test_batch_run.py`
- Create: `README.md`
- Modify: `launch/reactive_robot.launch`
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: launched simulation and measurement topics in a separate process.
- Produces: one JSON record per run plus aggregate completion rate, time mean/variance, minimum clearance, peak command, peak measured speed, and planner latency percentiles.

- [ ] **Step 1: Write parser tests for successful, contact, timeout, and malformed records**

Use Python `unittest` with four fixed JSON-line fixtures. Assert sample variance uses `statistics.variance` for two or more runs and zero for one run. Assert a contact always makes the run unsuccessful even if it reached the terminal region.

- [ ] **Step 2: Run parser tests and confirm the missing module failure**

```bash
python3 -m unittest scripts.test_batch_run -v
```

Expected: import failure before `batch_run.py` exists.

- [ ] **Step 3: Implement isolated measurement and process control**

The runner launches `roslaunch ultrafast_lidar_mpc reactive_robot.launch gui:=false rviz:=false ground_truth_measurement:=true`, waits for a controller-ready log, starts timing at the first nonzero `/cmd_vel`, and stops at terminal `BRAKE`, contact, process death, or 30 seconds. Ground truth and Gazebo state are read only by the runner process; verify with `rosnode info /ultrafast_lidar_mpc` that the production node still subscribes only to `/scan` and `/clock`.

Write results atomically to a run-specific directory outside the source tree. Send SIGINT for shutdown, wait five seconds, then SIGKILL only remaining child processes.

- [ ] **Step 4: Implement the staged speed sweep**

Run ceilings `2, 3, 4, 5, 6 m/s`, five runs each. Advance to the next ceiling only when all five finish without contact. At the fastest 5/5 ceiling, run 20 final repetitions. Rank configurations by completion rate first, then mean completion time, then variance.

- [ ] **Step 5: Document exact operator commands and telemetry fields**

Document build harness, launch command, batch command, production topic contract, parameter groups, output JSON schema, and the difference between control-only LiDAR input and measurement-only ground truth.

- [ ] **Step 6: Run parser tests and dry-run argument validation**

```bash
python3 -m unittest scripts.test_batch_run -v
python3 scripts/batch_run.py --runs 0
```

Expected: parser tests pass; invalid run count exits nonzero with `--runs must be positive`.

- [ ] **Step 7: Commit benchmark tooling and documentation**

```bash
git add itürover_Answer_AhmetEfeŞensoy/ros1/solution_ultrafast_lidar_mpc
git commit -m "test: benchmark ultrafast gazebo runs"
```

---

### Task 13: Gazebo Validation and Fastest Safe Configuration

**Files:**
- Modify: `config/race.yaml` only when supported by batch evidence.
- Create: `docs/benchmark-results.md`

**Interfaces:**
- Consumes: completed production package and batch runner.
- Produces: reproducible fastest collision-free configuration and measured limits.

- [ ] **Step 1: Establish a collision-free 2 m/s baseline**

Run:

```bash
python3 scripts/batch_run.py --speed 2.0 --runs 5 --output /tmp/ultrafast-2ms
```

Expected gate: 5/5 completion, zero contact, and all planner cycles below 100 ms. If this gate fails, diagnose the failing subsystem from telemetry and add a deterministic regression before changing parameters.

- [ ] **Step 2: Sweep speed ceilings without weakening geometry margins**

Run 3, 4, 5, and 6 m/s ceilings with five repetitions each. Stop increasing after the first contact or incomplete run. Do not reduce robot radius, raw-ray collision coverage, braking acceleration assumptions, or gate post margin to pass a faster tier.

- [ ] **Step 3: Tune only evidence-linked parameters**

Permitted tuning variables are trajectory horizon, candidate density, gate association smoothing, course-direction smoothing, and open/gate speed caps. For each change, record the observed failure, exact parameter delta, and five-run result in `docs/benchmark-results.md`. Revert any change that lowers completion rate.

- [ ] **Step 4: Run the final 20-run validation**

At the fastest tier that achieved 5/5, run:

```bash
python3 scripts/batch_run.py --speed selected --runs 20 --output /tmp/ultrafast-final
```

Replace `selected` with the numeric ceiling proven in Step 2. Acceptance requires 20/20 completion, zero contact, planner p99 below 80 ms, and no safety invariant violation. Report mean, sample variance, fastest, slowest, peak command, peak measured speed, and minimum clearance.

- [ ] **Step 5: Run final verification from a clean catkin workspace**

Recreate `/tmp/ultrafast_ws` using the build harness, then run build, all C++ tests, Python tests, source contract, and one headless smoke run. Capture command output in `docs/benchmark-results.md`.

- [ ] **Step 6: Commit only the evidence-backed configuration and results**

```bash
git add itürover_Answer_AhmetEfeŞensoy/ros1/solution_ultrafast_lidar_mpc/config/race.yaml \
        itürover_Answer_AhmetEfeŞensoy/ros1/solution_ultrafast_lidar_mpc/docs/benchmark-results.md
git commit -m "perf: select fastest collision-free race profile"
```

---

## Final Review Gate

Before declaring completion, verify all of the following from fresh command output:

- production node topic inspection shows only `/scan` and `/clock` subscriptions;
- all C++ and Python tests pass;
- forbidden-topic source contract passes;
- release planner p99 is below 80 ms and maximum below 100 ms;
- synthetic noisy/dropout suite completes 50/50 without collision;
- final Gazebo batch completes 20/20 without contact;
- terminal wall produces a complete stop;
- the reported fastest time comes from the repeatable final configuration;
- `git status --short` contains no unintended generated files.
