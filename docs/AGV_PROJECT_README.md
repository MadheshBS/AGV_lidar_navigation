# AGV Mini Project: Lidar-Based Navigation with Obstacle Avoidance

> Use this file as the project README and as the master prompt for the coding agent (Google Antigravity).
> Read the whole file before writing any code. Follow the phases in order. Ask before changing the scope.

---

## 1. Project summary

**Title:** Design and Simulation of an Industry-Standard Automated Guided Vehicle (AGV)

**Context:** Mini project for the Autonomous Mobile Robots course. This project covers **lidar-based navigation on a known map, with obstacle avoidance**.

**Aim:** Build a simulated differential-drive AGV that
1. localizes itself on a known warehouse map using a simulated 2D lidar plus wheel odometry,
2. plans a route from a start point to a goal,
3. follows that route autonomously,
4. detects and avoids obstacles that are not on the map.

Everything runs in simulation. There is no physical hardware.

"Industry standard" means following general commercial conventions (realistic proportions, standard components, industrial-range speeds). It does **not** mean formal compliance with standards such as ISO 3691-4. Keep this wording in any report text.

---

## 2. Tools

- **MATLAB** with Navigation Toolbox (occupancy maps, `rangeSensor`, `monteCarloLocalization`, planners, `controllerPurePursuit`, `controllerVFH`)
- **Simscape / Simscape Multibody** for the physics model
- **SolidWorks** (mechanical design) and the **Simscape Multibody Link** plugin (import)
- Working folder on the user's Windows machine: `C:\AGV_MATLAB`

If a toolbox is missing, tell the user and propose a plain-MATLAB fallback instead of silently switching tools.

---

## 3. Current state (already done)

- SolidWorks chassis assembly **AGV Frame**: differential drive, two driven wheels plus one caster.
- The model is imported into Simscape Multibody as **AGV__Frame** in `C:\AGV_MATLAB`. The Link export wrote only the XML, so the 8 part STEP files were saved by hand. The model simulates without errors.
- Not done yet: floor, wheel-ground contact, odometry, lidar, map, localization, planning, avoidance.

**Do not modify the SolidWorks model.** The lidar is a virtual sensor (see section 5.4). An optional cylinder for looks can be added in Simscape later.

---

## 4. Scope decisions

- Drive: differential drive (two driven wheels, one caster).
- Sensor: simulated 2D lidar, computed in MATLAB. It is not a physical part in CAD.
- Localization: particle filter (AMCL) against a known occupancy map, with wheel odometry as the motion model.
- Planning: global planner on the occupancy map, local controller for tracking and obstacle avoidance.
- Environment: a simple warehouse map (walls, shelf blocks, pickup, delivery and return points) plus a few unexpected obstacles.
- Odometry drift is a known effect. Show it in a comparison and show that the lidar corrects it.

---

## 5. What to build

### 5.1 Specifications (Phase 1)
Put every parameter in one file, `agv_params.m`, so nothing is hard-coded elsewhere. The values below are proposed defaults. Ask the user to confirm or replace them with the real chassis numbers from the CAD model.

| Parameter | Proposed default |
|---|---|
| Wheel radius | 0.10 m |
| Wheel separation (track width) | 0.50 m |
| Max linear speed | 1.0 m/s |
| Max angular speed | 1.5 rad/s |
| Lidar max range | 10 m |
| Lidar field of view | 360 degrees |
| Lidar beams | 360 (1 degree resolution) |
| Lidar scan rate | 10 Hz |
| Lidar noise (std dev) | 0.02 m |
| Lidar mount pose (x, y, theta) | (0.2 m, 0, 0) relative to the base frame |
| Encoder noise / wheel slip | small, adjustable |

### 5.2 Kinematics and odometry (Phase 3 and 5)
- Differential-drive forward kinematics: wheel speeds to (v, omega).
- Odometry: integrate wheel increments into pose (x, y, theta). Add noise so that drift is visible.
- Deliverable: `diffdrive_odometry.m` and a test script that drives a known path and plots true vs odometry pose.

### 5.3 Map (Phase 5)
- Build a `binaryOccupancyMap` (or `occupancyMap`) of a small warehouse, about 20 m x 15 m at 0.05 m resolution.
- Include outer walls, 3 to 4 shelf blocks forming aisles, and named points: **pickup**, **delivery**, **return**.
- Deliverable: `build_warehouse_map.m` that returns the map and the named points, plus a plot.

### 5.4 Virtual lidar (Phase 5)
- Use `rangeSensor` or `rayIntersection` (or a custom ray-casting function) against the map.
- Inputs: true AGV pose and the lidar mount pose. Output: a range vector and angles.
- Add Gaussian noise. Clip to the max range.
- Unexpected obstacles must appear in the lidar view even though they are not in the known map used for localization. Keep two maps: the **known map** (for localization and planning) and the **true world** (known map plus obstacles, used to generate scans).
- Deliverable: `virtual_lidar.m` and a plot of a scan overlaid on the map.

### 5.5 Localization (Phase 6)
- Use `monteCarloLocalization` (or a hand-written particle filter) with the odometry motion model and the lidar sensor model.
- Initialize with a rough guess near the start pose.
- Log the true pose, odometry-only pose, and estimated pose at every step.
- Deliverable: `run_localization_test.m` with a plot of the three trajectories and the position error over time.

### 5.6 Planning and avoidance (Phase 7)
- **Global planner:** A* on the inflated known map (`plannerAStarGrid`) or `plannerHybridAStar`. Inflate obstacles by the AGV radius plus a margin.
- **Path follower:** `controllerPurePursuit`.
- **Local avoidance:** `controllerVFH` (preferred) or DWA, using the lidar scan. When an unexpected obstacle blocks the path, steer around it and rejoin the route. Optionally add a stop-and-wait fallback if the AGV is boxed in.
- Mission sequence: start at pickup, go to delivery, go to return.
- Deliverable: `run_mission.m` as the full 2D kinematic simulation.

### 5.7 Simscape integration (Phase 4 and 8)
- Add a floor and Spatial Contact Force blocks between each wheel and the floor, with friction set. Tune the contact parameters until the AGV drives straight under equal wheel commands.
- Drive the wheel joints from the (v, omega) commands, converted to wheel speeds.
- Read the wheel joint angles to produce encoder values for the odometry.
- Run the final demonstration here after everything works in the 2D simulation.
- Optional: a small cylinder on the chassis in Simscape to show the lidar.

---

## 6. Suggested development order

Simscape physics is slow, so **develop and tune everything in the plain 2D kinematic simulation first**, then move it into Simscape.

1. `agv_params.m`
2. Odometry and kinematics test (5.2)
3. Map (5.3)
4. Virtual lidar (5.4)
5. Localization test (5.5)
6. Planning and avoidance (5.6) and the full mission in 2D
7. Floor and wheel contact in Simscape (5.7)
8. Port the controller into the Simscape model and run the final demonstration
9. Plots, results and report

After each step, run the script, show the plot, and confirm the result with the user before moving on.

---

## 7. Final results expected

| Deliverable | Description |
|---|---|
| SolidWorks and Simscape model | AGV that moves with physics on a floor |
| Map and lidar plots | Warehouse map, lidar scan overlay |
| Planned vs actual path | How closely the route is followed |
| Localization error graph | Odometry-only error grows with time, odometry plus lidar stays small. This is the key result. |
| Obstacle avoidance run | AGV detours around an unexpected obstacle and returns to its route |
| Mission run | Pickup, delivery, return completed autonomously |
| Report or presentation | Design choices, methods, results, limitations, future work |

**Success criteria (adjust with the user):**
- AGV completes the pickup, delivery, return mission with no collision.
- Mean localization error with lidar stays below about 0.15 m, clearly lower than odometry alone over the same run.
- At least one unexpected obstacle is avoided in each demonstration run.

---

## 8. Rules for the agent

- Keep the code modular: one function per file where sensible, one parameters file, one top-level script per experiment.
- Comment the code in plain language. The user is a student who needs to explain it in a viva.
- Fix random seeds so runs are reproducible.
- Do not invent specifications the user has not confirmed. Use the proposed defaults only after flagging them.
- Do not change the SolidWorks model or the imported Simscape geometry unless asked.
- Do not expand the scope (3D lidar, SLAM, multi-robot) without asking.
- If something cannot be run in your environment, for example MATLAB or Simscape, say so clearly and write the code so the user can run it, with instructions.
- At the end of each step, give a short summary: what was built, how to run it, what the plot should show, and what comes next.

---

## 9. Report notes

- State clearly that "industry standard" means commercial design conventions, not formal ISO compliance.
- Present odometry drift as an expected effect and show how lidar localization corrects it.
- Future work: real lidar hardware, dynamic obstacles, multi-AGV traffic management, SLAM for unknown maps.
- Add a table separating this approach from the other three team members' guidance methods.

---

## 10. First task for the agent

Start with Phase 1 and the 2D simulation. Do the following in order:
1. Ask the user to confirm or replace the default parameters in section 5.1.
2. Create `agv_params.m`.
3. Create the odometry and kinematics code (5.2) with a test that drives a square and plots true vs odometry pose.
4. Stop, show the result, and wait for confirmation before building the map and lidar.
