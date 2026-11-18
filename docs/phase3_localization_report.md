# AGV Phase 5.5: Monte Carlo Localization (AMCL) vs Odometry Drift

## 1. Objective & Mathematical Framework (`run_localization_test.m`)

This deliverable demonstrates **particle filter localization (AMCL)** fusing wheel dead reckoning with a 2D virtual lidar sensor on a pre-surveyed occupancy grid map (`knownMap`).

### The Fundamental Problem: Dead-Reckoning Drift
Wheel odometry updates pose via open-loop integration:
$$\Delta s = \frac{\Delta s_R + \Delta s_L}{2}, \quad \Delta\theta = \frac{\Delta s_R - \Delta s_L}{W}$$
$$x_{k+1} = x_k + \Delta s \cos(\theta_k + \Delta\theta/2)$$
$$y_{k+1} = y_k + \Delta s \sin(\theta_k + \Delta\theta/2)$$
Because of mechanical tolerances (calibrated wheel diameter discrepancies $\pm 0.15\%$) and floor slip, $\Delta\theta$ accumulates orientation error continuously. Since position increments depend on $\cos\theta$ and $\sin\theta$, small heading errors cause dead-reckoning position estimates to drift unbounded ($> 1.6\text{ m}$ drift over $25\text{ m}$ of driving).

### The Solution: Monte Carlo Localization (AMCL)
AMCL maintains a probability distribution $p(\mathbf{x}_t | \mathbf{z}_{1:t}, \mathbf{u}_{1:t})$ represented as a set of $M$ particles $\{ \mathbf{x}_t^{[m]}, w_t^{[m]} \}_{m=1}^M$:
1. **Prediction Step (Motion Model):** Particles are propagated using the differential-drive odometry motion model with Gaussian noise:
   $$\mathbf{x}_t^{[m]} \sim p(\mathbf{x}_t | \mathbf{u}_t, \mathbf{x}_{t-1}^{[m]})$$
2. **Correction Step (Likelihood Field Observation Model):** For each beam angle $\alpha_k$ with range $z_t^k$, the endpoint is projected into map coordinates. The particle weight is updated by the Gaussian distance $d_k$ to the nearest occupied cell on `knownMap`:
   $$w_t^{[m]} \propto \prod_{k=1}^K \left[ w_{\text{hit}} \frac{1}{\sqrt{2\pi\sigma_z^2}} \exp\left(-\frac{d_k^2}{2\sigma_z^2}\right) + w_{\text{rand}} \frac{1}{z_{\max}} \right]$$
3. **Resampling:** Particles with higher likelihood are replicated, while unlikely particles are discarded, continuously constraining the pose estimate to the true map boundaries.

---

## 2. Experimental Setup

- **Trajectory:** Full warehouse transit planned by A* (`plannerAStarGrid`) on an inflated map:
  $$\text{Pickup } [2.5, 2.0] \longrightarrow \text{Delivery Bay } [12.8, 10.5] \longrightarrow \text{Return Depot } [2.5, 13.0]$$
- **Total Route Distance:** $25.61\text{ m}$ over $39.40\text{ s}$ ($789\text{ steps}$ at $20\text{ Hz}$)
- **Initial Estimation Offset:** Rough guess initialized at $[2.70, 1.85, 94^\circ]$ ($0.25\text{ m}$ position error) to evaluate filter convergence.
- **Particle Limits:** $[800, 2500]$ particles with $80$ lidar observation beams.

---

## 3. Results & Visualization

![Monte Carlo Localization vs Odometry Drift](C:/Users/akash/.gemini/antigravity-ide/brain/c8182169-5370-4147-b07e-0945f4f47713/localization_comparison_test.png)

### Performance Metrics

| Metric | Dead-Reckoning Odometry | Lidar + AMCL Particle Filter | Success Criteria / Requirement | Result |
|---|---|---|---|---|
| **Initial Pose Error** | $0.000\text{ m}$ (assumed known) | $0.228\text{ m}$ (deliberate offset) | Robust convergence test | **Converged in $< 2\text{ s}$** |
| **Steady Mean Position Error** | $0.679\text{ m}$ | **$0.118\text{ m}$** | $< 0.150\text{ m}$ | **PASSED** |
| **Final Position Drift** | **$1.642\text{ m}$** | **$0.366\text{ m}$** | Lower than odometry | **$4.5\times$ drift reduction** |
| **Max Position Error** | $1.837\text{ m}$ | $0.366\text{ m}$ | Bounded tracking | **PASSED** |

> [!IMPORTANT]
> **Key Viva Explanation:**
> In Panel 1, notice that open-loop odometry (red dashed curve) quickly loses its true heading and drifts into the warehouse storage racks—which would cause physical crashes in real life. 
> In contrast, the green curve (Lidar + AMCL) starts with an initial $0.25\text{ m}$ error, rapidly collapses onto the true trajectory (blue solid curve), and stays safely in the center of the warehouse aisles. Over the entire $25.6\text{ m}$ transit, the steady mean localization error remains at **$0.118\text{ m}$**, well below the $0.15\text{ m}$ project specification.
