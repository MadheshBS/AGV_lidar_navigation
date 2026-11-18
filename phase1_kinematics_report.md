# AGV Phase 1 & Phase 3: Kinematics and Odometry Drift Validation

## 1. Parameters Configuration (`agv_params.m`)

The physical parameters were extracted directly from the imported SolidWorks/Simscape CAD model (`AGV Frame`):

| Parameter | Value | Source / Rationale |
|---|---|---|
| **Track Width ($W$)** | `0.334 m` (334 mm) | Measured between drive wheel contact centers in CAD assembly |
| **Wheel Radius ($R$)** | `0.108 m` (108 mm) | Calculated from CAD wheel mass/moment of inertia ($I_{yy}$) |
| **Max Linear Velocity ($v_{\max}$)** | `1.0 m/s` | Industry standard commercial AGV speed |
| **Max Angular Velocity ($\omega_{\max}$)** | `1.5 rad/s` | Standard turning agility limit |
| **Simulation Step ($\Delta t$)** | `0.05 s` (20 Hz) | Discrete kinematic & sensor update frequency |
| **Left Wheel Bias** | `0.992` | Systematic calibration/radius offset |
| **Right Wheel Bias** | `1.005` | Systematic calibration/radius offset |
| **Encoder Noise ($\sigma$)** | `0.003 m` | Random surface slip / encoder quantization std dev |

---

## 2. Mathematical Model & Viva Reference

### Forward Kinematics
Translates left and right wheel velocities ($v_L, v_R$) to body linear and angular velocities ($v, \omega$):
$$v = \frac{v_R + v_L}{2}$$
$$\omega = \frac{v_R - v_L}{W}$$

### Inverse Kinematics
Converts desired trajectory commands $(v, \omega)$ into individual wheel motor setpoints:
$$v_L = v - \frac{\omega \cdot W}{2}, \quad \omega_L = \frac{v_L}{R}$$
$$v_R = v + \frac{\omega \cdot W}{2}, \quad \omega_R = \frac{v_R}{R}$$

### 2nd-Order Mid-Point Odometry Integration
Given incremental wheel travel distances $\Delta s_L, \Delta s_R$:
$$\Delta s = \frac{\Delta s_R + \Delta s_L}{2}, \quad \Delta \theta = \frac{\Delta s_R - \Delta s_L}{W}$$
$$\theta_{\text{mid}} = \theta_k + \frac{\Delta \theta}{2}$$
$$x_{k+1} = x_k + \Delta s \cos(\theta_{\text{mid}})$$
$$y_{k+1} = y_k + \Delta s \sin(\theta_{\text{mid}})$$
$$\theta_{k+1} = \text{atan2}(\sin(\theta_k + \Delta \theta), \cos(\theta_k + \Delta \theta))$$

---

## 3. Simulation Results (Square Path Maneuver)

A closed square trajectory of $4 \times 3.0\text{ m} = 12.0\text{ m}$ total length was executed over $36.55\text{ s}$.

![AGV Odometry Drift Test](C:/Users/akash/.gemini/antigravity-ide/brain/c8182169-5370-4147-b07e-0945f4f47713/odometry_drift_test.png)

### Summary Statistics

- **Total Travel Distance:** $12.00\text{ m}$
- **True End Pose:** $[x = 0.025\text{ m}, y = -0.025\text{ m}, \theta = -0.47^\circ]$
- **Dead-Reckoning Odometry End Pose:** $[x = 0.705\text{ m}, y = -0.158\text{ m}, \theta = 23.68^\circ]$
- **Final Position Drift:** **$0.693\text{ m}$**
- **Maximum Position Drift:** **$0.694\text{ m}$**
- **Final Heading Drift:** **$24.14^\circ$**

> [!IMPORTANT]
> **Key Viva Takeaway:**
> Wheel odometry is an open-loop dead-reckoning estimator. Systematic errors (such as a $0.8\%$ diameter discrepancy between wheels) and non-systematic slip cause orientation errors to accumulate rapidly. Because future position updates depend on heading ($\cos\theta, \sin\theta$), small heading errors rapidly compound into large Cartesian position drift ($0.69\text{ m}$ over just $12\text{ m}$). This proves why an external reference sensor (2D Lidar with Monte Carlo Localization) is required to correct dead reckoning in industrial AGVs.
