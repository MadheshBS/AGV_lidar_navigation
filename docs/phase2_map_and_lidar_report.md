# AGV Phase 5.3 & 5.4: Warehouse Map & Virtual Lidar Simulation

## 1. Warehouse Occupancy Map (`build_warehouse_map.m`)

The environment is modeled as a `binaryOccupancyMap` conforming to industrial warehouse standards:
- **Dimensions:** $20.0\text{ m} \times 15.0\text{ m}$
- **Grid Resolution:** $0.05\text{ m/cell}$ ($20\text{ cells/m}$)
- **Perimeter Walls:** $0.3\text{ m}$ solid boundary enclosing the facility
- **Storage Shelves:** 3 longitudinal rack blocks divided into upper and lower bays:
  - 4 longitudinal aisles (aisle widths: $3.5\text{ m} - 4.0\text{ m}$)
  - 3 transverse cross-aisles ($y \in [0.3, 3.0]$, $y \in [6.5, 8.5]$, $y \in [12.0, 14.7]$)
- **Occupancy Density:** $18.45\%$ occupied cells (shelves + walls)

### Two-Map Architecture (Key Project Concept)

1. **`knownMap` (Static Baseline Map):**
   - Contains only the permanent walls and shelf racks.
   - Used by the global path planner (A*) to generate optimal routes.
   - Used by the Monte Carlo Localization (AMCL) particle filter as the reference observation model.
2. **`trueWorldMap` (Ground Truth Physical World):**
   - Contains everything in `knownMap` plus **unmapped unexpected obstacles**:
     - **Pallet A ($0.8\text{ m} \times 0.8\text{ m}$):** Dropped in the central cross-aisle at $[7.3, 7.2]\text{ m}$.
     - **Crate B ($0.7\text{ m} \times 0.7\text{ m}$):** Sitting in Aisle 3 at $[12.8, 5.8]\text{ m}$.
     - **Cart C ($0.6\text{ m} \times 0.9\text{ m}$):** Parked in Aisle 4 at $[17.2, 7.4]\text{ m}$.
   - Scanned in real time by the virtual lidar to trigger local obstacle avoidance (VFH).

### Mission Waypoints

| Waypoint | Coordinates $[x, y, \theta]$ | Operational Purpose |
|---|---|---|
| **Pickup** | $[2.5\text{ m}, 2.0\text{ m}, +90^\circ]$ | Inbound receiving & loading bay |
| **Delivery** | $[12.8\text{ m}, 10.5\text{ m}, 0^\circ]$ | Target storage shelf bay in Aisle 3 |
| **Return** | $[2.5\text{ m}, 13.0\text{ m}, -90^\circ]$ | Outbound dispatch & recharge station |

---

## 2. Virtual 2D Lidar Sensor (`virtual_lidar.m`)

The range sensor models an industrial 2D safety laser scanner:
- **Field of View (FOV):** $360^\circ$ ($2\pi\text{ rad}$)
- **Angular Resolution:** $1^\circ$ ($360\text{ beams}$)
- **Range Limits:** $d_{\min} = 0.10\text{ m}$, $d_{\max} = 10.0\text{ m}$
- **Measurement Noise:** Zero-mean Gaussian noise $\mathcal{N}(0, \sigma^2)$ with $\sigma = 0.02\text{ m}$
- **Mount Offset:** $[dx = 0.20\text{ m}, dy = 0.0\text{ m}]$ mounted forward on the AGV chassis

### Ray-Tracing Equations
Given AGV pose $[x, y, \theta]$ and mount offset $[d_x, d_y]$:
$$x_s = x + d_x \cos\theta - d_y \sin\theta$$
$$y_s = y + d_x \sin\theta + d_y \cos\theta$$
$$\theta_s = \theta + \Delta\theta$$
For each beam angle $\alpha_i \in [-\pi, \pi)$:
$$d_{\text{meas}, i} = d_{\text{true}, i} + \mathcal{N}(0, \sigma^2), \quad d_{\min} \le d_{\text{meas}, i} \le d_{\max}$$
$$\mathbf{p}_{\text{hit}, i} = [x_s + d_{\text{meas}, i}\cos(\theta_s + \alpha_i), \; y_s + d_{\text{meas}, i}\sin(\theta_s + \alpha_i)]^T$$

---

## 3. Visualization Results

![Warehouse Map & Virtual Lidar Scan](C:/Users/akash/.gemini/antigravity-ide/brain/c8182169-5370-4147-b07e-0945f4f47713/warehouse_map_and_lidar.png)

### Scan Verification
When the AGV was positioned in Aisle 2 at $[x = 7.00\text{ m}, y = 5.20\text{ m}, \theta = 65^\circ]$:
- **Total Beams:** 360
- **Valid Obstacle Returns:** 358 ($99.4\%$)
- **Minimum Sensed Range:** $1.05\text{ m}$ (closest face of Shelf 1)
- **Unmapped Pallet Detection:** Beams successfully intersected and reflected off the unmapped Pallet A at $[7.3, 7.2]\text{ m}$, confirming that unmapped objects will be clearly visible to the avoidance controller.
