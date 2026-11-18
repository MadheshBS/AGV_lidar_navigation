%% plan_cargo_mission.m
% Generates the complete 5-phase multi-stop Autonomous Cargo Mission:
%   Phase 1: Depot [3.0, 3.0] -> Inbound Pickup Station [3.5, 12.5] (Empty, 0 kg)
%   Phase 2: Cargo Loading Docking Pass (2.5 s pause, 0 -> 20 kg)
%   Phase 3: Loaded Transit -> Outbound Delivery Station [21.5, 4.0] (Loaded, 20 kg)
%   Phase 4: Cargo Unloading Docking Pass (2.5 s pause, 20 -> 0 kg)
%   Phase 5: Return to Home Depot [3.0, 3.0] (Empty return, 0 kg)
%
% Generates synchronized signals:
%   sim_traj_x, sim_traj_y, sim_traj_yaw (AGV motion)
%   sim_cargo_x, sim_cargo_y, sim_cargo_yaw (3D Cargo Crate motion)
%   sim_payload_mass (0 kg -> 20 kg -> 0 kg)
%   sim_normal_force (Wheel ground reaction force)
%   sim_motor_torque (Motor drive torque demand)
%   sim_battery_power & sim_battery_energy (Instantaneous power & cumulative kJ)
%   sim_mission_phase (Phase 1 to 5)

function mission = plan_cargo_mission()
    fprintf('=================================================================\n');
    fprintf(' Planning 5-Phase Autonomous Cargo Transport Mission (25m x 25m) \n');
    fprintf('=================================================================\n');

    % Step 1: Build 2D Occupancy Grid representing the 25m x 25m warehouse arena
    map = binaryOccupancyMap(25, 25, 5);
    
    % Boundary walls (1.5m perimeter cushion)
    [Xs, Ys] = meshgrid(0:0.05:25, 0:0.05:1.5); setOccupancy(map, [Xs(:), Ys(:)], 1);
    [Xn, Yn] = meshgrid(0:0.05:25, 23.5:0.05:25); setOccupancy(map, [Xn(:), Yn(:)], 1);
    [Xw, Yw] = meshgrid(0:0.05:1.5, 0:0.05:25); setOccupancy(map, [Xw(:), Yw(:)], 1);
    [Xe, Ye] = meshgrid(23.5:0.05:25, 0:0.05:25); setOccupancy(map, [Xe(:), Ye(:)], 1);

    % West Racks
    [Ox1, Oy1] = meshgrid(6.2:0.05:7.8, 5.0:0.05:10.0); setOccupancy(map, [Ox1(:), Oy1(:)], 1);
    [Ox2, Oy2] = meshgrid(6.2:0.05:7.8, 14.0:0.05:19.0); setOccupancy(map, [Ox2(:), Oy2(:)], 1);

    % East Racks
    [Ox3, Oy3] = meshgrid(17.2:0.05:18.8, 5.0:0.05:10.0); setOccupancy(map, [Ox3(:), Oy3(:)], 1);
    [Ox4, Oy4] = meshgrid(17.2:0.05:18.8, 14.0:0.05:19.0); setOccupancy(map, [Ox4(:), Oy4(:)], 1);

    % Central Cargo Crates
    [Ox5, Oy5] = meshgrid(11.1:0.05:13.9, 10.6:0.05:13.4); setOccupancy(map, [Ox5(:), Oy5(:)], 1);
    [Ox6, Oy6] = meshgrid(11.4:0.05:13.6, 16.9:0.05:19.1); setOccupancy(map, [Ox6(:), Oy6(:)], 1);

    % Cylindrical Columns
    [Ox7, Oy7] = meshgrid(10.5:0.05:11.5, 5.5:0.05:6.5); setOccupancy(map, [Ox7(:), Oy7(:)], 1);
    [Ox8, Oy8] = meshgrid(13.5:0.05:14.5, 5.5:0.05:6.5); setOccupancy(map, [Ox8(:), Oy8(:)], 1);

    % Staging Pallet East
    [Ox10, Oy10] = meshgrid(20.6:0.05:22.4, 11.4:0.05:13.6); setOccupancy(map, [Ox10(:), Oy10(:)], 1);

    % Inflate map with vehicle safety radius (0.65m)
    inflated = copy(map);
    inflate(inflated, 0.65);

    sv = validatorOccupancyMap;
    sv.Map = inflated;
    sv.ValidationDistance = 0.2;
    planner = plannerHybridAStar(sv, 'MinTurningRadius', 0.8, 'MotionPrimitiveLength', 0.8);

    % Mission Waypoint Stations
    depotPose    = [3.0, 3.0, 0];
    pickupPose   = [3.5, 12.5, pi/2];
    deliveryPose = [21.5, 4.0, -pi/2];

    fprintf('[1/4] Planning Leg 1: Depot [3.0, 3.0] -> Pickup Station [3.5, 12.5]...\n');
    w1 = plan_path_segment(planner, depotPose, pickupPose);

    fprintf('[2/4] Planning Leg 2: Pickup Station [3.5, 12.5] -> Delivery Station [21.5, 4.0]...\n');
    % Segmented path through central cross aisle
    midDelivery = [12.5, 8.5, -pi/4];
    w2 = plan_path_segment(planner, pickupPose, deliveryPose, midDelivery);

    fprintf('[3/4] Planning Leg 3: Delivery Station [21.5, 4.0] -> Return Depot [3.0, 3.0]...\n');
    midReturn = [12.5, 3.5, pi];
    w3 = plan_path_segment(planner, deliveryPose, depotPose, midReturn);

    fprintf('[4/4] Generating synchronized trajectory & cargo dynamics...\n');

    dt = 0.05; % 20 Hz
    v_cruise = 0.85; % m/s
    lookahead = 0.90; % m

    % Track Leg 1 (Depot -> Pickup)
    [t1, x1, y1, yaw1, v1, w_ang1] = track_segment(w1, depotPose, pickupPose, v_cruise, lookahead, dt);
    
    % Station 1 Loading Dwell (2.5 s pause at Pickup Bay)
    t_dwell1 = (0:dt:2.5)';
    x_dwell1 = repmat(x1(end), length(t_dwell1), 1);
    y_dwell1 = repmat(y1(end), length(t_dwell1), 1);
    yaw_dwell1 = repmat(yaw1(end), length(t_dwell1), 1);
    v_dwell1 = zeros(length(t_dwell1), 1);
    w_dwell1 = zeros(length(t_dwell1), 1);

    % Track Leg 2 (Pickup -> Delivery, Loaded with 20 kg Crate)
    [t2, x2, y2, yaw2, v2, w_ang2] = track_segment(w2, pickupPose, deliveryPose, v_cruise, lookahead, dt);

    % Station 2 Unloading Dwell (2.5 s pause at Delivery Bay)
    t_dwell2 = (0:dt:2.5)';
    x_dwell2 = repmat(x2(end), length(t_dwell2), 1);
    y_dwell2 = repmat(y2(end), length(t_dwell2), 1);
    yaw_dwell2 = repmat(yaw2(end), length(t_dwell2), 1);
    v_dwell2 = zeros(length(t_dwell2), 1);
    w_dwell2 = zeros(length(t_dwell2), 1);

    % Track Leg 3 (Delivery -> Return Depot)
    [t3, x3, y3, yaw3, v3, w_ang3] = track_segment(w3, deliveryPose, depotPose, v_cruise, lookahead, dt);

    % Assemble Continuous Trajectory Signals
    t_all = [t1; ...
             t1(end) + t_dwell1(2:end); ...
             t1(end) + t_dwell1(end) + t2(2:end); ...
             t1(end) + t_dwell1(end) + t2(end) + t_dwell2(2:end); ...
             t1(end) + t_dwell1(end) + t2(end) + t_dwell2(end) + t3(2:end)];

    x_all   = [x1; x_dwell1(2:end); x2(2:end); x_dwell2(2:end); x3(2:end)];
    y_all   = [y1; y_dwell1(2:end); y2(2:end); y_dwell2(2:end); y3(2:end)];
    yaw_all = [yaw1; yaw_dwell1(2:end); yaw2(2:end); yaw_dwell2(2:end); yaw3(2:end)];
    v_all   = [v1; v_dwell1(2:end); v2(2:end); v_dwell2(2:end); v3(2:end)];
    w_all   = [w_ang1; w_dwell1(2:end); w_ang2(2:end); w_dwell2(2:end); w_ang3(2:end)];

    % Time indices for the 5 Phases
    n1 = length(t1);
    n2 = n1 + length(t_dwell1) - 1;
    n3 = n2 + length(t2) - 1;
    n4 = n3 + length(t_dwell2) - 1;
    n5 = length(t_all);

    % Synchronized 3D Cargo Crate Trajectory
    % Phase 1: Resting on Pickup Station platform [3.5, 12.5]
    % Phase 2: Docking loading pass onto AGV chassis
    % Phase 3: Riding on AGV chassis [x_all(t), y_all(t), yaw_all(t)]
    % Phase 4: Unloading onto Delivery Station platform [21.5, 4.0]
    % Phase 5: Resting on Delivery Station platform while AGV returns empty
    cargo_x   = zeros(size(x_all));
    cargo_y   = zeros(size(y_all));
    cargo_yaw = zeros(size(yaw_all));

    % Phase 1: Sitting at Pickup Station
    cargo_x(1:n1)   = pickupPose(1);
    cargo_y(1:n1)   = pickupPose(2);
    cargo_yaw(1:n1) = pickupPose(3);

    % Phase 2: Loading transition onto AGV bed
    cargo_x(n1:n2)   = x_all(n1:n2);
    cargo_y(n1:n2)   = y_all(n1:n2);
    cargo_yaw(n1:n2) = yaw_all(n1:n2);

    % Phase 3: Moving synchronously on the AGV chassis
    cargo_x(n2:n3)   = x_all(n2:n3);
    cargo_y(n2:n3)   = y_all(n2:n3);
    cargo_yaw(n2:n3) = yaw_all(n2:n3);

    % Phase 4 & 5: Deposited on Delivery Station platform
    cargo_x(n3:end)   = deliveryPose(1);
    cargo_y(n3:end)   = deliveryPose(2);
    cargo_yaw(n3:end) = deliveryPose(3);

    % Dynamic Physical Signals
    m_chassis = 22.72; % kg (chassis + 2 drive wheels)
    m_cargo = 20.00;   % kg (cargo crate payload)
    g = 9.81;          % m/s^2
    R = 0.108;         % m (wheel radius)

    payload_mass = zeros(size(t_all));
    payload_mass(n2:n3) = m_cargo; % Loaded only during Phase 3

    % Smooth transition during docking
    for k = n1:n2
        alpha = (k - n1) / (n2 - n1);
        payload_mass(k) = alpha * m_cargo;
    end
    for k = n3:n4
        alpha = 1 - (k - n3) / (n4 - n3);
        payload_mass(k) = alpha * m_cargo;
    end

    total_mass = m_chassis + payload_mass;
    normal_force = total_mass * g; % Newtons (normal ground reaction force)

    % Linear acceleration a = dv/dt
    a_lin = [0; diff(v_all) ./ diff(t_all)];
    a_lin = smoothdata(a_lin, 'gaussian', 10);

    % Motor torque: rolling resistance + inertial acceleration
    c_rr = 0.025; % rolling resistance coefficient
    f_roll = c_rr * normal_force;
    tau_roll = (f_roll * R) / 2; % per drive wheel
    tau_acc = (total_mass .* a_lin * R) / 4;
    motor_torque = abs(tau_roll + tau_acc) + 0.35; % add bearing friction
    motor_torque = smoothdata(motor_torque, 'gaussian', 8);

    % Battery electrical power: 2 * (tau * omega) / eta + P_aux
    omega_wheel = abs(v_all) / R;
    p_mech = 2 * (motor_torque .* omega_wheel);
    eta = 0.85; % motor & inverter efficiency
    p_aux = 35.0; % W (sensors, computing, lidar)
    battery_power = (p_mech / eta) + p_aux; % Watts
    battery_energy = cumtrapz(t_all, battery_power) / 1000; % kJ

    % Mission Phase State (1 to 5)
    mission_phase = ones(size(t_all));
    mission_phase(n1:n2) = 2; % Loading
    mission_phase(n2:n3) = 3; % Loaded Transit
    mission_phase(n3:n4) = 4; % Unloading
    mission_phase(n4:end) = 5; % Return Empty

    % Package Time-Series Arrays for Simulink From Workspace
    mission.sim_traj_x        = [t_all, x_all];
    mission.sim_traj_y        = [t_all, y_all];
    mission.sim_traj_yaw      = [t_all, yaw_all];
    mission.sim_cargo_x       = [t_all, cargo_x];
    mission.sim_cargo_y       = [t_all, cargo_y];
    mission.sim_cargo_yaw     = [t_all, cargo_yaw];
    mission.sim_payload_mass  = [t_all, payload_mass];
    mission.sim_normal_force  = [t_all, normal_force];
    mission.sim_motor_torque  = [t_all, motor_torque];
    mission.sim_battery_power = [t_all, battery_power];
    mission.sim_battery_energy= [t_all, battery_energy];
    mission.sim_mission_phase = [t_all, mission_phase];
    mission.total_time        = t_all(end);
    mission.waypoints_leg1    = w1;
    mission.waypoints_leg2    = w2;
    mission.waypoints_leg3    = w3;
    mission.stations.depot    = depotPose;
    mission.stations.pickup   = pickupPose;
    mission.stations.delivery = deliveryPose;
    mission.map               = map;

    fprintf('      Full mission generated: %.2f seconds (%d points)\n', t_all(end), length(t_all));
    fprintf('      Phase 1 (Depot -> Pickup):    0.0s -> %.1fs (Empty)\n', t1(end));
    fprintf('      Phase 2 (Cargo Loading Dock): %.1fs -> %.1fs (Loading)\n', t1(end), t_all(n2));
    fprintf('      Phase 3 (Loaded Transit):     %.1fs -> %.1fs (Loaded: 20kg, Force: %.0fN)\n', t_all(n2), t_all(n3), max(normal_force));
    fprintf('      Phase 4 (Cargo Unload Dock):  %.1fs -> %.1fs (Unloading)\n', t_all(n3), t_all(n4));
    fprintf('      Phase 5 (Return to Depot):    %.1fs -> %.1fs (Empty Return)\n', t_all(n4), t_all(end));
    fprintf('=================================================================\n');
end

%% Helper: Plan Path Segment using Hybrid A*
function w = plan_path_segment(planner, startPose, goalPose, midPose)
    if nargin >= 4 && ~isempty(midPose)
        try
            pA = plan(planner, startPose, midPose);
            pB = plan(planner, midPose, goalPose);
            w = [pA.States(:, 1:2); pB.States(:, 1:2)];
            return;
        catch
        end
    end
    try
        p = plan(planner, startPose, goalPose);
        w = p.States(:, 1:2);
    catch
        % Fallback direct corridor waypoints if complex turn
        w = [linspace(startPose(1), goalPose(1), 25)', linspace(startPose(2), goalPose(2), 25)'];
    end
end

%% Helper: Pure Pursuit Path Tracking
function [t_arr, x_arr, y_arr, yaw_arr, v_arr, w_arr] = track_segment(w, startPose, goalPose, v_cruise, lookahead, dt)
    pp = controllerPurePursuit('Waypoints', w, ...
                               'DesiredLinearVelocity', v_cruise, ...
                               'MaxAngularVelocity', 1.2, ...
                               'LookaheadDistance', lookahead);
    cur = startPose;
    t_arr = 0;
    x_arr = cur(1);
    y_arr = cur(2);
    yaw_arr = cur(3);
    v_arr = 0;
    w_arr = 0;

    while (norm(cur(1:2) - goalPose(1:2)) > 0.45) && (t_arr(end) < 45.0)
        [v, w] = pp(cur);
        cur(1) = cur(1) + v * cos(cur(3)) * dt;
        cur(2) = cur(2) + v * sin(cur(3)) * dt;
        cur(3) = cur(3) + w * dt;
        cur(3) = atan2(sin(cur(3)), cos(cur(3)));

        t_next = t_arr(end) + dt;
        t_arr = [t_arr; t_next];
        x_arr = [x_arr; cur(1)];
        y_arr = [y_arr; cur(2)];
        yaw_arr = [yaw_arr; cur(3)];
        v_arr = [v_arr; v];
        w_arr = [w_arr; w];
    end
    % Settle to final goal pose
    cur(1) = goalPose(1);
    cur(2) = goalPose(2);
    cur(3) = goalPose(3);
    t_arr = [t_arr; t_arr(end) + dt];
    x_arr = [x_arr; cur(1)];
    y_arr = [y_arr; cur(2)];
    yaw_arr = [yaw_arr; cur(3)];
    v_arr = [v_arr; 0];
    w_arr = [w_arr; 0];
end
