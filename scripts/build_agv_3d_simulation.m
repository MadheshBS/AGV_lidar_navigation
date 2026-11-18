%% build_agv_3d_simulation.m
% Replicates the 3D Simscape Multibody Autonomous Navigation Simulation
% using the custom SolidWorks-imported AGV mobile robot (AGV Frame) in an
% expanded 25m x 25m warehouse arena featuring dynamic 3D cargo payload handling.
%
% Features:
%   - Exact wheel ground calibration: drive wheels sit flush on floor at Z = 0.00m
%   - 3-DOF MobileBaseJoint (Planar Joint) actuated with 2nd-order filtered trajectory signals
%   - 3D Dynamic Cargo Payload (Cargo_Payload_Box) with Planar Joint actuation
%   - Full 5-phase Autonomous Cargo Mission support (Pickup -> Transport -> Deliver -> Return)
%   - Telemetry Scopes for Payload Mass, Ground Reaction Force, Motor Torque, and Battery Power
%   - 25m x 25m warehouse arena with boundary walls, racking, and automated docking stations
%   - Hybrid A* kinodynamic path planning + Pure Pursuit trajectory generation
%   - Automatic InitFcn / PreLoadFcn callbacks loading default_trajectory.mat
%
% USAGE:
%   build_agv_3d_simulation(goalX, goalY, goalYaw)
%   build_agv_3d_simulation('cargo_mission')

function build_agv_3d_simulation(goalX, goalY, goalYaw)
    is_cargo_mission = false;
    if nargin >= 1 && (ischar(goalX) || isstring(goalX)) && strcmpi(goalX, 'cargo_mission')
        is_cargo_mission = true;
        goalX = 21.5; goalY = 4.0; goalYaw = -pi/2;
    else
        if nargin < 1 || isempty(goalX), goalX = 22.0; end
        if nargin < 2 || isempty(goalY), goalY = 22.0; end
        if nargin < 3 || isempty(goalYaw), goalYaw = pi/2; end
    end

    fprintf('=================================================================\n');
    if is_cargo_mission
        fprintf(' Building 3D Simscape Model: AUTONOMOUS 5-PHASE CARGO MISSION   \n');
        fprintf(' Flow: Depot -> Pickup [3.5, 12.5] -> Delivery [21.5, 4.0] -> Depot\n');
    else
        fprintf(' Building 3D Simscape Navigation Model for Target: [%.1f, %.1f]\n', goalX, goalY);
    end
    fprintf(' Robot: Custom Differential-Drive Mobile Robot (AGV Frame)\n');
    fprintf(' Feature: Dynamic 3D Cargo Payload Transport & Simulink Scopes\n');
    fprintf('=================================================================\n');

    src_model = 'AGV__Frame_original_backup';
    target_model = 'agv_3d_nav';

    if bdIsLoaded(target_model), close_system(target_model, 0); end
    if bdIsLoaded(src_model), close_system(src_model, 0); end

    % Load required libraries & CAD data
    load_system('sm_lib');
    load_system('nesl_utility');
    load_system('simulink');
    run('AGV__Frame_DataFile.m');

    fprintf('[1/6] Loading base AGV mobile robot model...\n');
    load_system(src_model);
    save_system(src_model, target_model);
    close_system(src_model, 0);
    load_system(target_model);

    % Safeguard: Remove algebraic kinematic-loop cut joints from CAD import
    cutJoints = {'Cylindrical1', 'Cylindrical2', 'Planar'};
    for k = 1:length(cutJoints)
        b = [target_model '/' cutJoints{k}];
        if getSimulinkBlockHandle(b) ~= -1
            delete_block(b);
            fprintf('      Removed cut joint: %s\n', cutJoints{k});
        end
    end

    % Remove original static ground Transform block
    tr_blk = [target_model '/Transform'];
    if getSimulinkBlockHandle(tr_blk) ~= -1
        lines = get_param(tr_blk, 'LineHandles');
        if lines.LConn(1) ~= -1, delete_line(lines.LConn(1)); end
        if lines.RConn(1) ~= -1, delete_line(lines.RConn(1)); end
        delete_block(tr_blk);
        fprintf('      Removed static ground Transform block.\n');
    end

    fprintf('[2/6] Configuring MobileBaseJoint & Chassis Contact Alignment...\n');

    % Planar Joint (MobileBaseJoint) providing 3-DOF planar mobility (Px, Py, Rz)
    pj_name = [target_model '/MobileBaseJoint'];
    if getSimulinkBlockHandle(pj_name) == -1
        add_block('sm_lib/Joints/Planar Joint', pj_name, 'Position', [120, 180, 190, 260]);
    end
    set_param(pj_name, 'PxMotionActuationMode', 'InputMotion');
    set_param(pj_name, 'PyMotionActuationMode', 'InputMotion');
    set_param(pj_name, 'RzMotionActuationMode', 'InputMotion');
    set_param(pj_name, 'PxTorqueActuationMode', 'ComputedTorque');
    set_param(pj_name, 'PyTorqueActuationMode', 'ComputedTorque');
    set_param(pj_name, 'RzTorqueActuationMode', 'ComputedTorque');

    pj_ph = get_param(pj_name, 'PortHandles');
    w_ph  = get_param([target_model '/World'], 'PortHandles');

    % Rigid Transform for exact Wheel-to-Ground Contact & Chassis Orientation
    rt_mount_name = [target_model '/RT_Chassis_Mount'];
    if getSimulinkBlockHandle(rt_mount_name) == -1
        add_block('sm_lib/Frames and Transforms/Rigid Transform', rt_mount_name, ...
            'Position', [220, 200, 260, 240]);
    end
    set_param(rt_mount_name, 'TranslationMethod', 'Cartesian');
    set_param(rt_mount_name, 'TranslationCartesianOffset', '[-0.175 0 0.074]');
    set_param(rt_mount_name, 'TranslationCartesianOffsetUnits', 'm');
    set_param(rt_mount_name, 'RotationMethod', 'ArbitraryAxis');
    set_param(rt_mount_name, 'RotationArbitraryAxis', '[0 1 1]');
    set_param(rt_mount_name, 'RotationAngle', '180');
    set_param(rt_mount_name, 'RotationAngleUnits', 'deg');

    rt_mount_ph = get_param(rt_mount_name, 'PortHandles');
    chasis_ph   = get_param([target_model '/Chasis_1_RIGID'], 'PortHandles');

    % Connect World -> MobileBaseJoint Base (LConn1)
    add_line(target_model, w_ph.RConn(1), pj_ph.LConn(1), 'autorouting', 'smart');

    % Connect MobileBaseJoint Follower (RConn1) -> RT_Chassis_Mount (LConn1)
    add_line(target_model, pj_ph.RConn(1), rt_mount_ph.LConn(1), 'autorouting', 'smart');

    % Connect RT_Chassis_Mount (RConn1) -> Chasis_1_RIGID (LConn2)
    add_line(target_model, rt_mount_ph.RConn(1), chasis_ph.LConn(2), 'autorouting', 'smart');

    % Connect Trajectory Signal Inputs (X, Y, Yaw) with 2nd-order filtering
    axes_labels = {'X', 'Y', 'Yaw'};
    units = {'m', 'm', 'rad'};
    var_names = {'sim_traj_x', 'sim_traj_y', 'sim_traj_yaw'};

    for k = 1:3
        ps_name = [target_model '/PS_' axes_labels{k}];
        ws_name = [target_model '/FromWs_' axes_labels{k}];
        y_pos = 160 + (k-1) * 60;

        if getSimulinkBlockHandle(ps_name) == -1
            add_block('nesl_utility/Simulink-PS Converter', ps_name, ...
                'Position', [40, y_pos, 75, y_pos + 30]);
        end
        set_param(ps_name, 'Unit', units{k});
        set_param(ps_name, 'FilteringAndDerivatives', 'filter');
        set_param(ps_name, 'SimscapeFilterOrder', '2');
        set_param(ps_name, 'InputFilterTimeConstant', '0.02');

        if getSimulinkBlockHandle(ws_name) == -1
            add_block('simulink/Sources/From Workspace', ws_name, ...
                'Position', [-60, y_pos + 5, -5, y_pos + 25]);
        end
        set_param(ws_name, 'VariableName', var_names{k});
        set_param(ws_name, 'Interpolate', 'on');

        from_ph = get_param(ws_name, 'PortHandles');
        ps_ph   = get_param(ps_name, 'PortHandles');
        add_line(target_model, from_ph.Outport(1), ps_ph.Inport(1), 'autorouting', 'smart');
        add_line(target_model, ps_ph.RConn(1), pj_ph.LConn(1 + k), 'autorouting', 'smart');
    end

    % =========================================================================
    % 3D Dynamic Cargo Payload Assembly & Motion Actuation
    % =========================================================================
    fprintf('[2b/6] Adding Dynamic 3D Cargo Payload & Actuation...\n');
    cargo_pj = [target_model '/Cargo_MobileJoint'];
    if getSimulinkBlockHandle(cargo_pj) == -1
        add_block('sm_lib/Joints/Planar Joint', cargo_pj, 'Position', [120, 380, 190, 460]);
    end
    set_param(cargo_pj, 'PxMotionActuationMode', 'InputMotion');
    set_param(cargo_pj, 'PyMotionActuationMode', 'InputMotion');
    set_param(cargo_pj, 'RzMotionActuationMode', 'InputMotion');
    set_param(cargo_pj, 'PxTorqueActuationMode', 'ComputedTorque');
    set_param(cargo_pj, 'PyTorqueActuationMode', 'ComputedTorque');
    set_param(cargo_pj, 'RzTorqueActuationMode', 'ComputedTorque');

    cargo_pj_ph = get_param(cargo_pj, 'PortHandles');
    add_line(target_model, w_ph.RConn(1), cargo_pj_ph.LConn(1), 'autorouting', 'smart');

    % Elevation transform so the box rests precisely on the AGV chassis deck plate
    rt_cargo = [target_model '/RT_Cargo_Elevation'];
    if getSimulinkBlockHandle(rt_cargo) == -1
        add_block('sm_lib/Frames and Transforms/Rigid Transform', rt_cargo, ...
            'Position', [220, 400, 260, 440]);
    end
    set_param(rt_cargo, 'TranslationMethod', 'Cartesian');
    set_param(rt_cargo, 'TranslationCartesianOffset', '[0 0 0.22]');
    set_param(rt_cargo, 'TranslationCartesianOffsetUnits', 'm');

    rt_cargo_ph = get_param(rt_cargo, 'PortHandles');
    add_line(target_model, cargo_pj_ph.RConn(1), rt_cargo_ph.LConn(1), 'autorouting', 'smart');

    % 3D Solid Cargo Box Body (Safety Amber / Golden Cargo Container)
    box_solid = [target_model '/Cargo_Payload_Box'];
    if getSimulinkBlockHandle(box_solid) == -1
        add_block('sm_lib/Body Elements/Brick Solid', box_solid, ...
            'Position', [300, 400, 360, 440]);
    end
    set_param(box_solid, 'BrickDimensions', '[0.44 0.38 0.24]');
    set_param(box_solid, 'GraphicDiffuseColor', '[0.98 0.58 0.08]'); % Golden Amber
    set_param(box_solid, 'GraphicSpecularColor', '[0.80 0.80 0.80]');

    box_ph = get_param(box_solid, 'PortHandles');
    add_line(target_model, rt_cargo_ph.RConn(1), box_ph.RConn(1), 'autorouting', 'smart');

    % Cargo Motion Signals (sim_cargo_x, sim_cargo_y, sim_cargo_yaw)
    cargo_var_names = {'sim_cargo_x', 'sim_cargo_y', 'sim_cargo_yaw'};
    for k = 1:3
        ps_c_name = [target_model '/PS_Cargo_' axes_labels{k}];
        ws_c_name = [target_model '/FromWs_Cargo_' axes_labels{k}];
        y_pos = 360 + (k-1) * 60;

        if getSimulinkBlockHandle(ps_c_name) == -1
            add_block('nesl_utility/Simulink-PS Converter', ps_c_name, ...
                'Position', [40, y_pos, 75, y_pos + 30]);
        end
        set_param(ps_c_name, 'Unit', units{k});
        set_param(ps_c_name, 'FilteringAndDerivatives', 'filter');
        set_param(ps_c_name, 'SimscapeFilterOrder', '2');
        set_param(ps_c_name, 'InputFilterTimeConstant', '0.02');

        if getSimulinkBlockHandle(ws_c_name) == -1
            add_block('simulink/Sources/From Workspace', ws_c_name, ...
                'Position', [-60, y_pos + 5, -5, y_pos + 25]);
        end
        set_param(ws_c_name, 'VariableName', cargo_var_names{k});
        set_param(ws_c_name, 'Interpolate', 'on');

        from_c_ph = get_param(ws_c_name, 'PortHandles');
        ps_c_ph   = get_param(ps_c_name, 'PortHandles');
        add_line(target_model, from_c_ph.Outport(1), ps_c_ph.Inport(1), 'autorouting', 'smart');
        add_line(target_model, ps_c_ph.RConn(1), cargo_pj_ph.LConn(1 + k), 'autorouting', 'smart');
    end

    % =========================================================================
    % Add Telemetry Scopes (Payload Mass, Ground Force, Motor Torque, Power)
    % =========================================================================
    fprintf('[2c/6] Adding Simulink Scopes for Telemetry & Dynamics...\n');
    telem_vars = {
        'sim_payload_mass',   'Scope_Payload_Mass_kg',            560;
        'sim_normal_force',   'Scope_Ground_Reaction_Force_N',    620;
        'sim_motor_torque',   'Scope_Motor_Drive_Torque_Nm',      680;
        'sim_battery_power',  'Scope_Battery_Power_Watts',        740;
        'sim_battery_energy', 'Scope_Battery_Energy_kJ',          800;
        'sim_mission_phase',  'Scope_Mission_Phase_State',        860
    };

    for m = 1:size(telem_vars, 1)
        v_name = telem_vars{m, 1};
        s_name = [target_model '/' telem_vars{m, 2}];
        ws_t_name = [target_model '/FromWs_T_' v_name];
        y_pos = telem_vars{m, 3};

        if getSimulinkBlockHandle(ws_t_name) == -1
            add_block('simulink/Sources/From Workspace', ws_t_name, ...
                'Position', [-60, y_pos, 40, y_pos + 30]);
        end
        set_param(ws_t_name, 'VariableName', v_name);
        set_param(ws_t_name, 'Interpolate', 'on');

        if getSimulinkBlockHandle(s_name) == -1
            add_block('simulink/Sinks/Scope', s_name, ...
                'Position', [100, y_pos, 160, y_pos + 30]);
        end

        from_t_ph = get_param(ws_t_name, 'PortHandles');
        scope_ph  = get_param(s_name, 'PortHandles');
        add_line(target_model, from_t_ph.Outport(1), scope_ph.Inport(1), 'autorouting', 'smart');
    end

    fprintf('[3/6] Placing 3D Obstacles (New Layout), Walls, and Start/Target Markers...\n');

    % 3D Environment Definition:
    env_objects = {
        % 1. Industrial Ground Floor (25m x 25m, Top Surface at Z = 0.00m)
        'Env_Ground_Floor',           [12.5, 12.5, -0.05],  [25.0, 25.0, 0.1],  [0.72, 0.74, 0.76], 'brick';

        % 2. Perimeter Enclosure Walls (25m Boundary, 1.2m Height, Base at Z = 0.00m)
        'Env_Wall_South',             [12.5, -0.125, 0.60], [25.5, 0.25, 1.2],  [0.28, 0.32, 0.36], 'brick';
        'Env_Wall_North',             [12.5, 25.125, 0.60], [25.5, 0.25, 1.2],  [0.28, 0.32, 0.36], 'brick';
        'Env_Wall_West',              [-0.125, 12.5, 0.60], [0.25, 25.0, 1.2],  [0.28, 0.32, 0.36], 'brick';
        'Env_Wall_East',              [25.125, 12.5, 0.60], [0.25, 25.0, 1.2],  [0.28, 0.32, 0.36], 'brick';

        % 3. Origin Point: Rectangular Staging Dock Platform & Terminal Station
        'Marker_Start_Dock_Pad',      [3.0, 3.0, 0.010],        [2.0, 1.6, 0.02],   [0.15, 0.80, 0.35], 'brick';
        'Marker_Start_Dock_Border',   [3.0, 3.0, 0.005],        [2.3, 1.9, 0.01],   [0.18, 0.22, 0.28], 'brick';
        'Marker_Start_Terminal',      [3.0 - 0.85, 3.0, 0.65],  [0.25, 0.85, 1.30], [0.12, 0.55, 0.35], 'brick';
        'Marker_Start_LightBar',      [3.0 - 0.85, 3.0, 1.35],  [0.20, 0.75, 0.10], [0.20, 0.95, 0.40], 'brick';

        % 4. Dynamic Target Destination: Rectangular Docking Platform & Terminal Station
        'Marker_Goal_Dock_Pad',       [goalX, goalY, 0.010],        [2.0, 1.6, 0.02],   [0.98, 0.72, 0.10], 'brick';
        'Marker_Goal_Dock_Border',    [goalX, goalY, 0.005],        [2.3, 1.9, 0.01],   [0.18, 0.22, 0.28], 'brick';
        'Marker_Goal_Terminal',       [goalX - 0.85, goalY, 0.65],  [0.25, 0.85, 1.30], [0.15, 0.50, 0.80], 'brick';
        'Marker_Goal_LightBar',       [goalX - 0.85, goalY, 1.35],  [0.20, 0.75, 0.10], [1.00, 0.25, 0.10], 'brick';

        % 5. Inbound Pickup Bay [3.5, 12.5] (Dedicated Staging Pad for Cargo Missions)
        'Marker_Pickup_Dock_Pad',     [3.5, 12.5, 0.010],       [2.0, 1.6, 0.02],   [0.20, 0.75, 0.85], 'brick';
        'Marker_Pickup_Dock_Border',  [3.5, 12.5, 0.005],       [2.3, 1.9, 0.01],   [0.18, 0.22, 0.28], 'brick';
        'Marker_Pickup_Terminal',     [3.5 - 0.85, 12.5, 0.65], [0.25, 0.85, 1.30], [0.15, 0.45, 0.70], 'brick';

        % 6. Heavy-Duty Warehouse Shelving Racks (Longitudinal Aisle Configuration)
        'Obs_Rack_West_1',            [7.0, 7.5, 0.90],     [1.6, 5.0, 1.8],    [0.18, 0.35, 0.68], 'brick';
        'Obs_Rack_West_2',            [7.0, 16.5, 0.90],    [1.6, 5.0, 1.8],    [0.18, 0.35, 0.68], 'brick';
        'Obs_Rack_East_1',            [18.0, 7.5, 0.90],    [1.6, 5.0, 1.8],    [0.18, 0.35, 0.68], 'brick';
        'Obs_Rack_East_2',            [18.0, 16.5, 0.90],   [1.6, 5.0, 1.8],    [0.18, 0.35, 0.68], 'brick';

        % 7. Central Cargo Crate Stacks
        'Obs_Crate_Cluster_Mid',      [12.5, 12.0, 0.75],   [2.8, 2.8, 1.5],    [0.90, 0.55, 0.15], 'brick';
        'Obs_Crate_Stack_N',          [12.5, 18.0, 0.60],   [2.2, 2.2, 1.2],    [0.85, 0.50, 0.12], 'brick';

        % 8. Cylindrical Structural Concrete Columns
        'Obs_Pillar_NW',              [11.0, 6.0, 1.25],    [0.50, 2.5, 0.0],   [0.25, 0.65, 0.60], 'cylinder';
        'Obs_Pillar_SE',              [14.0, 6.0, 1.25],    [0.50, 2.5, 0.0],   [0.25, 0.65, 0.60], 'cylinder';

        % 9. Outbound Staging Pallet Stacks
        'Obs_Pallet_Staging_East',     [21.5, 12.5, 0.50],   [1.8, 2.2, 1.0],    [0.72, 0.46, 0.22], 'brick'
    };

    for i = 1:size(env_objects, 1)
        name = env_objects{i, 1};
        pos = env_objects{i, 2};
        dims = env_objects{i, 3};
        col = env_objects{i, 4};
        shape = env_objects{i, 5};

        rt_name = [target_model '/RT_' name];
        body_name = [target_model '/' name];

        row = mod(i-1, 10);
        col_offset = floor((i-1) / 10) * 150;

        if getSimulinkBlockHandle(rt_name) == -1
            add_block('sm_lib/Frames and Transforms/Rigid Transform', rt_name, ...
                'Position', [-600 - col_offset, 80 + row*55, -560 - col_offset, 120 + row*55]);
        end
        set_param(rt_name, 'TranslationMethod', 'Cartesian');
        set_param(rt_name, 'TranslationCartesianOffset', sprintf('[%.2f %.2f %.2f]', pos(1), pos(2), pos(3)));
        set_param(rt_name, 'TranslationCartesianOffsetUnits', 'm');

        if getSimulinkBlockHandle(body_name) == -1
            if strcmp(shape, 'cylinder')
                add_block('sm_lib/Body Elements/Cylindrical Solid', body_name, ...
                    'Position', [-520 - col_offset, 80 + row*55, -460 - col_offset, 120 + row*55]);
            else
                add_block('sm_lib/Body Elements/Brick Solid', body_name, ...
                    'Position', [-520 - col_offset, 80 + row*55, -460 - col_offset, 120 + row*55]);
            end
        end

        if strcmp(shape, 'cylinder')
            set_param(body_name, 'CylinderRadius', num2str(dims(1)));
            set_param(body_name, 'CylinderLength', num2str(dims(2)));
        else
            set_param(body_name, 'BrickDimensions', sprintf('[%.2f %.2f %.2f]', dims(1), dims(2), dims(3)));
        end
        set_param(body_name, 'GraphicDiffuseColor', sprintf('[%.2f %.2f %.2f]', col(1), col(2), col(3)));

        rt_ph = get_param(rt_name, 'PortHandles');
        body_ph = get_param(body_name, 'PortHandles');
        add_line(target_model, w_ph.RConn(1), rt_ph.LConn(1), 'autorouting', 'smart');
        add_line(target_model, rt_ph.RConn(1), body_ph.RConn(1), 'autorouting', 'smart');
    end

    % =========================================================================
    % Path Planning & Trajectory Generation
    % =========================================================================
    if is_cargo_mission
        fprintf('[4/6] Executing Multi-Stop 5-Phase Cargo Mission Planner...\n');
        mission = plan_cargo_mission();
        map = mission.map;

        sim_traj_x        = mission.sim_traj_x;
        sim_traj_y        = mission.sim_traj_y;
        sim_traj_yaw      = mission.sim_traj_yaw;
        sim_cargo_x       = mission.sim_cargo_x;
        sim_cargo_y       = mission.sim_cargo_y;
        sim_cargo_yaw     = mission.sim_cargo_yaw;
        sim_payload_mass  = mission.sim_payload_mass;
        sim_normal_force  = mission.sim_normal_force;
        sim_motor_torque  = mission.sim_motor_torque;
        sim_battery_power = mission.sim_battery_power;
        sim_battery_energy= mission.sim_battery_energy;
        sim_mission_phase = mission.sim_mission_phase;
        totalSimTime      = mission.total_time;

        w_selected = [mission.waypoints_leg1; mission.waypoints_leg2; mission.waypoints_leg3];
        current_target_point = mission.stations.delivery;

        assignin('base', 'mission_data', mission);
        assignin('base', 'env_occupancy_map', mission.map);
    else
        fprintf('[4/6] Planning Kinodynamic Path to Target [%.1f, %.1f] on Map...\n', goalX, goalY);

        % Build 2D Occupancy Grid representing the new obstacle layout
        map = binaryOccupancyMap(25, 25, 5);
        
        [Xs, Ys] = meshgrid(0:0.05:25, 0:0.05:1.5); setOccupancy(map, [Xs(:), Ys(:)], 1);
        [Xn, Yn] = meshgrid(0:0.05:25, 23.5:0.05:25); setOccupancy(map, [Xn(:), Yn(:)], 1);
        [Xw, Yw] = meshgrid(0:0.05:1.5, 0:0.05:25); setOccupancy(map, [Xw(:), Yw(:)], 1);
        [Xe, Ye] = meshgrid(23.5:0.05:25, 0:0.05:25); setOccupancy(map, [Xe(:), Ye(:)], 1);

        [Ox1, Oy1] = meshgrid(6.2:0.05:7.8, 5.0:0.05:10.0); setOccupancy(map, [Ox1(:), Oy1(:)], 1);
        [Ox2, Oy2] = meshgrid(6.2:0.05:7.8, 14.0:0.05:19.0); setOccupancy(map, [Ox2(:), Oy2(:)], 1);
        [Ox3, Oy3] = meshgrid(17.2:0.05:18.8, 5.0:0.05:10.0); setOccupancy(map, [Ox3(:), Oy3(:)], 1);
        [Ox4, Oy4] = meshgrid(17.2:0.05:18.8, 14.0:0.05:19.0); setOccupancy(map, [Ox4(:), Oy4(:)], 1);

        [Ox5, Oy5] = meshgrid(11.1:0.05:13.9, 10.6:0.05:13.4); setOccupancy(map, [Ox5(:), Oy5(:)], 1);
        [Ox6, Oy6] = meshgrid(11.4:0.05:13.6, 16.9:0.05:19.1); setOccupancy(map, [Ox6(:), Oy6(:)], 1);
        [Ox7, Oy7] = meshgrid(10.5:0.05:11.5, 5.5:0.05:6.5); setOccupancy(map, [Ox7(:), Oy7(:)], 1);
        [Ox8, Oy8] = meshgrid(13.5:0.05:14.5, 5.5:0.05:6.5); setOccupancy(map, [Ox8(:), Oy8(:)], 1);

        inflated = copy(map);
        inflate(inflated, 0.65);

        sv = validatorOccupancyMap;
        sv.Map = inflated;
        sv.ValidationDistance = 0.2;
        planner = plannerHybridAStar(sv, 'MinTurningRadius', 0.8, 'MotionPrimitiveLength', 0.8);

        startPose = [3.0, 3.0, 0];
        goalPose  = [goalX, goalY, goalYaw];

        midPose = [ (startPose(1) + goalX)/2, (startPose(2) + goalY)/2, atan2(goalY - startPose(2), goalX - startPose(1)) ];
        try
            pA = plan(planner, startPose, midPose);
            pB = plan(planner, [midPose(1), midPose(2), midPose(3)], goalPose);
            w_selected = [pA.States(:, 1:2); pB.States(:, 1:2)];
            len = pathLength(pA) + pathLength(pB);
        catch
            p_direct = plan(planner, startPose, goalPose);
            w_selected = p_direct.States(:, 1:2);
            len = pathLength(p_direct);
        end

        fprintf('      Collision-free path planned! Length: %.2f meters (%d waypoints)\n', len, size(w_selected, 1));

        pp = controllerPurePursuit('Waypoints', w_selected, ...
                                   'DesiredLinearVelocity', 0.90, ...
                                   'MaxAngularVelocity', 1.2, ...
                                   'LookaheadDistance', 0.90);

        cur = startPose;
        dt = 0.05;
        t_arr = 0;
        x_arr = cur(1);
        y_arr = cur(2);
        yaw_arr = cur(3);

        while (norm(cur(1:2) - goalPose(1:2)) > 0.40) && (t_arr(end) < 65.0)
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
        end

        totalSimTime = t_arr(end);
        sim_traj_x = [t_arr, x_arr];
        sim_traj_y = [t_arr, y_arr];
        sim_traj_yaw = [t_arr, yaw_arr];

        % In single target mode, cargo rides with vehicle
        sim_cargo_x = sim_traj_x;
        sim_cargo_y = sim_traj_y;
        sim_cargo_yaw = sim_traj_yaw;

        m_cargo = 20.0;
        sim_payload_mass = [t_arr, repmat(m_cargo, length(t_arr), 1)];
        sim_normal_force = [t_arr, repmat((22.72 + m_cargo) * 9.81, length(t_arr), 1)];
        sim_motor_torque = [t_arr, repmat(1.85, length(t_arr), 1)];
        sim_battery_power = [t_arr, repmat(78.5, length(t_arr), 1)];
        sim_battery_energy = [t_arr, (78.5 * t_arr) / 1000];
        sim_mission_phase = [t_arr, repmat(3, length(t_arr), 1)];
        current_target_point = [goalX, goalY, goalYaw];
    end

    % Push signals to base workspace
    assignin('base', 'env_occupancy_map', map);
    assignin('base', 'sim_traj_x', sim_traj_x);
    assignin('base', 'sim_traj_y', sim_traj_y);
    assignin('base', 'sim_traj_yaw', sim_traj_yaw);
    assignin('base', 'sim_cargo_x', sim_cargo_x);
    assignin('base', 'sim_cargo_y', sim_cargo_y);
    assignin('base', 'sim_cargo_yaw', sim_cargo_yaw);
    assignin('base', 'sim_payload_mass', sim_payload_mass);
    assignin('base', 'sim_normal_force', sim_normal_force);
    assignin('base', 'sim_motor_torque', sim_motor_torque);
    assignin('base', 'sim_battery_power', sim_battery_power);
    assignin('base', 'sim_battery_energy', sim_battery_energy);
    assignin('base', 'sim_mission_phase', sim_mission_phase);
    assignin('base', 'active_path_waypoints', w_selected);
    assignin('base', 'current_target_point', current_target_point);

    % Save persistent trajectory fallback file
    save('default_trajectory.mat', ...
         'sim_traj_x', 'sim_traj_y', 'sim_traj_yaw', ...
         'sim_cargo_x', 'sim_cargo_y', 'sim_cargo_yaw', ...
         'sim_payload_mass', 'sim_normal_force', 'sim_motor_torque', ...
         'sim_battery_power', 'sim_battery_energy', 'sim_mission_phase', ...
         'w_selected', 'current_target_point');

    % Auto-load callbacks for zero-error execution directly from Simulink Editor
    cb = sprintf([...
        'if ~evalin(''base'', ''exist(''''sim_traj_x'''', ''''var'''')'')\n' ...
        '    if exist(''default_trajectory.mat'', ''file'')\n' ...
        '        evalin(''base'', ''load(''''default_trajectory.mat'''')'');\n' ...
        '    end\n' ...
        'end\n']);
    set_param(target_model, 'PreLoadFcn', cb);
    set_param(target_model, 'InitFcn', cb);

    set_param(target_model, 'StopTime', num2str(totalSimTime));
    set_param(target_model, 'Solver', 'daessc');
    save_system(target_model, [], 'OverwriteIfChangedOnDisk', true);

    fprintf('[6/6] Model updated and saved as %s.slx.\n', target_model);
    fprintf('      Dynamic 3D Cargo Payload (20kg Amber Crate) successfully configured!\n');
    fprintf('      Simulink Scopes connected for Live Force, Torque, and Energy telemetry.\n');
    fprintf('=================================================================\n');
end
