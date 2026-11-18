%% run_agv_3d_nav.m
% Interactive 3D Simscape Navigation for Custom AGV Mobile Robot.
% Featuring a modernized Mission Control GUI, dynamic 3D cargo payload handling,
% custom docking-bay visual markings, and real-time Simscape Mechanics Explorer animation.
%
% USAGE:
%   run_agv_3d_nav                  % Opens interactive map (click anywhere to navigate)
%   run_agv_3d_nav(X, Y)            % Opens with initial target at (X, Y)
%   run_agv_3d_nav('cargo_mission') % Directly runs the 5-phase cargo transport mission

function run_agv_3d_nav(targetX, targetY, targetYaw)
    is_cargo_init = false;
    if nargin >= 1 && (ischar(targetX) || isstring(targetX)) && strcmpi(targetX, 'cargo_mission')
        is_cargo_init = true;
        targetX = 21.5; targetY = 4.0; targetYaw = -pi/2;
    else
        if nargin < 1 || isempty(targetX), targetX = 22.0; end
        if nargin < 2 || isempty(targetY), targetY = 22.0; end
        if nargin < 3 || isempty(targetYaw), targetYaw = pi/2; end
    end

    fprintf('=================================================================\n');
    fprintf(' AGV Mission Control - Interactive 3D Navigation Console          \n');
    fprintf(' Review 2 Feature: Dynamic 3D Cargo Payload Handling in Simscape  \n');
    fprintf(' Click anywhere on the map to dispatch to a new docking target!  \n');
    fprintf('=================================================================\n');

    % Step 1: Build model and generate trajectory
    if is_cargo_init
        build_agv_3d_simulation('cargo_mission');
    else
        build_agv_3d_simulation(targetX, targetY, targetYaw);
    end

    model_name = 'agv_3d_nav';
    if evalin('base', 'exist(''env_occupancy_map'', ''var'')')
        map = evalin('base', 'env_occupancy_map');
    else
        % Fallback: build 25x25m occupancy map directly
        map = binaryOccupancyMap(25, 25, 5);
        [Xs, Ys] = meshgrid(0:0.05:25, 0:0.05:1.5); setOccupancy(map, [Xs(:), Ys(:)], 1);
        [Xn, Yn] = meshgrid(0:0.05:25, 23.5:0.05:25); setOccupancy(map, [Xn(:), Yn(:)], 1);
        [Xw, Yw] = meshgrid(0:0.05:1.5, 0:0.05:25); setOccupancy(map, [Xw(:), Yw(:)], 1);
        [Xe, Ye] = meshgrid(23.5:0.05:25, 0:0.05:25); setOccupancy(map, [Xe(:), Ye(:)], 1);
        assignin('base', 'env_occupancy_map', map);
    end

    if evalin('base', 'exist(''active_path_waypoints'', ''var'')')
        w = evalin('base', 'active_path_waypoints');
    else
        w = [3.0, 3.0; 22.0, 22.0];
        assignin('base', 'active_path_waypoints', w);
    end

    % Step 2: Create Modern Mission Control GUI Window
    close(findobj('Name', 'AGV Mission Control - Autonomous 3D Navigation Console'));
    fig = figure('Name', 'AGV Mission Control - Autonomous 3D Navigation Console', ...
                 'NumberTitle', 'off', 'Position', [50, 40, 1100, 840], ...
                 'Color', [0.10, 0.12, 0.16]);

    % Main Navigation Axes
    ax = axes(fig, 'Position', [0.06, 0.10, 0.68, 0.82], ...
              'Color', [0.06, 0.07, 0.09], ...
              'XColor', [0.75, 0.80, 0.88], 'YColor', [0.75, 0.80, 0.88], ...
              'GridColor', [0.25, 0.32, 0.42], 'GridAlpha', 0.5);
    show(map, 'Parent', ax);
    hold(ax, 'on');
    grid(ax, 'on');
    title(ax, '🏭 AGV AUTONOMOUS FLEET DISPATCH - 25m x 25m WAREHOUSE ARENA', ...
          'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.25, 0.85, 0.95]);
    xlabel(ax, 'Arena X (meters)', 'FontSize', 10, 'Color', [0.85, 0.88, 0.92]);
    ylabel(ax, 'Arena Y (meters)', 'FontSize', 10, 'Color', [0.85, 0.88, 0.92]);

    % Plot active kinodynamic path (Electric Amber Line)
    plot(ax, w(:, 1), w(:, 2), '-', 'Color', [1.00, 0.52, 0.12], 'LineWidth', 3.2, ...
         'DisplayName', 'Kinodynamic Route', 'Tag', 'ActivePath');

    % Plot Origin Staging Bay Rectangle Layout [2.0m x 1.6m]
    startBoxX = [3.0-1.0, 3.0+1.0, 3.0+1.0, 3.0-1.0, 3.0-1.0];
    startBoxY = [3.0-0.8, 3.0-0.8, 3.0+0.8, 3.0+0.8, 3.0-0.8];
    plot(ax, startBoxX, startBoxY, '--', 'Color', [0.20, 0.85, 0.45], 'LineWidth', 2.0, ...
         'DisplayName', 'Origin Depot Zone', 'Tag', 'StartBox');

    % Plot Origin / Start Point (Emerald Diamond)
    plot(ax, 3.0, 3.0, 'd', 'MarkerSize', 12, 'MarkerFaceColor', [0.15, 0.85, 0.40], ...
         'MarkerEdgeColor', [1.0, 1.0, 1.0], 'LineWidth', 2.0, ...
         'DisplayName', 'Origin Depot [3.0, 3.0]', 'Tag', 'StartMarker');

    % Inbound Pickup Bay Station Marker [3.5, 12.5]
    plot(ax, 3.5, 12.5, 's', 'MarkerSize', 13, 'MarkerFaceColor', [0.10, 0.75, 0.90], ...
         'MarkerEdgeColor', [1.0, 1.0, 1.0], 'LineWidth', 2.0, ...
         'DisplayName', 'Pickup Bay [3.5, 12.5]', 'Tag', 'PickupMarker');

    % Plot Target Docking Bay Rectangle Layout [2.0m x 1.6m]
    boxX = [targetX-1.0, targetX+1.0, targetX+1.0, targetX-1.0, targetX-1.0];
    boxY = [targetY-0.8, targetY-0.8, targetY+0.8, targetY+0.8, targetY-0.8];
    plot(ax, boxX, boxY, '--', 'Color', [1.00, 0.80, 0.20], 'LineWidth', 2.0, ...
         'DisplayName', 'Docking Bay Zone', 'Tag', 'TargetBox');

    % Plot Target Terminal Center Marker (Golden Bullseye Square)
    plot(ax, targetX, targetY, 'p', 'MarkerSize', 14, ...
         'MarkerFaceColor', [1.00, 0.72, 0.10], 'MarkerEdgeColor', [0.95, 0.20, 0.10], ...
         'LineWidth', 2.2, 'DisplayName', sprintf('Target Bay [%.1f, %.1f]', targetX, targetY), ...
         'Tag', 'TargetMarker');

    % Custom Sleek Legend
    legend(ax, 'Location', 'northeast', 'FontSize', 8.5, ...
           'Color', [0.12, 0.15, 0.20], 'TextColor', [0.92, 0.95, 0.98], ...
           'EdgeColor', [0.30, 0.40, 0.52]);

    % Top Glassmorphic Status Banner
    statusText = text(ax, 1.5, 24.2, ...
        sprintf('  ⚡ READY: Click ANYWHERE on map, or launch the 5-Phase Cargo Mission!  '), ...
        'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.20, 0.95, 0.85], ...
        'BackgroundColor', [0.12, 0.18, 0.26], 'EdgeColor', [0.25, 0.65, 0.85], ...
        'Margin', 6, 'Tag', 'StatusBanner');

    % Right Control Panel Background Card
    uicontrol(fig, 'Style', 'text', 'String', '', ...
              'Position', [810, 35, 265, 770], ...
              'BackgroundColor', [0.14, 0.17, 0.22]);

    % Panel Header
    uicontrol(fig, 'Style', 'text', 'String', '🎛️ MISSION CONTROL (REVIEW 2)', ...
              'Position', [820, 770, 245, 25], 'FontSize', 11, 'FontWeight', 'bold', ...
              'ForegroundColor', [0.25, 0.85, 0.95], 'BackgroundColor', [0.14, 0.17, 0.22], ...
              'HorizontalAlignment', 'center');

    % REVIEW 2 FEATURE: 5-Phase Autonomous Cargo Mission Button
    uicontrol(fig, 'Style', 'pushbutton', ...
              'String', '📦 Run 5-Phase Cargo Mission (Load & Drop)', ...
              'Position', [825, 715, 235, 45], 'FontSize', 9.5, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.95, 0.50, 0.08], 'ForegroundColor', [1 1 1], ...
              'Callback', @(~,~) execute_cargo_gui(ax, statusText, model_name, findobj(fig, 'Tag', 'InfoBox')));

    % Interactive Target Selection Button
    uicontrol(fig, 'Style', 'pushbutton', ...
              'String', '📍 Select Single Destination on Map', ...
              'Position', [825, 665, 235, 40], 'FontSize', 9.5, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.12, 0.52, 0.72], 'ForegroundColor', [1 1 1], ...
              'Callback', @(~,~) click_pick_callback(ax, statusText, model_name));

    % 3D Simulation Execution / Replay Button
    uicontrol(fig, 'Style', 'pushbutton', ...
              'String', '▶ Replay Active 3D Simulation', ...
              'Position', [825, 620, 235, 38], 'FontSize', 9.5, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.18, 0.65, 0.38], 'ForegroundColor', [1 1 1], ...
              'Callback', @(~,~) replay_callback(model_name, statusText));

    % Quick Target Preset Header
    uicontrol(fig, 'Style', 'text', 'String', '⚡ Quick Preset Dispatch:', ...
              'Position', [825, 580, 235, 20], 'FontSize', 9.5, 'FontWeight', 'bold', ...
              'ForegroundColor', [1.00, 0.80, 0.25], 'BackgroundColor', [0.14, 0.17, 0.22], ...
              'HorizontalAlignment', 'left');

    % Preset Buttons
    uicontrol(fig, 'Style', 'pushbutton', 'String', '🏭 Zone 1: Far Corner [22, 22]', ...
              'Position', [825, 545, 235, 30], 'FontSize', 9, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.20, 0.24, 0.32], 'ForegroundColor', [0.90, 0.94, 0.98], ...
              'Callback', @(~,~) execute_target_nav(22.0, 22.0, ax, statusText, model_name));

    uicontrol(fig, 'Style', 'pushbutton', 'String', '📦 Zone 2: North Staging [12.5, 22]', ...
              'Position', [825, 510, 235, 30], 'FontSize', 9, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.20, 0.24, 0.32], 'ForegroundColor', [0.90, 0.94, 0.98], ...
              'Callback', @(~,~) execute_target_nav(12.5, 22.0, ax, statusText, model_name));

    uicontrol(fig, 'Style', 'pushbutton', 'String', '🚛 Zone 3: East Delivery Bay [21.5, 4]', ...
              'Position', [825, 475, 235, 30], 'FontSize', 9, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.20, 0.24, 0.32], 'ForegroundColor', [0.90, 0.94, 0.98], ...
              'Callback', @(~,~) execute_target_nav(21.5, 4.0, ax, statusText, model_name));

    uicontrol(fig, 'Style', 'pushbutton', 'String', '🏛️ Zone 4: Central Depot [12.5, 8]', ...
              'Position', [825, 440, 235, 30], 'FontSize', 9, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.20, 0.24, 0.32], 'ForegroundColor', [0.90, 0.94, 0.98], ...
              'Callback', @(~,~) execute_target_nav(12.5, 8.0, ax, statusText, model_name));

    uicontrol(fig, 'Style', 'pushbutton', 'String', '🛠️ Zone 5: Inbound Pickup [3.5, 12.5]', ...
              'Position', [825, 405, 235, 30], 'FontSize', 9, 'FontWeight', 'bold', ...
              'BackgroundColor', [0.20, 0.24, 0.32], 'ForegroundColor', [0.90, 0.94, 0.98], ...
              'Callback', @(~,~) execute_target_nav(3.5, 12.5, ax, statusText, model_name));

    % Telemetry / Status Box Card
    uicontrol(fig, 'Style', 'text', 'String', '📊 REVIEW 2 DYNAMIC TELEMETRY', ...
              'Position', [825, 365, 235, 20], 'FontSize', 9, 'FontWeight', 'bold', ...
              'ForegroundColor', [0.65, 0.75, 0.85], 'BackgroundColor', [0.14, 0.17, 0.22], ...
              'HorizontalAlignment', 'left');

    infoBox = uicontrol(fig, 'Style', 'text', ...
              'String', sprintf('🎯 Target: [%.1f m, %.1f m]\n📦 Payload: Dynamic Crate\n⚖️ Mass: 0 kg -> 20 kg\n⚡ Force: 180 N -> 419 N\n📊 Scopes: Active in Simulink\nStatus: System Ready', targetX, targetY), ...
              'Position', [825, 215, 235, 140], 'FontSize', 9, 'FontWeight', 'bold', ...
              'ForegroundColor', [0.20, 0.95, 0.70], 'BackgroundColor', [0.08, 0.10, 0.14], ...
              'Tag', 'InfoBox');

    % Direct Map Click Callback (Click anywhere on the arena axes!)
    set(fig, 'WindowButtonDownFcn', @(src, ~) map_window_click(src, ax, statusText, model_name, infoBox));

    drawnow;

    % Step 3: Run initial 3D simulation
    if is_cargo_init
        execute_cargo_gui(ax, statusText, model_name, infoBox);
    else
        execute_target_nav(targetX, targetY, ax, statusText, model_name, infoBox);
    end
end

% Callback for 5-Phase Cargo Mission
function execute_cargo_gui(ax, statusText, model_name, infoBox)
    fprintf('\n--> Executing Full 5-Phase Cargo Transport Mission (Pickup -> Deliver -> Return)...\n');

    if ~isempty(statusText) && isvalid(statusText)
        set(statusText, 'String', '  📦 RUNNING 5-PHASE CARGO MISSION: Loading [3.5, 12.5] -> Delivery [21.5, 4.0] -> Depot...  ', ...
            'Color', [1.0, 0.85, 0.20], 'BackgroundColor', [0.25, 0.15, 0.05], 'EdgeColor', [0.95, 0.65, 0.10]);
    end
    if ~isempty(infoBox) && isvalid(infoBox)
        set(infoBox, 'String', sprintf('🎯 Mission: 5-Phase Cargo Cycle\n📦 Payload: 20kg Amber Crate\n⚖️ Force: 180N -> 419N (+132%%)\n⚙️ Torque: Doubled under Load\n⚡ Status: ⏳ Simulating in 3D...'), ...
            'ForegroundColor', [1.0, 0.85, 0.20]);
    end
    drawnow;

    build_agv_3d_simulation('cargo_mission');

    % Update 2D Map with multi-stop trajectory
    m = evalin('base', 'mission_data');
    x = m.sim_traj_x(:, 2);
    y = m.sim_traj_y(:, 2);
    h_path = findobj(ax, 'Tag', 'ActivePath');
    if ~isempty(h_path), set(h_path, 'XData', x, 'YData', y); end

    h_target = findobj(ax, 'Tag', 'TargetMarker');
    if ~isempty(h_target)
        set(h_target, 'XData', 21.5, 'YData', 4.0, 'DisplayName', 'Delivery Bay [21.5, 4.0]');
    end

    h_box = findobj(ax, 'Tag', 'TargetBox');
    if ~isempty(h_box)
        boxX = [21.5-1.0, 21.5+1.0, 21.5+1.0, 21.5-1.0, 21.5-1.0];
        boxY = [4.0-0.8, 4.0-0.8, 4.0+0.8, 4.0+0.8, 4.0-0.8];
        set(h_box, 'XData', boxX, 'YData', boxY);
    end
    drawnow;

    if ~bdIsLoaded(model_name), load_system(model_name); end
    set_param(model_name, 'SimMechanicsOpenEditorOnUpdate', 'on');
    sim(model_name);

    if ~isempty(statusText) && isvalid(statusText)
        set(statusText, 'String', '  ✅ CARGO MISSION COMPLETED! 20kg Crate picked at [3.5, 12.5] and delivered to [21.5, 4.0]!  ', ...
            'Color', [0.20, 0.95, 0.70], 'BackgroundColor', [0.08, 0.20, 0.14], 'EdgeColor', [0.15, 0.70, 0.40]);
    end
    if ~isempty(infoBox) && isvalid(infoBox)
        set(infoBox, 'String', sprintf('🎯 Mission: COMPLETED!\n📦 Payload: Successfully Delivered\n⚖️ Cycle: 0kg -> 20kg -> 0kg\n🔋 Energy Drain: Recorded\n⚡ Status: ✅ DOCKED AT DEPOT'), ...
            'ForegroundColor', [0.20, 0.95, 0.70]);
    end
    drawnow;
end

% Callback when clicking anywhere in the figure window
function map_window_click(~, ax, statusText, model_name, infoBox)
    cp = get(ax, 'CurrentPoint');
    clickX = cp(1, 1);
    clickY = cp(1, 2);

    if clickX >= 1.5 && clickX <= 23.5 && clickY >= 1.5 && clickY <= 23.5
        execute_target_nav(clickX, clickY, ax, statusText, model_name, infoBox);
    end
end

% Callback for the "Select Destination on Map" button
function click_pick_callback(ax, statusText, model_name)
    set(statusText, 'String', '  🎯 Crosshair Active: Click ANY point on the map to set target docking bay...  ', ...
        'Color', [1.0, 0.9, 0.3], 'BackgroundColor', [0.25, 0.20, 0.05], 'EdgeColor', [0.9, 0.7, 0.2]);
    drawnow;
    
    [clickX, clickY, btn] = ginput(1);
    if ~isempty(clickX) && btn == 1
        if clickX >= 1.5 && clickX <= 23.5 && clickY >= 1.5 && clickY <= 23.5
            fig = ancestor(ax, 'figure');
            infoBox = findobj(fig, 'Tag', 'InfoBox');
            execute_target_nav(clickX, clickY, ax, statusText, model_name, infoBox);
        end
    end
end

% Execute navigation to target coordinates
function execute_target_nav(targetX, targetY, ax, statusText, model_name, infoBox)
    fprintf('\n--> Destination selected at [%.2f, %.2f]. Replanning route & updating 3D model...\n', targetX, targetY);

    if ~isempty(statusText) && isvalid(statusText)
        set(statusText, 'String', sprintf('  ⏳ Replanning route & updating 3D docking bay at [%.1f, %.1f]...  ', targetX, targetY), ...
            'Color', [1.0, 0.85, 0.30], 'BackgroundColor', [0.22, 0.18, 0.08], 'EdgeColor', [0.85, 0.65, 0.20]);
    end
    if nargin >= 6 && ~isempty(infoBox) && isvalid(infoBox)
        set(infoBox, 'String', sprintf('🎯 Target: [%.1f m, %.1f m]\n🧭 Planner: Re-solving...\n📡 Controller: Active\n⚡ Status: ⏳ Simulating in 3D...', targetX, targetY), ...
            'ForegroundColor', [1.0, 0.85, 0.30]);
    end
    drawnow;

    % 1. Rebuild model with 3D rectangular docking station at clicked coordinates
    build_agv_3d_simulation(targetX, targetY);

    % 2. Update 2D figure with new route, target marker, and docking zone box
    w = evalin('base', 'active_path_waypoints');
    h_path = findobj(ax, 'Tag', 'ActivePath');
    if ~isempty(h_path), set(h_path, 'XData', w(:, 1), 'YData', w(:, 2)); end

    h_target = findobj(ax, 'Tag', 'TargetMarker');
    if ~isempty(h_target)
        set(h_target, 'XData', targetX, 'YData', targetY, ...
            'DisplayName', sprintf('Target Bay [%.1f, %.1f]', targetX, targetY));
    end

    h_box = findobj(ax, 'Tag', 'TargetBox');
    if ~isempty(h_box)
        boxX = [targetX-1.0, targetX+1.0, targetX+1.0, targetX-1.0, targetX-1.0];
        boxY = [targetY-0.8, targetY-0.8, targetY+0.8, targetY+0.8, targetY-0.8];
        set(h_box, 'XData', boxX, 'YData', boxY);
    end
    drawnow;

    % 3. Run Simscape Multibody 3D simulation
    if ~bdIsLoaded(model_name), load_system(model_name); end
    set_param(model_name, 'SimMechanicsOpenEditorOnUpdate', 'on');
    simOut = sim(model_name);

    fprintf('--> 3D Simulation finished! AGV successfully docked at [%.2f, %.2f] in Mechanics Explorer.\n', targetX, targetY);

    if ~isempty(statusText) && isvalid(statusText)
        set(statusText, 'String', sprintf('  ✅ DOCKED AT [%.1f, %.1f]! Click anywhere on map to dispatch to a new bay.  ', targetX, targetY), ...
            'Color', [0.20, 0.95, 0.70], 'BackgroundColor', [0.08, 0.20, 0.14], 'EdgeColor', [0.15, 0.70, 0.40]);
    end
    if nargin >= 6 && ~isempty(infoBox) && isvalid(infoBox)
        set(infoBox, 'String', sprintf('🎯 Target: [%.1f m, %.1f m]\n🧭 Planner: Complete\n📡 Controller: Finished\n⚡ Status: ✅ DOCKED AT TARGET!', targetX, targetY), ...
            'ForegroundColor', [0.20, 0.95, 0.70]);
    end
    drawnow;
end

% Replay existing simulation
function replay_callback(model_name, statusText)
    set(statusText, 'String', '  ▶ Replaying 3D simulation in Mechanics Explorer...  ', ...
        'Color', [0.30, 0.85, 1.0], 'BackgroundColor', [0.08, 0.18, 0.26], 'EdgeColor', [0.20, 0.60, 0.85]);
    drawnow;
    if ~bdIsLoaded(model_name), load_system(model_name); end
    set_param(model_name, 'SimMechanicsOpenEditorOnUpdate', 'on');
    sim(model_name);
    set(statusText, 'String', '  ✅ Replay finished. Click anywhere on map to set a new target.  ', ...
        'Color', [0.20, 0.95, 0.70], 'BackgroundColor', [0.08, 0.20, 0.14], 'EdgeColor', [0.15, 0.70, 0.40]);
    drawnow;
end
