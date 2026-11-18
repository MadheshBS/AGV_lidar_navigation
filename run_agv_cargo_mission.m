%% run_agv_cargo_mission.m
% High-Level Autonomous 5-Phase Cargo Mission Launcher for Review 2
% Features:
%   - Multi-Stop Kinodynamic Path: Depot -> Pickup [3.5, 12.5] -> Delivery [21.5, 4.0] -> Depot
%   - 3D Dynamic Cargo Payload in Simscape Mechanics Explorer (Loading, Carrying, Unloading)
%   - Live Telemetry: Payload Mass, Ground Reaction Force, Motor Torque, Battery Energy
%   - Auto-opens Mechanics Explorer 3D animation & Simulink Scopes
%
% USAGE:
%   run_agv_cargo_mission           % Runs full mission, plots telemetry, and opens 3D simulation

function run_agv_cargo_mission()
    fprintf('=================================================================\n');
    fprintf('   AGV AUTONOMOUS 5-PHASE CARGO HANDLING MISSION (REVIEW 2)      \n');
    fprintf('   Pickup [3.5, 12.5] -> Delivery [21.5, 4.0] -> Home Depot     \n');
    fprintf('=================================================================\n');

    % 1. Build 3D Simscape Multibody model configured for Cargo Mission
    build_agv_3d_simulation('cargo_mission');

    m = evalin('base', 'mission_data');
    t = m.sim_traj_x(:, 1);
    x = m.sim_traj_x(:, 2);
    y = m.sim_traj_y(:, 2);
    mass = m.sim_payload_mass(:, 2);
    force = m.sim_normal_force(:, 2);
    torque = m.sim_motor_torque(:, 2);
    power = m.sim_battery_power(:, 2);
    energy = m.sim_battery_energy(:, 2);
    phase = m.sim_mission_phase(:, 2);

    % 2. Generate Publication-Quality Verification Plot
    fig = figure('Name', 'AGV Review 2 - Dynamic 3D Cargo Payload Telemetry', ...
                 'NumberTitle', 'off', 'Position', [60, 50, 1200, 800], ...
                 'Color', [1.0, 1.0, 1.0], 'Visible', 'off');

    % Subplot 1: Multi-Stop Warehouse Arena Trajectory
    ax1 = subplot(2, 2, 1);
    hold(ax1, 'on'); grid(ax1, 'on'); box(ax1, 'on');
    map = evalin('base', 'env_occupancy_map');
    show(map, 'Parent', ax1);
    
    % Color-code legs: Empty vs Loaded
    idx_p1 = find(phase == 1);
    idx_p3 = find(phase == 3);
    idx_p5 = find(phase == 5);

    plot(ax1, x(idx_p1), y(idx_p1), '-', 'Color', [0.15, 0.45, 0.85], 'LineWidth', 2.8, ...
         'DisplayName', 'Phase 1: Transit to Pickup (Empty, 0kg)');
    plot(ax1, x(idx_p3), y(idx_p3), '-', 'Color', [0.95, 0.50, 0.05], 'LineWidth', 3.5, ...
         'DisplayName', 'Phase 3: Loaded Transit (20kg Crate)');
    plot(ax1, x(idx_p5), y(idx_p5), '--', 'Color', [0.10, 0.70, 0.35], 'LineWidth', 2.6, ...
         'DisplayName', 'Phase 5: Return to Depot (Empty, 0kg)');

    % Station Markers
    plot(ax1, 3.0, 3.0, 'kd', 'MarkerSize', 11, 'MarkerFaceColor', [0.2, 0.8, 0.4], ...
         'LineWidth', 1.8, 'DisplayName', 'Depot [3.0, 3.0]');
    plot(ax1, 3.5, 12.5, 'ks', 'MarkerSize', 12, 'MarkerFaceColor', [0.1, 0.7, 0.9], ...
         'LineWidth', 1.8, 'DisplayName', 'Pickup Bay [3.5, 12.5]');
    plot(ax1, 21.5, 4.0, 'kp', 'MarkerSize', 14, 'MarkerFaceColor', [1.0, 0.7, 0.1], ...
         'LineWidth', 1.8, 'DisplayName', 'Delivery Bay [21.5, 4.0]');

    title(ax1, '🏭 Autonomous Multi-Stop Cargo Mission Route', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel(ax1, 'Warehouse X (m)'); ylabel(ax1, 'Warehouse Y (m)');
    legend(ax1, 'Location', 'northeast', 'FontSize', 8);

    % Subplot 2: Dynamic Payload Mass & Total Ground Normal Reaction Force
    ax2 = subplot(2, 2, 2);
    yyaxis(ax2, 'left');
    plot(ax2, t, mass, '-', 'Color', [0.90, 0.45, 0.05], 'LineWidth', 2.2);
    ylabel(ax2, 'Payload Mass (kg)', 'FontWeight', 'bold');
    ylim(ax2, [-2, 25]);
    ax2.YColor = [0.85, 0.40, 0.05];

    yyaxis(ax2, 'right');
    plot(ax2, t, force, '-', 'Color', [0.15, 0.35, 0.75], 'LineWidth', 2.2);
    ylabel(ax2, 'Total Ground Normal Force (N)', 'FontWeight', 'bold');
    ylim(ax2, [150, 460]);
    ax2.YColor = [0.15, 0.35, 0.75];

    grid(ax2, 'on'); box(ax2, 'on');
    title(ax2, '⚖️ Payload Mass & Wheel Contact Force (180N -> 419N)', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel(ax2, 'Simulation Time (s)');
    xline(ax2, 11.5, '--k', 'Dock 1 (Load)');
    xline(ax2, 41.2, '--k', 'Dock 2 (Unload)');

    % Subplot 3: Motor Drive Torque Demand
    ax3 = subplot(2, 2, 3);
    plot(ax3, t, torque, '-', 'Color', [0.80, 0.15, 0.25], 'LineWidth', 2.0);
    grid(ax3, 'on'); box(ax3, 'on');
    title(ax3, '⚙️ Dynamic Motor Torque Demand (Unloaded vs Loaded)', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel(ax3, 'Simulation Time (s)'); ylabel(ax3, 'Motor Torque (N·m)');
    xline(ax3, 11.5, '--k', 'Pickup');
    xline(ax3, 41.2, '--k', 'Delivery');
    ylim(ax3, [0, max(torque)*1.25]);

    % Subplot 4: Battery Instantaneous Power & Cumulative Energy Consumption
    ax4 = subplot(2, 2, 4);
    yyaxis(ax4, 'left');
    plot(ax4, t, power, '-', 'Color', [0.10, 0.65, 0.45], 'LineWidth', 2.0);
    ylabel(ax4, 'Instantaneous Power (Watts)', 'FontWeight', 'bold');
    ax4.YColor = [0.10, 0.65, 0.45];

    yyaxis(ax4, 'right');
    plot(ax4, t, energy, '-', 'Color', [0.55, 0.20, 0.75], 'LineWidth', 2.2);
    ylabel(ax4, 'Cumulative Energy (kJ)', 'FontWeight', 'bold');
    ax4.YColor = [0.55, 0.20, 0.75];

    grid(ax4, 'on'); box(ax4, 'on');
    title(ax4, '🔋 Battery Power & Cumulative Energy Drain (kJ)', 'FontSize', 11, 'FontWeight', 'bold');
    xlabel(ax4, 'Simulation Time (s)');
    xline(ax4, 11.5, '--k', 'Pickup');
    xline(ax4, 41.2, '--k', 'Delivery');

    saveas(fig, 'agv_cargo_payload_telemetry.png');
    close(fig);
    fprintf('      Saved verification plot to agv_cargo_payload_telemetry.png\n');

    % 3. Open Model & Scopes if running with desktop GUI
    if usejava('desktop')
        fprintf('      Launching Simscape 3D Mechanics Explorer Animation...\n');
        open_system('agv_3d_nav');
        open_system('agv_3d_nav/Scope_Payload_Mass_kg');
        open_system('agv_3d_nav/Scope_Ground_Reaction_Force_N');
        open_system('agv_3d_nav/Scope_Motor_Drive_Torque_Nm');
        open_system('agv_3d_nav/Scope_Battery_Energy_kJ');
    end

    sim('agv_3d_nav');
    fprintf('=================================================================\n');
    fprintf(' 5-Phase Cargo Mission Simulation Complete!\n');
    fprintf(' In Mechanics Explorer: Observe 3D amber crate loading onto AGV,\n');
    fprintf(' transporting across warehouse, and unloading at Delivery Bay!\n');
    fprintf('=================================================================\n');
end
