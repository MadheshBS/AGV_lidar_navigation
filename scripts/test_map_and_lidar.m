%% test_map_and_lidar.m - Validation Test for Warehouse Map & Virtual Lidar
%
% PURPOSE:
%   Tests Phase 5.3 (Warehouse Binary Occupancy Map & Mission Waypoints)
%   and Phase 5.4 (2D Virtual Lidar Simulation with Unmapped Obstacles).
%
% VIVA EXPLANATION:
%   Industrial AGVs rely on a pre-surveyed static map (knownMap) for global
%   path planning and particle filter localization. However, the physical
%   environment (trueWorldMap) contains dynamic or unexpected obstacles
%   (such as fallen pallets or parked carts).
%   The virtual lidar simulates real-time laser range scans against the true
%   world, enabling both map-based localization and real-time obstacle avoidance.
%
% OUTPUTS:
%   Generates and exports 'warehouse_map_and_lidar.png'.

clear; clc; close all;
rng(42); % Fix seed for reproducibility

%% 1. Load Parameters & Generate Maps
p = agv_params();
fprintf('=====================================================\n');
fprintf('       WAREHOUSE MAP & VIRTUAL LIDAR TEST            \n');
fprintf('=====================================================\n');
fprintf('Map Dimensions:          %.1f m x %.1f m\n', p.map.width, p.map.height);
fprintf('Grid Resolution:         %.2f m/cell (%.0f cells/m)\n', 1/p.map.resolution, p.map.resolution);
fprintf('Lidar FOV / Beams:       %.0f deg / %d beams\n', rad2deg(p.lidar.fov), p.lidar.numBeams);
fprintf('Lidar Max Range:         %.1f m\n', p.lidar.maxRange);
fprintf('Lidar Noise (std dev):   %.3f m\n', p.lidar.noiseStd);
fprintf('Lidar Mount Offset:      [dx=%.2f m, dy=%.2f m]\n', p.lidar.mountPose(1), p.lidar.mountPose(2));
fprintf('-----------------------------------------------------\n');

[knownMap, trueWorldMap, waypoints] = build_warehouse_map(false);

%% 2. Position AGV in Aisle Near an Unexpected Obstacle
% Test pose located in Aisle 2 looking towards the unmapped pallet (Obstacle A)
agvPose = [7.0, 5.2, deg2rad(65)]; % [x=7.0 m, y=5.2 m, heading=65 deg]

fprintf('Simulating Virtual Lidar at AGV Pose:\n');
fprintf('  X = %.2f m, Y = %.2f m, Heading = %.1f deg\n', ...
    agvPose(1), agvPose(2), rad2deg(agvPose(3)));

% Execute virtual lidar scan against true world map
[ranges, angles, scanPtsGlobal, sensorPose] = virtual_lidar(agvPose, trueWorldMap, p);

validHits = ~isnan(ranges);
numHits   = sum(validHits);
minDist   = min(ranges(validHits));
maxDist   = max(ranges(validHits));

fprintf('Lidar Scan Statistics:\n');
fprintf('  Total Beams:           %d\n', length(ranges));
fprintf('  Valid Obstacle Hits:   %d (%.1f%%)\n', numHits, (numHits/length(ranges))*100);
fprintf('  Minimum Sensed Range:  %.2f m\n', minDist);
fprintf('  Maximum Sensed Range:  %.2f m\n', maxDist);
fprintf('=====================================================\n');

%% 3. Visualization & Publication-Grade Figure
fig = figure('Name', 'Warehouse Map & Virtual Lidar Scan', 'Units', 'pixels', ...
             'Position', [50, 50, 1350, 600], 'Color', 'w');

%% Panel 1: Known Map (for Planning & AMCL Localization)
subplot(1, 2, 1);
show(knownMap);
colormap(gca, flipud(gray)); % White = Free aisle space, Black = Solid shelves & walls
hold on; grid on; box on;
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.8 0.8 0.8], 'GridAlpha', 0.6);

% Draw Mission Waypoints
plot(waypoints.pickup(1), waypoints.pickup(2), 'o', 'MarkerSize', 11, ...
     'MarkerFaceColor', [0.1 0.75 0.1], 'MarkerEdgeColor', 'k', 'DisplayName', 'Pickup Bay');
text(waypoints.pickup(1)+0.4, waypoints.pickup(2)-0.3, 'Pickup [2.5, 2.0]', ...
     'FontSize', 9, 'FontWeight', 'bold', 'Color', [0 0.5 0]);

plot(waypoints.delivery(1), waypoints.delivery(2), 's', 'MarkerSize', 11, ...
     'MarkerFaceColor', [0.1 0.4 0.9], 'MarkerEdgeColor', 'k', 'DisplayName', 'Delivery Bay');
text(waypoints.delivery(1)+0.4, waypoints.delivery(2)+0.3, 'Delivery [12.8, 10.5]', ...
     'FontSize', 9, 'FontWeight', 'bold', 'Color', [0 0.2 0.8]);

plot(waypoints.return(1), waypoints.return(2), '^', 'MarkerSize', 11, ...
     'MarkerFaceColor', [0.9 0.1 0.1], 'MarkerEdgeColor', 'k', 'DisplayName', 'Return Depot');
text(waypoints.return(1)+0.4, waypoints.return(2)+0.3, 'Return [2.5, 13.0]', ...
     'FontSize', 9, 'FontWeight', 'bold', 'Color', [0.8 0 0]);

% Shelf Labels
text(5.25, 4.75, 'Shelf 1', 'FontSize', 8, 'Color', 'w', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
text(10.25, 4.75, 'Shelf 2', 'FontSize', 8, 'Color', 'w', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
text(15.25, 4.75, 'Shelf 3', 'FontSize', 8, 'Color', 'w', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
text(5.25, 10.25, 'Shelf 1', 'FontSize', 8, 'Color', 'w', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
text(10.25, 10.25, 'Shelf 2', 'FontSize', 8, 'Color', 'w', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');
text(15.25, 10.25, 'Shelf 3', 'FontSize', 8, 'Color', 'w', 'HorizontalAlignment', 'center', 'FontWeight', 'bold');

title({'1. Known Static Warehouse Map'; '(Used for Global A* Planning & AMCL Localization)'}, ...
      'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
xlabel('X [meters]', 'FontWeight', 'bold', 'Color', 'k');
ylabel('Y [meters]', 'FontWeight', 'bold', 'Color', 'k');
xlim([0, p.map.width]);
ylim([0, p.map.height]);
legend('Location', 'southeast', 'FontSize', 9);

%% Panel 2: True World Map with Lidar Scan Overlay
subplot(1, 2, 2);
show(trueWorldMap);
colormap(gca, flipud(gray));
hold on; grid on; box on;
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.8 0.8 0.8], 'GridAlpha', 0.6);

% Highlight unexpected obstacles with prominent red outlines
rectangle('Position', [7.3, 7.2, 0.8, 0.8], 'EdgeColor', [0.85 0 0], 'LineWidth', 2.2, 'LineStyle', '-');
text(7.3, 8.4, 'Pallet A (Unmapped)', 'Color', [0.85 0 0], 'FontSize', 8, 'FontWeight', 'bold');

rectangle('Position', [12.8, 5.8, 0.7, 0.7], 'EdgeColor', [0.85 0 0], 'LineWidth', 2.2, 'LineStyle', '-');
text(12.8, 5.2, 'Crate B (Unmapped)', 'Color', [0.85 0 0], 'FontSize', 8, 'FontWeight', 'bold');

rectangle('Position', [17.2, 7.4, 0.6, 0.9], 'EdgeColor', [0.85 0 0], 'LineWidth', 2.2, 'LineStyle', '-');
text(17.2, 8.7, 'Cart C (Unmapped)', 'Color', [0.85 0 0], 'FontSize', 8, 'FontWeight', 'bold');

% Draw Lidar Beams (subsample 1 out of 3 rays for clear ray visualization)
rayIdx = 1:3:p.lidar.numBeams;
for i = rayIdx
    if validHits(i)
        plot([sensorPose(1), scanPtsGlobal(i, 1)], [sensorPose(2), scanPtsGlobal(i, 2)], ...
             '-', 'Color', [0.2 0.75 0.9 0.35], 'LineWidth', 0.9, 'HandleVisibility', 'off');
    end
end

% Plot Lidar Laser Hit Returns (bright blue-green dots)
plot(scanPtsGlobal(validHits, 1), scanPtsGlobal(validHits, 2), '.', ...
     'Color', [0 0.55 0.9], 'MarkerSize', 8, 'DisplayName', 'Lidar Hits on Obstacles');

% Draw AGV Footprint & Orientation
robotR = p.robotRadius;
viscircles([agvPose(1), agvPose(2)], robotR, 'Color', [0.1 0.4 0.8], 'LineWidth', 2.0);
plot(agvPose(1), agvPose(2), 'bo', 'MarkerFaceColor', [0.1 0.3 0.8], 'MarkerSize', 7, 'DisplayName', 'AGV Center');
quiver(agvPose(1), agvPose(2), 0.85*cos(agvPose(3)), 0.85*sin(agvPose(3)), 0, ...
       'Color', [0 0.2 0.7], 'LineWidth', 2.5, 'MaxHeadSize', 0.8, 'DisplayName', 'AGV Heading');

% Plot Lidar Mount Location (in front of AGV chassis)
plot(sensorPose(1), sensorPose(2), '^', 'MarkerFaceColor', [0.9 0.4 0], 'MarkerEdgeColor', 'k', ...
     'MarkerSize', 8, 'DisplayName', 'Lidar Sensor Mount');

title({'2. True World Map with Virtual Lidar Scan Overlay'; ...
       sprintf('(360 Rays, Noise \\sigma = %.2fm, Sensed Hits = %d, Min Dist = %.2fm)', ...
       p.lidar.noiseStd, numHits, minDist)}, ...
       'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
xlabel('X [meters]', 'FontWeight', 'bold', 'Color', 'k');
ylabel('Y [meters]', 'FontWeight', 'bold', 'Color', 'k');
xlim([0, p.map.width]);
ylim([0, p.map.height]);
legend('Location', 'southeast', 'FontSize', 9);

% Save high-res plot
plotFileName = 'warehouse_map_and_lidar.png';
exportgraphics(fig, plotFileName, 'Resolution', 150);
fprintf('Plot successfully saved to: %s\n', plotFileName);
