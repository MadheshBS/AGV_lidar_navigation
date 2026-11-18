function [knownMap, trueWorldMap, waypoints] = build_warehouse_map(showPlot)
% BUILD_WAREHOUSE_MAP Creates binary occupancy grid maps and mission waypoints.
%
% Deliverable for Phase 5.3:
%   Generates a 20 m x 15 m warehouse map at 0.05 m resolution (20 cells/m)
%   consisting of outer perimeter walls, 3 double-shelf storage racks forming
%   4 vertical aisles and 3 cross-aisles, and named mission waypoints.
%
% Outputs:
%   knownMap     - binaryOccupancyMap of static environment (walls + shelves).
%                  Used for global path planning (A*) and AMCL localization.
%   trueWorldMap - binaryOccupancyMap containing knownMap PLUS unexpected obstacles.
%                  Used for virtual lidar scanning to test obstacle avoidance.
%   waypoints    - Struct containing named poses [x, y, theta]:
%                  .pickup   - Inbound loading dock
%                  .delivery - Target storage shelf bay
%                  .return   - Outbound dispatch / recharge station
%
% Usage:
%   [knownMap, trueWorldMap, waypoints] = build_warehouse_map(true);

    if nargin < 1
        showPlot = false;
    end

    p = agv_params();
    mapWidth  = p.map.width;       % 20.0 m
    mapHeight = p.map.height;      % 15.0 m
    res       = p.map.resolution;  % 20 cells/m (0.05 m per cell)

    %% 1. Initialize Empty Binary Occupancy Maps
    knownMap     = binaryOccupancyMap(mapWidth, mapHeight, res);
    trueWorldMap = binaryOccupancyMap(mapWidth, mapHeight, res);

    %% 2. Add Perimeter Boundary Walls (Thickness: 0.3 m)
    wallThick = 0.30;
    % Bottom wall
    add_box(knownMap, 0, 0, mapWidth, wallThick);
    % Top wall
    add_box(knownMap, 0, mapHeight - wallThick, mapWidth, wallThick);
    % Left wall
    add_box(knownMap, 0, 0, wallThick, mapHeight);
    % Right wall
    add_box(knownMap, mapWidth - wallThick, 0, wallThick, mapHeight);

    %% 3. Add 3 Storage Shelf Blocks (Racks)
    % Layout creates 4 longitudinal aisles and 3 transverse cross-aisles:
    % - Bottom cross-aisle: y in [0.3, 3.0]
    % - Central cross-aisle: y in [6.5, 8.5]
    % - Top cross-aisle: y in [12.0, 14.7]
    rackWidth  = 1.5;   % Width of shelf block (m)
    rackH_low  = 3.5;   % Height of lower shelf segment (m) [y: 3.0 -> 6.5]
    rackH_high = 3.5;   % Height of upper shelf segment (m) [y: 8.5 -> 12.0]

    % Shelf 1 (x: 4.5 to 6.0)
    add_box(knownMap, 4.5, 3.0, rackWidth, rackH_low);
    add_box(knownMap, 4.5, 8.5, rackWidth, rackH_high);

    % Shelf 2 (x: 9.5 to 11.0)
    add_box(knownMap, 9.5, 3.0, rackWidth, rackH_low);
    add_box(knownMap, 9.5, 8.5, rackWidth, rackH_high);

    % Shelf 3 (x: 14.5 to 16.0)
    add_box(knownMap, 14.5, 3.0, rackWidth, rackH_low);
    add_box(knownMap, 14.5, 8.5, rackWidth, rackH_high);

    %% 4. Copy Known Static Map into True World Map
    % The true world contains everything on the known map plus unexpected obstacles
    gridMatrix = getOccupancy(knownMap);
    setOccupancy(trueWorldMap, gridMatrix);

    %% 5. Add Unexpected Obstacles into True World Map (Phase 5.4 & 5.6)
    % These obstacles are NOT on knownMap. They test whether:
    % 1) Lidar detects unmapped obstacles
    % 2) MCL localization remains robust despite unmapped sensor returns
    % 3) VFH local controller successfully steers around obstacles
    
    % Obstacle A: Pallet dropped in Aisle 2 transit path (blocks nominal A* route)
    add_box(trueWorldMap, 8.4, 5.4, 0.7, 0.7);

    % Obstacle B: Crate sitting in Aisle 3 (directly in front of delivery shelf)
    add_box(trueWorldMap, 12.8, 5.8, 0.7, 0.7);

    % Obstacle C: Work cart parked in Aisle 4
    add_box(trueWorldMap, 17.2, 7.4, 0.6, 0.9);

    %% 6. Define Standard Mission Waypoints [x, y, theta]
    % Coordinates chosen within safe clearances of aisles
    waypoints.pickup   = [2.5, 2.0, pi/2];    % Inbound area (facing North)
    waypoints.delivery = [12.8, 10.5, 0.0];   % Shelf storage bay in Aisle 3 (facing East)
    waypoints.return   = [2.5, 13.0, -pi/2];  % Outbound depot (facing South)

    %% 7. Optional Plotting
    if showPlot
        figure('Name', 'Warehouse Layout', 'Color', 'w', 'Position', [150, 150, 950, 650]);
        show(trueWorldMap);
        hold on; grid on; box on;
        set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k');

        % Plot Waypoints
        plot(waypoints.pickup(1), waypoints.pickup(2), 'go', 'MarkerSize', 12, ...
             'MarkerFaceColor', [0.2 0.8 0.2], 'MarkerEdgeColor', 'k', 'DisplayName', 'Pickup');
        text(waypoints.pickup(1)+0.3, waypoints.pickup(2), 'Pickup [2.5, 2.0]', ...
             'FontWeight', 'bold', 'FontSize', 10, 'Color', [0 0.5 0]);

        plot(waypoints.delivery(1), waypoints.delivery(2), 'bs', 'MarkerSize', 12, ...
             'MarkerFaceColor', [0.2 0.4 0.9], 'MarkerEdgeColor', 'k', 'DisplayName', 'Delivery');
        text(waypoints.delivery(1)+0.3, waypoints.delivery(2), 'Delivery [12.8, 10.5]', ...
             'FontWeight', 'bold', 'FontSize', 10, 'Color', [0 0 0.8]);

        plot(waypoints.return(1), waypoints.return(2), 'r^', 'MarkerSize', 12, ...
             'MarkerFaceColor', [0.9 0.2 0.2], 'MarkerEdgeColor', 'k', 'DisplayName', 'Return');
        text(waypoints.return(1)+0.3, waypoints.return(2), 'Return [2.5, 13.0]', ...
             'FontWeight', 'bold', 'FontSize', 10, 'Color', [0.8 0 0]);

        % Annotate unexpected obstacles
        text(8.8, 5.8, 'Obstacle A', 'Color', 'r', 'FontSize', 8, 'FontWeight', 'bold');
        text(13.2, 6.2, 'Obstacle B', 'Color', 'r', 'FontSize', 8, 'FontWeight', 'bold');
        text(17.6, 7.8, 'Obstacle C', 'Color', 'r', 'FontSize', 8, 'FontWeight', 'bold');

        title('Industrial Warehouse Environment (20m x 15m, 0.05m res)', ...
              'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
        xlabel('X [m]', 'FontWeight', 'bold', 'Color', 'k');
        ylabel('Y [m]', 'FontWeight', 'bold', 'Color', 'k');
        legend('Location', 'northeast');
    end
end

%% Helper Function: Add Solid Rectangular Obstacle to Occupancy Map
function add_box(map, xMin, yMin, width, height)
    step = 0.03; % Finer than map resolution (0.05m) to ensure contiguous fill
    [X, Y] = meshgrid(xMin:step:(xMin + width), yMin:step:(yMin + height));
    setOccupancy(map, [X(:), Y(:)], 1);
end
