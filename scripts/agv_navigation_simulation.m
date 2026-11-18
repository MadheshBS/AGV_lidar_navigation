%% agv_navigation_simulation.m
% Standalone 2D Autonomous Navigation Simulation for Custom AGV Mobile Robot
% - Builds the new 25m x 25m warehouse environment with redesigned obstacles
% - Plans collision-free paths using Hybrid A*
% - Simulates Pure Pursuit path following with differential-drive kinematics
% - Live animation with vehicle footprint, heading vector, and real-time trajectory trail
%
% USAGE:
%   agv_navigation_simulation()
%   agv_navigation_simulation(goalX, goalY)

function agv_navigation_simulation(goalX, goalY)
    if nargin < 1 || isempty(goalX), goalX = 22.0; end
    if nargin < 2 || isempty(goalY), goalY = 22.0; end
    goalYaw = pi/2;

    close all;
    fprintf('=================================================================\n');
    fprintf(' Starting AGV Mobile Robot 2D Autonomous Navigation Simulation \n');
    fprintf('=================================================================\n');

    %% 1. Create 25m x 25m Warehouse Environment with Redesigned Obstacles
    mapResolution = 5; % 5 cells per meter (0.2m resolution)
    arenaWidth = 25;
    arenaHeight = 25;
    map = binaryOccupancyMap(arenaWidth, arenaHeight, mapResolution);

    % Solid outer boundary perimeter walls (1.5m perimeter cushion)
    [Xs, Ys] = meshgrid(0:0.05:25, 0:0.05:1.5); setOccupancy(map, [Xs(:), Ys(:)], 1);
    [Xn, Yn] = meshgrid(0:0.05:25, 23.5:0.05:25); setOccupancy(map, [Xn(:), Yn(:)], 1);
    [Xw, Yw] = meshgrid(0:0.05:1.5, 0:0.05:25); setOccupancy(map, [Xw(:), Yw(:)], 1);
    [Xe, Ye] = meshgrid(23.5:0.05:25, 0:0.05:25); setOccupancy(map, [Xe(:), Ye(:)], 1);

    % 1. West Racks (North-South longitudinal aisles)
    [Ox1, Oy1] = meshgrid(6.2:0.05:7.8, 5.0:0.05:10.0); setOccupancy(map, [Ox1(:), Oy1(:)], 1);
    [Ox2, Oy2] = meshgrid(6.2:0.05:7.8, 14.0:0.05:19.0); setOccupancy(map, [Ox2(:), Oy2(:)], 1);

    % 2. East Racks
    [Ox3, Oy3] = meshgrid(17.2:0.05:18.8, 5.0:0.05:10.0); setOccupancy(map, [Ox3(:), Oy3(:)], 1);
    [Ox4, Oy4] = meshgrid(17.2:0.05:18.8, 14.0:0.05:19.0); setOccupancy(map, [Ox4(:), Oy4(:)], 1);

    % 3. Central Cargo Crates
    [Ox5, Oy5] = meshgrid(11.1:0.05:13.9, 10.6:0.05:13.4); setOccupancy(map, [Ox5(:), Oy5(:)], 1);
    [Ox6, Oy6] = meshgrid(11.4:0.05:13.6, 16.9:0.05:19.1); setOccupancy(map, [Ox6(:), Oy6(:)], 1);

    % 4. Cylindrical Structural Columns
    [Ox7, Oy7] = meshgrid(10.5:0.05:11.5, 5.5:0.05:6.5); setOccupancy(map, [Ox7(:), Oy7(:)], 1);
    [Ox8, Oy8] = meshgrid(13.5:0.05:14.5, 5.5:0.05:6.5); setOccupancy(map, [Ox8(:), Oy8(:)], 1);

    % 5. Staging Pallet Stacks
    [Ox9, Oy9] = meshgrid(2.6:0.05:4.4, 11.4:0.05:13.6); setOccupancy(map, [Ox9(:), Oy9(:)], 1);
    [Ox10, Oy10] = meshgrid(20.6:0.05:22.4, 11.4:0.05:13.6); setOccupancy(map, [Ox10(:), Oy10(:)], 1);

    % Inflate map by robot safety radius
    robotRadius = 0.65;
    inflatedMap = copy(map);
    inflate(inflatedMap, robotRadius);

    %% 2. Define Start Pose and Target Goal Point
    startPose = [3.0, 3.0, 0];              % [x (m), y (m), theta (rad)]
    goalPose  = [goalX, goalY, goalYaw];     % Target Destination Point

    fprintf('Start Point:  [%.1f, %.1f, %.2f rad]\n', startPose(1), startPose(2), startPose(3));
    fprintf('Target Goal:  [%.1f, %.1f, %.2f rad]\n', goalPose(1), goalPose(2), goalPose(3));

    %% 3. Plan Path using State Validator and Hybrid A*
    stateValidator = validatorOccupancyMap;
    stateValidator.Map = inflatedMap;
    stateValidator.ValidationDistance = 0.2;

    planner = plannerHybridAStar(stateValidator, ...
        'MinTurningRadius', 0.8, ...
        'MotionPrimitiveLength', 0.8, ...
        'ForwardCost', 1.0, ...
        'ReverseCost', 3.0);

    fprintf('Planning collision-free path to target point...\n');
    midPose = [ (startPose(1) + goalX)/2, (startPose(2) + goalY)/2, atan2(goalY - startPose(2), goalX - startPose(1)) ];
    try
        pA = plan(planner, startPose, midPose);
        pB = plan(planner, [midPose(1), midPose(2), midPose(3)], goalPose);
        waypoints = [pA.States(:, 1:2); pB.States(:, 1:2)];
        pathLengthVal = pathLength(pA) + pathLength(pB);
    catch
        p_direct = plan(planner, startPose, goalPose);
        waypoints = p_direct.States(:, 1:2);
        pathLengthVal = pathLength(p_direct);
    end

    fprintf('Path generated successfully! Total path length: %.2f meters (%d waypoints)\n', ...
        pathLengthVal, size(waypoints, 1));

    %% 4. Configure Pure Pursuit Controller
    controller = controllerPurePursuit;
    controller.Waypoints = waypoints;
    controller.DesiredLinearVelocity = 0.90; % m/s
    controller.MaxAngularVelocity = 1.20;    % rad/s
    controller.LookaheadDistance = 0.90;     % m

    %% 5. Setup Live Visualization Window (Mission Control Dark Theme)
    fig = figure('Name', 'Custom AGV Mobile Robot Autonomous Navigation', ...
                 'NumberTitle', 'off', 'Position', [150, 80, 1000, 820], ...
                 'Color', [0.10, 0.12, 0.16]);

    ax = axes(fig, 'Color', [0.06, 0.07, 0.09], ...
              'XColor', [0.75, 0.80, 0.88], 'YColor', [0.75, 0.80, 0.88], ...
              'GridColor', [0.25, 0.32, 0.42], 'GridAlpha', 0.5);
    show(map, 'Parent', ax);
    hold(ax, 'on');
    grid(ax, 'on');
    title(ax, '🏭 AGV AUTONOMOUS FLEET DISPATCH - 25m x 25m WAREHOUSE ARENA', ...
          'FontSize', 12, 'FontWeight', 'bold', 'Color', [0.25, 0.85, 0.95]);
    xlabel(ax, 'Arena X (meters)', 'FontSize', 10, 'Color', [0.85, 0.88, 0.92]);
    ylabel(ax, 'Arena Y (meters)', 'FontSize', 10, 'Color', [0.85, 0.88, 0.92]);

    % Planned trajectory (Electric Amber)
    plot(ax, waypoints(:, 1), waypoints(:, 2), '-', 'Color', [1.00, 0.52, 0.12], ...
         'LineWidth', 3.2, 'DisplayName', 'Kinodynamic Route');

    % Origin Staging Bay Rectangle Layout [2.0m x 1.6m]
    startBoxX = [startPose(1)-1.0, startPose(1)+1.0, startPose(1)+1.0, startPose(1)-1.0, startPose(1)-1.0];
    startBoxY = [startPose(2)-0.8, startPose(2)-0.8, startPose(2)+0.8, startPose(2)+0.8, startPose(2)-0.8];
    plot(ax, startBoxX, startBoxY, '--', 'Color', [0.20, 0.85, 0.45], 'LineWidth', 2.0, ...
         'DisplayName', 'Origin Depot Zone');

    % Origin point (Emerald Diamond)
    plot(ax, startPose(1), startPose(2), 'd', 'MarkerSize', 12, ...
         'MarkerFaceColor', [0.15, 0.85, 0.40], 'MarkerEdgeColor', [1 1 1], 'LineWidth', 2.0, ...
         'DisplayName', 'Origin Depot [3.0, 3.0]');

    % Target Docking Bay Zone (Rectangular Layout)
    boxX = [goalX-1.0, goalX+1.0, goalX+1.0, goalX-1.0, goalX-1.0];
    boxY = [goalY-0.8, goalY-0.8, goalY+0.8, goalY+0.8, goalY-0.8];
    plot(ax, boxX, boxY, '--', 'Color', [1.00, 0.80, 0.20], 'LineWidth', 2.0, ...
         'DisplayName', 'Docking Bay Zone');

    % Target Terminal Center Marker (Golden Bullseye Square)
    plot(ax, goalPose(1), goalPose(2), 's', 'MarkerSize', 14, ...
         'MarkerFaceColor', [1.00, 0.72, 0.10], 'MarkerEdgeColor', [0.95, 0.20, 0.10], ...
         'LineWidth', 2.2, 'DisplayName', sprintf('Target Terminal [%.1f, %.1f]', goalX, goalY));

    % Robot visual shape (Custom AGV Chassis footprint: ~0.55m length, 0.45m width)
    footprintL = 0.55;
    footprintW = 0.45;
    cornerOffsets = [-footprintL/2, -footprintW/2;
                      footprintL/2, -footprintW/2;
                      footprintL/2,  footprintW/2;
                     -footprintL/2,  footprintW/2]';

    robotPatch = patch(ax, 'XData', [], 'YData', [], 'FaceColor', [0.15 0.65 0.35], ...
                       'FaceAlpha', 0.80, 'EdgeColor', [0.1 0.4 0.2], 'LineWidth', 1.5, ...
                       'DisplayName', 'AGV Mobile Robot');
    robotHeading = quiver(ax, 0, 0, 0, 0, 0.7, 'Color', 'y', 'LineWidth', 2.5, 'MaxHeadSize', 2);
    trailPlot = plot(ax, startPose(1), startPose(2), 'c-', 'LineWidth', 2.0, 'DisplayName', 'Actual Trajectory');

    legend(ax, 'Location', 'northeast', 'FontSize', 9, ...
           'Color', [0.12, 0.15, 0.20], 'TextColor', [0.92, 0.95, 0.98], ...
           'EdgeColor', [0.30, 0.40, 0.52]);

    %% 6. Simulation Execution Loop
    currentPose = startPose;
    goalRadius = 0.40; % meters
    sampleTime = 0.05; % 20 Hz
    maxSimTime = 65.0; % seconds
    simTime = 0;

    trajectoryHistory = currentPose(1:2);

    statusText = text(ax, 1.5, 24.2, ...
        sprintf('  Time: 0.00s | Distance: %.2fm  ', norm(currentPose(1:2) - goalPose(1:2))), ...
        'FontSize', 10, 'FontWeight', 'bold', 'Color', [0.20, 0.95, 0.85], ...
        'BackgroundColor', [0.12, 0.18, 0.26], 'EdgeColor', [0.25, 0.65, 0.85], 'Margin', 6);

    fprintf('\nNavigating AGV along planned path...\n');

    while (norm(currentPose(1:2) - goalPose(1:2)) > goalRadius) && (simTime < maxSimTime)
        [v, omega] = controller(currentPose);

        currentPose(1) = currentPose(1) + v * cos(currentPose(3)) * sampleTime;
        currentPose(2) = currentPose(2) + v * sin(currentPose(3)) * sampleTime;
        currentPose(3) = currentPose(3) + omega * sampleTime;
        currentPose(3) = atan2(sin(currentPose(3)), cos(currentPose(3)));

        simTime = simTime + sampleTime;
        trajectoryHistory = [trajectoryHistory; currentPose(1:2)];

        R = [cos(currentPose(3)), -sin(currentPose(3));
             sin(currentPose(3)),  cos(currentPose(3))];
        transformedCorners = R * cornerOffsets + currentPose(1:2)';

        set(robotPatch, 'XData', transformedCorners(1, :), 'YData', transformedCorners(2, :));
        set(robotHeading, 'XData', currentPose(1), 'YData', currentPose(2), ...
                          'UData', cos(currentPose(3)), 'VData', sin(currentPose(3)));
        set(trailPlot, 'XData', trajectoryHistory(:, 1), 'YData', trajectoryHistory(:, 2));

        distRemaining = norm(currentPose(1:2) - goalPose(1:2));
        set(statusText, 'String', sprintf('Time: %.2fs | Speed: %.2fm/s | Distance: %.2fm', ...
            simTime, v, distRemaining));

        drawnow limitrate;
        pause(sampleTime * 0.3);
    end

    %% 7. Summary
    fprintf('\n=================================================================\n');
    if norm(currentPose(1:2) - goalPose(1:2)) <= goalRadius
        fprintf(' SUCCESS: AGV arrived at target destination!\n');
        set(statusText, 'String', sprintf('GOAL REACHED! Final Time: %.2fs', simTime), 'BackgroundColor', [0.8 1.0 0.8]);
    else
        fprintf(' SIMULATION ENDED: Time limit reached.\n');
    end
    fprintf(' Final Position: [%.2f, %.2f]\n', currentPose(1), currentPose(2));
    fprintf(' Total Travel Time: %.2f seconds\n', simTime);
    fprintf('=================================================================\n');
end
