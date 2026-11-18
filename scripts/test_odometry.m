%% test_odometry.m - Differential-Drive Kinematics & Odometry Drift Test
%
% PURPOSE:
%   Validates forward/inverse differential-drive kinematics and demonstrates
%   dead-reckoning odometry drift over a closed square trajectory.
%
% WHY THIS MATTERS (Viva Explanation):
%   In open-loop dead reckoning, minute wheel slip, surface unevenness, and
%   systematic wheel diameter discrepancies cause position and heading errors
%   to accumulate monotonically over time. This proves why external sensors
%   (such as 2D Lidar with Monte Carlo Localization) are required for
%   reliable industrial AGV navigation.
%
% REPRODUCIBILITY:
%   Fixed random seed (rng(42)) ensures deterministic results across runs.

clear; clc; close all;
rng(42); % Fix seed for reproducibility

%% 1. Load Parameters
p = agv_params();
fprintf('=====================================================\n');
fprintf('  AGV DIFFERENTIAL-DRIVE ODOMETRY & KINEMATICS TEST  \n');
fprintf('=====================================================\n');
fprintf('Track width (W):         %.3f m\n', p.trackWidth);
fprintf('Wheel radius (R):        %.3f m\n', p.wheelRadius);
fprintf('Simulation step (dt):    %.3f s (%.0f Hz)\n', p.dt, 1/p.dt);
fprintf('Left wheel bias:         %.3f\n', p.odom.leftWheelRadiusBias);
fprintf('Right wheel bias:        %.3f\n', p.odom.rightWheelRadiusBias);
fprintf('Encoder noise std:       %.4f m\n', p.odom.wheelEncoderNoiseStd);
fprintf('-----------------------------------------------------\n');

%% 2. Generate Square Maneuver Commands
% A square maneuver of side length L = 3.0 m
sideLength = 3.0;            % Side length of square (m)
linearSpeed = 0.5;           % Driving speed (m/s)
angularSpeed = 0.5;          % Turning speed (rad/s)
turnAngle = pi / 2;          % 90-degree CCW turn (rad)

timeDrive = sideLength / linearSpeed;      % 6.0 seconds per straight side
timeTurn  = turnAngle / angularSpeed;       % ~3.14 seconds per 90-deg turn

% Build motion segments for 4 sides and 4 turns
% Segment types: 1 = straight, 2 = 90-deg turn
segmentTypes = [1, 2, 1, 2, 1, 2, 1, 2];
segmentDurations = [timeDrive, timeTurn, timeDrive, timeTurn, ...
                    timeDrive, timeTurn, timeDrive, timeTurn];

% Discretize commands over time
timeVector = [];
vCmdVector = [];
wCmdVector = [];

for i = 1:length(segmentTypes)
    segT = segmentDurations(i);
    numSteps = round(segT / p.dt);
    
    if segmentTypes(i) == 1
        % Straight motion
        v_seg = repmat(linearSpeed, numSteps, 1);
        w_seg = zeros(numSteps, 1);
    else
        % In-place rotation
        v_seg = zeros(numSteps, 1);
        w_seg = repmat(angularSpeed, numSteps, 1);
    end
    
    vCmdVector = [vCmdVector; v_seg];
    wCmdVector = [wCmdVector; w_seg];
end

totalSteps = length(vCmdVector);
timeVector = (0:totalSteps-1)' * p.dt;

%% 3. Simulate Ground Truth vs Noisy Odometry
% Initial pose: [x=0, y=0, theta=0]
initialPose = [0.0, 0.0, 0.0];

truePoseLog = zeros(totalSteps, 3);
odomPoseLog = zeros(totalSteps, 3);

truePose = initialPose;
odomPose = initialPose;

truePoseLog(1, :) = truePose;
odomPoseLog(1, :) = odomPose;

for k = 1:totalSteps-1
    v_des = vCmdVector(k);
    w_des = wCmdVector(k);
    
    % Inverse kinematics: desired wheel velocities (m/s)
    [vL_true, vR_true, ~, ~] = diffdrive_inverse_kinematics(v_des, w_des, p.trackWidth, p.wheelRadius);
    
    % True wheel incremental displacements over dt (m)
    dsL_true = vL_true * p.dt;
    dsR_true = vR_true * p.dt;
    
    % True pose update (ideal world physics)
    truePose = diffdrive_odometry(truePose, dsL_true, dsR_true, p.trackWidth);
    
    % Measured wheel increments with systematic calibration bias & slip noise
    dsL_meas = dsL_true * p.odom.leftWheelRadiusBias  + randn() * p.odom.wheelEncoderNoiseStd * sqrt(p.dt);
    dsR_meas = dsR_true * p.odom.rightWheelRadiusBias + randn() * p.odom.wheelEncoderNoiseStd * sqrt(p.dt);
    
    % Dead-reckoning odometry pose update (integrating noisy wheel measurements)
    odomPose = diffdrive_odometry(odomPose, dsL_meas, dsR_meas, p.trackWidth);
    
    truePoseLog(k+1, :) = truePose;
    odomPoseLog(k+1, :) = odomPose;
end

%% 4. Error Analysis
posError = sqrt((truePoseLog(:, 1) - odomPoseLog(:, 1)).^2 + ...
                (truePoseLog(:, 2) - odomPoseLog(:, 2)).^2);
headingError = abs(atan2(sin(truePoseLog(:, 3) - odomPoseLog(:, 3)), ...
                         cos(truePoseLog(:, 3) - odomPoseLog(:, 3))));

finalPosError = posError(end);
maxPosError   = max(posError);
finalHeadingErrorDeg = rad2deg(headingError(end));

fprintf('SIMULATION COMPLETED:\n');
fprintf('  Total Path Length:       %.2f m\n', 4 * sideLength);
fprintf('  Total Duration:          %.2f s\n', timeVector(end));
fprintf('  Final Ground Truth Pose: [x=%.3f m, y=%.3f m, theta=%.2f deg]\n', ...
    truePoseLog(end,1), truePoseLog(end,2), rad2deg(truePoseLog(end,3)));
fprintf('  Final Odometry Pose:     [x=%.3f m, y=%.3f m, theta=%.2f deg]\n', ...
    odomPoseLog(end,1), odomPoseLog(end,2), rad2deg(odomPoseLog(end,3)));
fprintf('  Final Position Drift:    %.3f m\n', finalPosError);
fprintf('  Maximum Position Drift:  %.3f m\n', maxPosError);
fprintf('  Final Heading Drift:     %.2f deg\n', finalHeadingErrorDeg);
fprintf('=====================================================\n');

%% 5. Visualization & Plotting
fig = figure('Name', 'AGV Odometry Drift Test', 'Units', 'pixels', ...
             'Position', [100, 100, 1150, 520], 'Color', 'w');

% Subplot 1: 2D Trajectory Comparison
subplot(1, 2, 1);
hold on; grid on; box on;
axis equal;
set(gca, 'Color', 'w', 'XColor', 'k', 'YColor', 'k', 'GridColor', [0.75 0.75 0.75], 'GridAlpha', 0.7);

plot(truePoseLog(:, 1), truePoseLog(:, 2), 'b-', 'LineWidth', 2.4, 'DisplayName', 'Ground Truth (True Path)');
plot(odomPoseLog(:, 1), odomPoseLog(:, 2), 'r--', 'LineWidth', 2.0, 'DisplayName', 'Wheel Odometry (Dead Reckoning)');

% Draw Start/End Markers
plot(truePoseLog(1, 1), truePoseLog(1, 2), 'o', 'MarkerFaceColor', [0.1, 0.75, 0.1], ...
     'MarkerEdgeColor', 'k', 'MarkerSize', 9, 'DisplayName', 'Start Point (0,0)');
plot(truePoseLog(end, 1), truePoseLog(end, 2), 's', 'MarkerFaceColor', [0 0.3 0.9], ...
     'MarkerEdgeColor', 'k', 'MarkerSize', 10, 'DisplayName', 'True End Pose');
plot(odomPoseLog(end, 1), odomPoseLog(end, 2), 'p', 'MarkerFaceColor', [0.9 0.1 0.1], ...
     'MarkerEdgeColor', 'k', 'MarkerSize', 12, 'DisplayName', sprintf('Odom End Pose (Drift = %.2fm)', finalPosError));

% Draw robot orientation arrows at key waypoints
sampleIdx = round(linspace(1, totalSteps, 10));
for idx = sampleIdx(1:end-1)
    quiver(truePoseLog(idx, 1), truePoseLog(idx, 2), ...
           0.35 * cos(truePoseLog(idx, 3)), 0.35 * sin(truePoseLog(idx, 3)), ...
           0, 'Color', [0 0.3 0.85], 'LineWidth', 1.4, 'MaxHeadSize', 0.6, 'HandleVisibility', 'off');
    quiver(odomPoseLog(idx, 1), odomPoseLog(idx, 2), ...
           0.35 * cos(odomPoseLog(idx, 3)), 0.35 * sin(odomPoseLog(idx, 3)), ...
           0, 'Color', [0.85 0.15 0.15], 'LineWidth', 1.4, 'MaxHeadSize', 0.6, 'HandleVisibility', 'off');
end

xlabel('X Position [m]', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
ylabel('Y Position [m]', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
title({'2D Path Comparison: True vs Dead-Reckoning Odometry'; ...
       sprintf('(Square Path: 4 x %.1fm sides, W = %.3fm, R = %.3fm)', sideLength, p.trackWidth, p.wheelRadius)}, ...
       'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');
lgd = legend('Location', 'southoutside', 'FontSize', 9);
set(lgd, 'Color', 'w', 'TextColor', 'k', 'EdgeColor', [0.7 0.7 0.7]);
xlim([-1.0, sideLength + 1.2]);
ylim([-1.2, sideLength + 1.0]);

% Subplot 2: Cumulative Drift Errors Over Time
subplot(1, 2, 2);
set(gca, 'Color', 'w');
yyaxis left;
plot(timeVector, posError, 'r-', 'LineWidth', 2.2);
ylabel('Position Drift Error [m]', 'FontSize', 11, 'FontWeight', 'bold');
ax = gca;
ax.YColor = [0.8 0 0];
grid on; box on;
set(ax, 'GridColor', [0.75 0.75 0.75], 'GridAlpha', 0.7);

yyaxis right;
plot(timeVector, rad2deg(headingError), 'm--', 'LineWidth', 1.9);
ylabel('Heading Drift Error [deg]', 'FontSize', 11, 'FontWeight', 'bold');
ax.YColor = [0.6 0 0.6];

xlabel('Time [seconds]', 'FontSize', 11, 'FontWeight', 'bold', 'Color', 'k');
title({'Odometry Drift Accumulation Over Time'; ...
       '(Open-Loop Dead Reckoning Error Growth)'}, ...
       'FontSize', 12, 'FontWeight', 'bold', 'Color', 'k');

% Text annotation box
annotationText = sprintf('Final Position Drift: %.2f m\nFinal Heading Drift: %.1f^\\circ\nMax Position Error: %.2f m', ...
                         finalPosError, finalHeadingErrorDeg, maxPosError);
annotation('textbox', [0.55, 0.72, 0.23, 0.13], 'String', annotationText, ...
           'FitBoxToText', 'on', 'BackgroundColor', [1 1 0.92], 'EdgeColor', [0.5 0.5 0.5], ...
           'Color', 'k', 'FontSize', 9, 'FontWeight', 'bold');

% Save output plot to file
plotFileName = 'odometry_drift_test.png';
exportgraphics(fig, plotFileName, 'Resolution', 150);
fprintf('Plot successfully saved to: %s\n', plotFileName);
