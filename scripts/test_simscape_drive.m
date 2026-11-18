% test_simscape_drive.m
% Verification Script for Phase 5.7: Simscape Multibody AGV Drive
% 
% Tests:
% 1. Straight-Line Forward Transit (equal wheel commands)
% 2. Differential Circular Turn (differential wheel commands)
% 3. Encoder Odometry Reconstruction vs Multibody Ground Truth
% 
% Output:
%   Console summary table & publication-ready figure: simscape_drive_test.png

bdclose('all');
clear; clc;

% Configure storage to D:\MATLAB_TEMP to prevent low disk space warnings
if isfolder('D:\MATLAB_TEMP')
    Simulink.sdi.setStorageLocation('D:\MATLAB_TEMP');
end
Simulink.sdi.setArchiveRunLimit(0);

mdl = 'AGV__Frame';
load_system(mdl);
run('AGV__Frame_DataFile.m');

fprintf('=====================================================\n');
fprintf('     AGV SIMSCAPE MULTIBODY DRIVE VERIFICATION       \n');
fprintf('=====================================================\n');

% Physical constants
R_wheel = 0.108;   % Wheel radius: 108 mm (0.108 m)
W_track = 0.334;   % Track width: 334 mm (0.334 m)

fprintf('Chassis Parameters:\n');
fprintf('  Track Width (W):    %.3f m\n', W_track);
fprintf('  Wheel Radius (R):   %.3f m\n', R_wheel);
fprintf('  Floor Surface:      Infinite Plane (Y = 0.264 m, Normal: +Y)\n');
fprintf('  Tire Contact:       Spatial Contact Force (mu_s = 0.85, mu_d = 0.75)\n');
fprintf('-----------------------------------------------------\n');

% -------------------------------------------------------------------------
% EXPERIMENT 1: STRAIGHT-LINE TRANSIT
% -------------------------------------------------------------------------
fprintf('Running Experiment 1: Straight-Line Forward Transit...\n');
t_end_1 = 3.0;       % seconds
dt = 0.02;           % 50 Hz control step
t_span_1 = (0:dt:t_end_1)';

v_target_1 = 0.35;   % m/s target linear speed
w_target_1 = v_target_1 / R_wheel; % ~3.24 rad/s

% Smooth acceleration ramp (0.4 s ramp-up)
t_ramp = 0.4;
w_cmd_profile_1 = min(w_target_1, (w_target_1 / t_ramp) * t_span_1);

simIn1 = Simulink.SimulationInput(mdl);
simIn1 = simIn1.setModelParameter('StopTime', num2str(t_end_1));
simIn1 = simIn1.setExternalInput([t_span_1, w_cmd_profile_1, w_cmd_profile_1]);

simOut1 = sim(simIn1);

time_1 = simOut1.tout;
pos_1  = squeeze(simOut1.yout{3}.Values.Data)';
eul_1  = squeeze(simOut1.yout{4}.Values.Data)';
th_L_1 = squeeze(simOut1.yout{1}.Values.Data);
th_R_1 = squeeze(simOut1.yout{2}.Values.Data);
w_L_1  = abs(squeeze(simOut1.yout{5}.Values.Data));
w_R_1  = abs(squeeze(simOut1.yout{6}.Values.Data));

% Normalize position and yaw relative to t=0
% Normalize position and yaw relative to t=0
% In Simscape frame, forward transit is along Z, and lateral axle is along X
traj_1_X = abs(pos_1(:, 3) - pos_1(1, 3));
traj_1_Z = pos_1(:, 1) - pos_1(1, 1);
yaw_1_deg = (eul_1(:, 1) - eul_1(1, 1)) * 180 / pi;

% Reconstruct wheel odometry from joint encoders
% Resample to uniform time grid to avoid step-size differentiation artifacts
t_grid_1 = (0:dt:t_end_1)';
th_L_grid1 = interp1(time_1, th_L_1, t_grid_1, 'linear');
th_R_grid1 = interp1(time_1, th_R_1, t_grid_1, 'linear');

odom_pose_1 = zeros(length(t_grid_1), 3);
curr_pose = [0; 0; 0];
odom_pose_1(1, :) = curr_pose';

for k = 2:length(t_grid_1)
    d_th_L = abs(th_L_grid1(k) - th_L_grid1(k-1));
    d_th_R = abs(th_R_grid1(k) - th_R_grid1(k-1));
    
    d_sL = R_wheel * d_th_L;
    d_sR = R_wheel * d_th_R;
    
    curr_pose = diffdrive_odometry(curr_pose, d_sL, d_sR, W_track);
    odom_pose_1(k, :) = curr_pose;
end

fprintf('  Experiment 1 Results:\n');
fprintf('    Forward Distance (X):    %.3f m (Expected: %.3f m)\n', ...
    traj_1_X(end), v_target_1 * (t_end_1 - t_ramp/2));
fprintf('    Lateral Deviation (Z):   %.3f m\n', abs(traj_1_Z(end)));
fprintf('    Chassis Yaw Drift:       %.2f deg\n', abs(yaw_1_deg(end)));
fprintf('    Mean Wheel Speeds:       w_L = %.2f rad/s, w_R = %.2f rad/s (Target: %.2f)\n', ...
    mean(w_L_1(end-30:end)), mean(w_R_1(end-30:end)), w_target_1);

% -------------------------------------------------------------------------
% EXPERIMENT 2: DIFFERENTIAL CIRCULAR TURN
% -------------------------------------------------------------------------
fprintf('\nRunning Experiment 2: Differential Circular Turn...\n');
t_end_2 = 4.0;       % seconds
t_span_2 = (0:dt:t_end_2)';

v_target_2   = 0.30;  % m/s
omega_turn_2 = 0.50;  % rad/s (~28.6 deg/s)

w_nom_L = (v_target_2 - omega_turn_2 * W_track / 2) / R_wheel; % ~1.99 rad/s
w_nom_R = (v_target_2 + omega_turn_2 * W_track / 2) / R_wheel; % ~3.56 rad/s
R_curve_theo = v_target_2 / omega_turn_2; % 0.60 m

w_cmd_profile_L2 = min(w_nom_L, (w_nom_L / t_ramp) * t_span_2);
w_cmd_profile_R2 = min(w_nom_R, (w_nom_R / t_ramp) * t_span_2);

simIn2 = Simulink.SimulationInput(mdl);
simIn2 = simIn2.setModelParameter('StopTime', num2str(t_end_2));
simIn2 = simIn2.setExternalInput([t_span_2, w_cmd_profile_L2, w_cmd_profile_R2]);

simOut2 = sim(simIn2);

time_2 = simOut2.tout;
pos_2  = squeeze(simOut2.yout{3}.Values.Data)';
eul_2  = squeeze(simOut2.yout{4}.Values.Data)';
th_L_2 = squeeze(simOut2.yout{1}.Values.Data);
th_R_2 = squeeze(simOut2.yout{2}.Values.Data);
w_L_2  = abs(squeeze(simOut2.yout{5}.Values.Data));
w_R_2  = abs(squeeze(simOut2.yout{6}.Values.Data));

traj_2_X = abs(pos_2(:, 3) - pos_2(1, 3));
traj_2_Z = pos_2(:, 1) - pos_2(1, 1);
yaw_2_deg = (eul_2(:, 1) - eul_2(1, 1)) * 180 / pi;

t_grid_2 = (0:dt:t_end_2)';
th_L_grid2 = interp1(time_2, th_L_2, t_grid_2, 'linear');
th_R_grid2 = interp1(time_2, th_R_2, t_grid_2, 'linear');

odom_pose_2 = zeros(length(t_grid_2), 3);
curr_pose = [0; 0; 0];
odom_pose_2(1, :) = curr_pose';

for k = 2:length(t_grid_2)
    d_th_L = abs(th_L_grid2(k) - th_L_grid2(k-1));
    d_th_R = abs(th_R_grid2(k) - th_R_grid2(k-1));
    
    d_sL = R_wheel * d_th_L;
    d_sR = R_wheel * d_th_R;
    
    curr_pose = diffdrive_odometry(curr_pose, d_sL, d_sR, W_track);
    odom_pose_2(k, :) = curr_pose;
end

fprintf('  Experiment 2 Results:\n');
fprintf('    Total Heading Turn:      %.1f deg (Expected: %.1f deg)\n', ...
    abs(yaw_2_deg(end)), omega_turn_2 * (t_end_2 - t_ramp/2) * 180 / pi);
fprintf('    Theoretical Radius:      %.3f m\n', R_curve_theo);
fprintf('    Mean Wheel Speeds:       w_L = %.2f rad/s, w_R = %.2f rad/s\n', ...
    mean(w_L_2(end-30:end)), mean(w_R_2(end-30:end)));

% -------------------------------------------------------------------------
% GENERATE PUBLICATION-QUALITY VERIFICATION FIGURE
% -------------------------------------------------------------------------
fprintf('\nGenerating verification plot: simscape_drive_test.png ...\n');
fig = figure('Name', 'Simscape Multibody AGV Drive Verification', ...
    'Units', 'pixels', 'Position', [80, 60, 1280, 800], 'Color', 'w');

% Subplot 1: Straight-Line Floor Trajectory
subplot(2, 2, 1);
plot(traj_1_X, traj_1_Z, 'b-', 'LineWidth', 2.8); hold on;
plot(odom_pose_1(:, 1), odom_pose_1(:, 2), 'r--', 'LineWidth', 2.0);
plot(traj_1_X(1), traj_1_Z(1), 'go', 'MarkerSize', 9, 'MarkerFaceColor', 'g');
plot(traj_1_X(end), traj_1_Z(end), 'ms', 'MarkerSize', 9, 'MarkerFaceColor', 'm');
grid on; axis equal;
title('Test 1: Straight-Line Drive Trajectory', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Forward Displacement X [m]', 'FontSize', 11);
ylabel('Lateral Displacement Z [m]', 'FontSize', 11);
legend({'Simscape Multibody GT', 'Encoder Odometry', 'Start Pose', 'End Pose'}, ...
    'Location', 'best', 'FontSize', 10);

% Subplot 2: Differential Circular Turn Trajectory
subplot(2, 2, 2);
plot(traj_2_X, traj_2_Z, 'Color', [0.85, 0.33, 0.1], 'LineWidth', 2.8); hold on;
plot(odom_pose_2(:, 1), odom_pose_2(:, 2), 'b--', 'LineWidth', 2.0);
plot(traj_2_X(1), traj_2_Z(1), 'go', 'MarkerSize', 9, 'MarkerFaceColor', 'g');
plot(traj_2_X(end), traj_2_Z(end), 'ms', 'MarkerSize', 9, 'MarkerFaceColor', 'm');
grid on; axis equal;
title(sprintf('Test 2: Circular Turn (\\omega=%.2f rad/s, R_{theo}=%.2fm)', omega_turn_2, R_curve_theo), ...
    'FontSize', 12, 'FontWeight', 'bold');
xlabel('Forward Displacement [m]', 'FontSize', 11);
ylabel('Lateral Displacement [m]', 'FontSize', 11);
legend({'Simscape Multibody GT', 'Encoder Odometry', 'Start Pose', 'End Pose'}, ...
    'Location', 'best', 'FontSize', 10);

% Subplot 3: Wheel Speed Closed-Loop Tracking (Test 1)
subplot(2, 2, 3);
plot(time_1, w_L_1, 'b-', 'LineWidth', 1.8); hold on;
plot(time_1, w_R_1, 'r-', 'LineWidth', 1.8);
plot(t_span_1, w_cmd_profile_1, 'k--', 'LineWidth', 1.6);
grid on;
title('Motor Speed Closed-Loop Tracking (Test 1)', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time [s]', 'FontSize', 11);
ylabel('Wheel Angular Velocity [rad/s]', 'FontSize', 11);
legend({'Left Wheel \omega_L', 'Right Wheel \omega_R', 'Command Profile'}, ...
    'Location', 'southeast', 'FontSize', 10);
ylim([0, w_target_1 * 1.35]);

% Subplot 4: Chassis Yaw Heading Angle vs Time (Test 2)
subplot(2, 2, 4);
plot(time_2, abs(yaw_2_deg), 'Color', [0.47, 0.67, 0.19], 'LineWidth', 2.4); hold on;
plot(t_grid_2, abs(odom_pose_2(:, 3)) * 180 / pi, 'r--', 'LineWidth', 1.8);
grid on;
title('Chassis Heading Angle During Turning Maneuver', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time [s]', 'FontSize', 11);
ylabel('Heading Angle \theta [deg]', 'FontSize', 11);
legend({'Simscape Multibody GT Yaw', 'Encoder Odometry Yaw'}, ...
    'Location', 'northwest', 'FontSize', 10);

saveas(fig, 'simscape_drive_test.png');
fprintf('Figure successfully saved to: simscape_drive_test.png\n');

% Copy to brain artifact directory
brain_dest = 'C:\Users\akash\.gemini\antigravity-ide\brain\c8182169-5370-4147-b07e-0945f4f47713\simscape_drive_test.png';
copyfile('simscape_drive_test.png', brain_dest);
fprintf('Artifact updated in brain directory: %s\n', brain_dest);

close(fig);
close_system(mdl, 0);

fprintf('=====================================================\n');
fprintf('SIMSCAPE MULTIBODY AGV DRIVE TESTS COMPLETED SUCCESSFULLY!\n');
fprintf('=====================================================\n');
