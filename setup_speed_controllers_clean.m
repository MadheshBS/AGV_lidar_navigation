% setup_speed_controllers_clean.m
bdclose('all');
clear; clc;
mdl = 'AGV__Frame';
load_system(mdl);
run('AGV__Frame_DataFile.m');

fprintf('Re-wiring clean motor speed controllers with confirmed negative feedback...\n');

% Ensure connections from PI to PS Converters
linesL = get_param([mdl '/PS_Conv_Left'], 'LineHandles');
if linesL.Inport(1) ~= -1, delete_line(linesL.Inport(1)); end
linesR = get_param([mdl '/PS_Conv_Right'], 'LineHandles');
if linesR.Inport(1) ~= -1, delete_line(linesR.Inport(1)); end

add_line(mdl, 'PI_Left/1', 'PS_Conv_Left/1');
add_line(mdl, 'PI_Right/1', 'PS_Conv_Right/1');

% Set gains to -1 so -w_raw becomes +w_fwd
set_param([mdl '/Gain_Left_Vel'], 'Gain', '-1');
set_param([mdl '/Gain_Right_Vel'], 'Gain', '-1');
set_param([mdl '/Gain_Left_Enc'], 'Gain', '-1');
set_param([mdl '/Gain_Right_Enc'], 'Gain', '-1');

% Set PI Controller gains: Kp = 2.0, Ki = 1.5, Limit = [-15, 15]
set_param([mdl '/PI_Left'], 'P', '2.0', 'I', '1.5', 'D', '0', ...
    'LimitOutput', 'on', 'UpperSaturationLimit', '15', 'LowerSaturationLimit', '-15');
set_param([mdl '/PI_Right'], 'P', '2.0', 'I', '1.5', 'D', '0', ...
    'LimitOutput', 'on', 'UpperSaturationLimit', '15', 'LowerSaturationLimit', '-15');

save_system(mdl);

% Now test 2.0s straight drive with ramp!
fprintf('Testing 2.0s straight drive with tuned PI controllers...\n');
t_test = (0:0.02:2.0)';
v_target = 0.35; % m/s
w_target = v_target / 0.108; % 3.24 rad/s
w_ramp = min(w_target, (w_target / 0.3) * t_test);

simIn = Simulink.SimulationInput(mdl);
simIn = simIn.setModelParameter('StopTime', '2.0');
simIn = simIn.setExternalInput([t_test, w_ramp, w_ramp]);
simOut = sim(simIn);

pos = squeeze(simOut.yout{3}.Values.Data)';
eul = squeeze(simOut.yout{4}.Values.Data)';
w_L = squeeze(simOut.yout{5}.Values.Data);
w_R = squeeze(simOut.yout{6}.Values.Data);

fprintf('RESULTS:\n');
fprintf('  Forward Displacement dX: %+.3f m (Expected: ~%.3f m)\n', ...
    pos(end,1)-pos(1,1), v_target * 1.85);
fprintf('  Lateral Drift dZ:        %+.3f m\n', pos(end,3)-pos(1,3));
fprintf('  Steady Wheel Speeds:     w_L = %.2f rad/s, w_R = %.2f rad/s (Target = %.2f)\n', ...
    mean(w_L(end-20:end)), mean(w_R(end-20:end)), w_target);
fprintf('  Chassis Yaw Drift:       %.2f deg\n', (eul(end,1)-eul(1,1))*180/pi);

close_system(mdl, 0);
