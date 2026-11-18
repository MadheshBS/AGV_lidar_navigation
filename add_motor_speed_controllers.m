% add_motor_speed_controllers.m
% Add closed-loop motor PI speed controllers with calibrated sign polarity

bdclose('all');
clear; clc;
mdl = 'AGV__Frame';
load_system(mdl);
run('AGV__Frame_DataFile.m');

fprintf('Updating motor speed controllers inside %s ...\n', mdl);

% Enable sensing on Revolute joints
set_param([mdl '/Revolute'], 'SenseVelocity', 'on', 'SensePosition', 'on');
set_param([mdl '/Revolute1'], 'SenseVelocity', 'on', 'SensePosition', 'on');

% ---------------- Left Motor Controller ----------------
% Add Gain -1 for Left Velocity to match forward rotation polarity
gainLeftVel = [mdl '/Gain_Left_Vel'];
if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'Gain_Left_Vel'))
    add_block('simulink/Math Operations/Gain', gainLeftVel);
    set_param(gainLeftVel, 'Gain', '-1');
    
    % Reconnect: PS_Left_Vel -> Gain_Left_Vel -> Sum_Left (port 2) & w_left_sensed
    lines = get_param([mdl '/PS_Left_Vel'], 'LineHandles');
    if lines.Outport(1) ~= -1
        delete_line(lines.Outport(1));
    end
    add_line(mdl, 'PS_Left_Vel/1', 'Gain_Left_Vel/1');
    add_line(mdl, 'Gain_Left_Vel/1', 'Sum_Left/2');
    add_line(mdl, 'Gain_Left_Vel/1', 'w_left_sensed/1');
end

% ---------------- Right Motor Controller ----------------
gainRightVel = [mdl '/Gain_Right_Vel'];
if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'Gain_Right_Vel'))
    add_block('simulink/Math Operations/Gain', gainRightVel);
    set_param(gainRightVel, 'Gain', '-1');
    
    % Reconnect: PS_Right_Vel -> Gain_Right_Vel -> Sum_Right (port 2) & w_right_sensed
    lines = get_param([mdl '/PS_Right_Vel'], 'LineHandles');
    if lines.Outport(1) ~= -1
        delete_line(lines.Outport(1));
    end
    add_line(mdl, 'PS_Right_Vel/1', 'Gain_Right_Vel/1');
    add_line(mdl, 'Gain_Right_Vel/1', 'Sum_Right/2');
    add_line(mdl, 'Gain_Right_Vel/1', 'w_right_sensed/1');
end

% ---------------- Encoder Gains (-1 so forward angle is positive) ----------------
gainLeftEnc = [mdl '/Gain_Left_Enc'];
if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'Gain_Left_Enc'))
    add_block('simulink/Math Operations/Gain', gainLeftEnc);
    set_param(gainLeftEnc, 'Gain', '-1');
    
    lines = get_param([mdl '/PS_Left_Pos'], 'LineHandles');
    if lines.Outport(1) ~= -1
        delete_line(lines.Outport(1));
    end
    add_line(mdl, 'PS_Left_Pos/1', 'Gain_Left_Enc/1');
    add_line(mdl, 'Gain_Left_Enc/1', 'theta_left_enc/1');
end

gainRightEnc = [mdl '/Gain_Right_Enc'];
if isempty(find_system(mdl, 'SearchDepth', 1, 'Name', 'Gain_Right_Enc'))
    add_block('simulink/Math Operations/Gain', gainRightEnc);
    set_param(gainRightEnc, 'Gain', '-1');
    
    lines = get_param([mdl '/PS_Right_Pos'], 'LineHandles');
    if lines.Outport(1) ~= -1
        delete_line(lines.Outport(1));
    end
    add_line(mdl, 'PS_Right_Pos/1', 'Gain_Right_Enc/1');
    add_line(mdl, 'Gain_Right_Enc/1', 'theta_right_enc/1');
end

% Set PI Controller gains: Kp=5.0 N*m/(rad/s), Ki=8.0 N*m/rad, Saturation=[-20, 20]
set_param([mdl '/PI_Left'], 'P', '5', 'I', '8', 'D', '0.05', ...
    'LimitOutput', 'on', 'UpperSaturationLimit', '20', 'LowerSaturationLimit', '-20');
set_param([mdl '/PI_Right'], 'P', '5', 'I', '8', 'D', '0.05', ...
    'LimitOutput', 'on', 'UpperSaturationLimit', '20', 'LowerSaturationLimit', '-20');

save_system(mdl);
close_system(mdl, 0);
fprintf('Motor speed controllers with calibrated polarity configured successfully!\n');
