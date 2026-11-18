% setup_agv_simscape.m
% Configures AGV__Frame.slx with:
% 1. Free chassis mobility (6-DOF Joint between ground transform and Chasis_1_RIGID)
% 2. Warehouse floor plane & Spatial Contact Force blocks on Left, Right, and Caster wheels
%    (Floor normal points +Y via RotationAngle = -90 deg)
% 3. Dynamic torque-actuated wheel revolute joints with realistic bearing damping (0.15 N*m/(rad/s))
% 4. Closed-loop PI speed controllers (Kp=2.0, Ki=4.0) with confirmed negative feedback
% 5. Encoder sensors (theta_L, theta_R) and sensed speeds (w_L, w_R)
% 6. Transform Sensor measuring chassis 3D position and yaw

bdclose('all');
clear; clc;

mdl = 'AGV__Frame';

% Restore from original backup to ensure clean state
if isfile('AGV__Frame_original_backup.slx')
    copyfile('AGV__Frame_original_backup.slx', 'AGV__Frame.slx');
    fprintf('Restored clean AGV__Frame.slx from backup.\n');
else
    copyfile('AGV__Frame.slx', 'AGV__Frame_original_backup.slx');
    fprintf('Created backup AGV__Frame_original_backup.slx.\n');
end

fprintf('Loading model: %s ...\n', mdl);
load_system(mdl);
load_system('sm_lib');
load_system('nesl_utility');
run('AGV__Frame_DataFile.m');

% -------------------------------------------------------------------------
% 0. REMOVE KINEMATIC-LOOP CUT JOINTS
% -------------------------------------------------------------------------
fprintf('Removing kinematic-loop cut joints...\n');
cutJoints = {'Cylindrical1', 'Cylindrical2', 'Planar'};
for k = 1:length(cutJoints)
    blkPath = [mdl '/' cutJoints{k}];
    found = find_system(mdl, 'SearchDepth', 1, 'Name', cutJoints{k});
    if ~isempty(found)
        fprintf('  Removing cut joint: %s\n', cutJoints{k});
        delete_block(blkPath);
    else
        fprintf('  Cut joint %s not found (skip).\n', cutJoints{k});
    end
end

fprintf('Configuring model parameters...\n');
set_param(mdl, 'Solver', 'daessc', 'SimMechanicsOpenEditorOnUpdate', 'off');

% -------------------------------------------------------------------------
% 1. INSERT 6-DOF JOINT BETWEEN TRANSFORM AND Chasis_1_RIGID
% -------------------------------------------------------------------------
fprintf('Configuring 6-DOF Joint for chassis free mobility...\n');
lines = get_param([mdl '/Transform'], 'LineHandles');
if lines.RConn(1) ~= -1
    delete_line(lines.RConn(1));
end

jointName = [mdl '/Chasis_Free_Joint'];
add_block('sm_lib/Joints/6-DOF Joint', jointName);
add_line(mdl, 'Transform/RConn1', 'Chasis_Free_Joint/LConn1');
add_line(mdl, 'Chasis_Free_Joint/RConn1', 'Chasis_1_RIGID/LConn2');

% -------------------------------------------------------------------------
% 2. ADD WAREHOUSE FLOOR GEOMETRY
%    Rotate -90 deg around X so normal points in +Y direction (upward against gravity)
% -------------------------------------------------------------------------
fprintf('Adding Floor Transform and Infinite Plane...\n');
floorTrName = [mdl '/Floor_Transform'];
floorPlaneName = [mdl '/Floor_Plane'];
add_block('sm_lib/Frames and Transforms/Rigid Transform', floorTrName);
set_param(floorTrName, ...
    'TranslationMethod', 'Cartesian', ...
    'TranslationCartesianOffset', '[0 0.263809 0]', ...
    'TranslationCartesianOffsetUnits', 'm', ...
    'RotationMethod', 'StandardAxis', ...
    'RotationStandardAxis', '+X', ...
    'RotationAngle', '-90', ...
    'RotationAngleUnits', 'deg');
add_line(mdl, 'World/RConn1', 'Floor_Transform/LConn1');

add_block('sm_lib/Curves and Surfaces/Infinite Plane', floorPlaneName);
add_line(mdl, 'Floor_Transform/RConn1', 'Floor_Plane/LConn1');

% -------------------------------------------------------------------------
% 3. ADD WHEEL CONTACT DISKS & SPATIAL CONTACT FORCES
% -------------------------------------------------------------------------
wheels = {
    'LeftWheel',   'motor_wheel_3_RIGID/LConn1', 0.108;
    'RightWheel',  'motor_wheel_4_RIGID/LConn1', 0.108;
    'CasterWheel', 'caster_wheel2_1_RIGID/LConn1', 0.0368
};

for w = 1:size(wheels, 1)
    wName = wheels{w, 1};
    portTarget = wheels{w, 2};
    rad = wheels{w, 3};
    
    diskName = [mdl '/' wName '_Disk'];
    scfName  = [mdl '/' wName '_SCF'];
    
    fprintf('Adding contact disk and SCF for %s (R=%.3fm)...\n', wName, rad);
    % Add Disk
    add_block('sm_lib/Curves and Surfaces/Disk', diskName);
    set_param(diskName, 'Radius', num2str(rad));
    add_line(mdl, portTarget, [wName '_Disk/LConn1']);
    
    % Add Spatial Contact Force
    add_block('sm_lib/Forces and Torques/Spatial Contact Force', scfName);
    set_param(scfName, ...
        'NormalForceType', 'SmoothSpringDamper', ...
        'NormalStiffness', '5e5', ...
        'NormalDamping', '5e3', ...
        'FrictionType', 'SmoothStickSlip', ...
        'CoefficientOfStaticFriction', '0.85', ...
        'CoefficientOfDynamicFriction', '0.75', ...
        'FrictionalCriticalVelocity', '0.05');
        
    % Connect Floor Plane to SCF Base (LConn1)
    add_line(mdl, 'Floor_Plane/RConn1', [wName '_SCF/LConn1']);
    % Connect Wheel Disk to SCF Follower (RConn1)
    add_line(mdl, [wName '_Disk/RConn1'], [wName '_SCF/RConn1']);
end

% -------------------------------------------------------------------------
% 4. CONFIGURE REVOLUTE ACTUATION (Left: Revolute, Right: Revolute1)
%    Forward dynamics with torque actuation, bearing damping & PI controllers
% -------------------------------------------------------------------------
fprintf('Configuring Revolute Joint Actuation & Speed Controllers...\n');
% Revolute (Left Motor):
set_param([mdl '/Revolute'], ...
    'MotionActuationMode', 'ComputedMotion', ...
    'TorqueActuationMode', 'InputTorque', ...
    'DampingCoefficient', '0.15', ...
    'DampingCoefficientUnits', 'N*m/(rad/s)', ...
    'SensePosition', 'on', ...
    'SenseVelocity', 'on');

% Revolute1 (Right Motor):
set_param([mdl '/Revolute1'], ...
    'MotionActuationMode', 'ComputedMotion', ...
    'TorqueActuationMode', 'InputTorque', ...
    'DampingCoefficient', '0.15', ...
    'DampingCoefficientUnits', 'N*m/(rad/s)', ...
    'SensePosition', 'on', ...
    'SenseVelocity', 'on');

% Actuators: Simulink-PS Converters (N*m)
add_block('nesl_utility/Simulink-PS Converter', [mdl '/PS_Conv_Left']);
set_param([mdl '/PS_Conv_Left'], 'Unit', 'N*m');
add_line(mdl, 'PS_Conv_Left/RConn1', 'Revolute/LConn2');

add_block('nesl_utility/Simulink-PS Converter', [mdl '/PS_Conv_Right']);
set_param([mdl '/PS_Conv_Right'], 'Unit', 'N*m');
add_line(mdl, 'PS_Conv_Right/RConn1', 'Revolute1/LConn2');

% Command Inports
add_block('simulink/Sources/In1', [mdl '/w_left_cmd']);
set_param([mdl '/w_left_cmd'], 'Port', '1');
add_block('simulink/Sources/In1', [mdl '/w_right_cmd']);
set_param([mdl '/w_right_cmd'], 'Port', '2');

% Velocity Sensing & Negative Feedback Gains (+1)
add_block('nesl_utility/PS-Simulink Converter', [mdl '/PS_Left_Vel']);
add_line(mdl, 'Revolute/RConn3', 'PS_Left_Vel/LConn1');
add_block('simulink/Math Operations/Gain', [mdl '/Gain_Left_Vel']);
set_param([mdl '/Gain_Left_Vel'], 'Gain', '1');
add_line(mdl, 'PS_Left_Vel/1', 'Gain_Left_Vel/1');

add_block('nesl_utility/PS-Simulink Converter', [mdl '/PS_Right_Vel']);
add_line(mdl, 'Revolute1/RConn3', 'PS_Right_Vel/LConn1');
add_block('simulink/Math Operations/Gain', [mdl '/Gain_Right_Vel']);
set_param([mdl '/Gain_Right_Vel'], 'Gain', '1');
add_line(mdl, 'PS_Right_Vel/1', 'Gain_Right_Vel/1');

% Sum Blocks for PI Control Error: e = w_cmd - w_meas
add_block('simulink/Math Operations/Sum', [mdl '/Sum_Left']);
set_param([mdl '/Sum_Left'], 'Inputs', '+-');
add_line(mdl, 'w_left_cmd/1', 'Sum_Left/1');
add_line(mdl, 'Gain_Left_Vel/1', 'Sum_Left/2');

add_block('simulink/Math Operations/Sum', [mdl '/Sum_Right']);
set_param([mdl '/Sum_Right'], 'Inputs', '+-');
add_line(mdl, 'w_right_cmd/1', 'Sum_Right/1');
add_line(mdl, 'Gain_Right_Vel/1', 'Sum_Right/2');

% PI Controllers (Kp = 2.0, Ki = 4.0, Saturation = [-25, 25] N*m)
add_block('simulink/Continuous/PID Controller', [mdl '/PI_Left']);
set_param([mdl '/PI_Left'], 'P', '2.0', 'I', '4.0', 'D', '0', ...
    'LimitOutput', 'on', 'UpperSaturationLimit', '25', 'LowerSaturationLimit', '-25');
add_line(mdl, 'Sum_Left/1', 'PI_Left/1');
add_line(mdl, 'PI_Left/1', 'PS_Conv_Left/1');

add_block('simulink/Continuous/PID Controller', [mdl '/PI_Right']);
set_param([mdl '/PI_Right'], 'P', '2.0', 'I', '4.0', 'D', '0', ...
    'LimitOutput', 'on', 'UpperSaturationLimit', '25', 'LowerSaturationLimit', '-25');
add_line(mdl, 'Sum_Right/1', 'PI_Right/1');
add_line(mdl, 'PI_Right/1', 'PS_Conv_Right/1');

% Encoders (Position sensing with polarity gain +1)
add_block('nesl_utility/PS-Simulink Converter', [mdl '/PS_Left_Pos']);
add_line(mdl, 'Revolute/RConn2', 'PS_Left_Pos/LConn1');
add_block('simulink/Math Operations/Gain', [mdl '/Gain_Left_Enc']);
set_param([mdl '/Gain_Left_Enc'], 'Gain', '1');
add_line(mdl, 'PS_Left_Pos/1', 'Gain_Left_Enc/1');
add_block('simulink/Sinks/Out1', [mdl '/theta_left_enc']);
set_param([mdl '/theta_left_enc'], 'Port', '1');
add_line(mdl, 'Gain_Left_Enc/1', 'theta_left_enc/1');

add_block('nesl_utility/PS-Simulink Converter', [mdl '/PS_Right_Pos']);
add_line(mdl, 'Revolute1/RConn2', 'PS_Right_Pos/LConn1');
add_block('simulink/Math Operations/Gain', [mdl '/Gain_Right_Enc']);
set_param([mdl '/Gain_Right_Enc'], 'Gain', '1');
add_line(mdl, 'PS_Right_Pos/1', 'Gain_Right_Enc/1');
add_block('simulink/Sinks/Out1', [mdl '/theta_right_enc']);
set_param([mdl '/theta_right_enc'], 'Port', '2');
add_line(mdl, 'Gain_Right_Enc/1', 'theta_right_enc/1');

% Sensed Speed Outports (Ports 5 and 6)
add_block('simulink/Sinks/Out1', [mdl '/w_left_sensed']);
set_param([mdl '/w_left_sensed'], 'Port', '5');
add_line(mdl, 'Gain_Left_Vel/1', 'w_left_sensed/1');

add_block('simulink/Sinks/Out1', [mdl '/w_right_sensed']);
set_param([mdl '/w_right_sensed'], 'Port', '6');
add_line(mdl, 'Gain_Right_Vel/1', 'w_right_sensed/1');

% -------------------------------------------------------------------------
% 5. ADD TRANSFORM SENSOR FOR CHASSIS POSE (Ports 3 and 4)
% -------------------------------------------------------------------------
fprintf('Adding Chassis Pose Sensor (Transform Sensor)...\n');
add_block('sm_lib/Frames and Transforms/Transform Sensor', [mdl '/Chassis_Pose_Sensor']);
set_param([mdl '/Chassis_Pose_Sensor'], ...
    'SenseXYZ', 'on', ...
    'SenseRotationSequence', 'on', ...
    'RotationSequence', 'YXZ');
    
% Base: World, Follower: Chasis_1_RIGID
add_line(mdl, 'World/RConn1', 'Chassis_Pose_Sensor/LConn1');
add_line(mdl, 'Chasis_1_RIGID/LConn2', 'Chassis_Pose_Sensor/RConn1');

% Outputs: Position XYZ (Port 3) and Euler Angles (Port 4)
add_block('nesl_utility/PS-Simulink Converter', [mdl '/PS_Conv_XYZ']);
add_block('simulink/Sinks/Out1', [mdl '/chassis_pos_xyz']);
set_param([mdl '/chassis_pos_xyz'], 'Port', '3');
add_line(mdl, 'Chassis_Pose_Sensor/RConn2', 'PS_Conv_XYZ/LConn1');
add_line(mdl, 'PS_Conv_XYZ/1', 'chassis_pos_xyz/1');

add_block('nesl_utility/PS-Simulink Converter', [mdl '/PS_Conv_Rot']);
add_block('simulink/Sinks/Out1', [mdl '/chassis_euler_yxz']);
set_param([mdl '/chassis_euler_yxz'], 'Port', '4');
add_line(mdl, 'Chassis_Pose_Sensor/RConn3', 'PS_Conv_Rot/LConn1');
add_line(mdl, 'PS_Conv_Rot/1', 'chassis_euler_yxz/1');

fprintf('Saving configured model...\n');
save_system(mdl);
close_system(mdl, 0);
fprintf('=== AGV SIMSCAPE MULTIBODY MODEL SETUP COMPLETE! ===\n');
