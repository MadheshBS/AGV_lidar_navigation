function p = agv_params()
% AGV_PARAMS Central parameter configuration for differential-drive AGV
%
% All dimensions and physical defaults align with the CAD model imported from
% SolidWorks / Simscape Multibody (AGV Frame) and the project specification.
%
% Usage:
%   p = agv_params();

    %% 1. Mechanical / Chassis Geometry (CAD Extracted)
    % Left and right motor wheels are separated along Z-axis in CAD by 334 mm.
    p.trackWidth   = 0.334;  % Wheel separation distance (m) [CAD: 334 mm]
    p.wheelRadius  = 0.108;  % Wheel radius (m) [CAD: ~108 mm radius]
    p.robotRadius  = 0.350;  % Collision footprint radius for planning/inflation (m)
    p.chassisMass  = 5.888;  % Chassis mass (kg) [from Simscape data]
    p.totalMass    = 9.73;   % Total vehicle mass approx (kg)

    %% 2. Motion and Velocity Limits (Industry Standard Conventions)
    p.maxLinearSpeed   = 1.0;   % Maximum forward linear velocity (m/s)
    p.maxAngularSpeed  = 1.5;   % Maximum angular velocity (rad/s)
    p.maxLinearAccel   = 0.8;   % Max linear acceleration (m/s^2)
    p.maxAngularAccel  = 2.0;   % Max angular acceleration (rad/s^2)

    %% 3. Simulation & Timing
    p.dt = 0.05;                % Simulation time step (s) -> 20 Hz update rate

    %% 4. Wheel Odometry & Noise Parameters (Simulating Dead Reckoning Drift)
    % In physical differential-drive robots, slight tire pressure differences,
    % mechanical tolerances, and surface slip cause unequal wheel radii and drift.
    p.odom.leftWheelRadiusBias  = 0.9985; % Scale factor error on left wheel (-0.15%)
    p.odom.rightWheelRadiusBias = 1.0015; % Scale factor error on right wheel (+0.15%)
    p.odom.wheelEncoderNoiseStd = 0.0003; % Encoder quantization noise (0.3 mm / tick)
    p.odom.gyroDriftRate        = 0.001;  % Angular drift rate (rad/s) if gyro integrated

    %% 5. Virtual 2D Lidar Sensor Specifications (Phase 5)
    p.lidar.maxRange     = 10.0;          % Maximum sensing range (m)
    p.lidar.minRange     = 0.10;          % Minimum sensing range (m)
    p.lidar.fov          = 2 * pi;        % Field of View: 360 degrees (rad)
    p.lidar.numBeams     = 360;           % Number of range beams (1 deg resolution)
    p.lidar.scanRate     = 10;            % Scan rate (Hz)
    p.lidar.noiseStd     = 0.02;          % Measurement Gaussian noise std dev (m)
    p.lidar.mountPose    = [0.20, 0, 0];  % Mount offset [x, y, theta] relative to robot center (m, m, rad)

    %% 6. Environment Map & Grid Properties
    p.map.width      = 20.0;  % Warehouse map width (m)
    p.map.height     = 15.0;  % Warehouse map height (m)
    p.map.resolution = 20;    % Grid resolution: 20 cells/meter (0.05 m per cell)

end
