function [ranges, angles, scanPtsGlobal, sensorPose] = virtual_lidar(robotPose, worldMap, p)
% VIRTUAL_LIDAR Simulates a 2D planar range sensor on an occupancy map.
%
% Deliverable for Phase 5.4:
%   Simulates 360-degree, 1-degree resolution range measurements against the
%   ground-truth environment map (including unexpected obstacles), accounting
%   for sensor mount offset, beam limits, and Gaussian measurement noise.
%
% Inputs:
%   robotPose      - 1x3 vector [x, y, theta] representing AGV base frame (m, m, rad)
%   worldMap       - binaryOccupancyMap of the true environment
%   p              - (Optional) Parameter struct from agv_params()
%
% Outputs:
%   ranges         - N x 1 vector of measured distances (m) [with Gaussian noise]
%   angles         - N x 1 vector of ray angles relative to sensor frame (rad)
%   scanPtsGlobal  - N x 2 matrix of hit coordinates in world frame [x, y] (m)
%   sensorPose     - 1x3 vector [xs, ys, thetas] of lidar in world frame (m, m, rad)
%
% Reference:
%   Project README Section 5.4 (Phase 5)

    if nargin < 3 || isempty(p)
        p = agv_params();
    end

    rx = robotPose(1);
    ry = robotPose(2);
    rtheta = robotPose(3);

    %% 1. Transform Sensor Mount Pose to World Frame
    % Mount offset relative to robot body: [dx, dy, dtheta]
    dx     = p.lidar.mountPose(1);
    dy     = p.lidar.mountPose(2);
    dtheta = p.lidar.mountPose(3);

    xs = rx + dx * cos(rtheta) - dy * sin(rtheta);
    ys = ry + dx * sin(rtheta) + dy * cos(rtheta);
    thetas = atan2(sin(rtheta + dtheta), cos(rtheta + dtheta));
    sensorPose = [xs, ys, thetas];

    %% 2. Generate Scan Beam Angles
    % 360 beams spanning [-pi, pi) with 1 degree (pi/180 rad) resolution
    numBeams = p.lidar.numBeams;
    dAngle   = (2 * pi) / numBeams;
    angles   = (-pi : dAngle : (pi - dAngle))';

    %% 3. Ray-Tracing Against the True World Map
    % rayIntersection returns N-by-2 intersection coordinates
    maxRange = p.lidar.maxRange;
    minRange = p.lidar.minRange;
    rawPts = rayIntersection(worldMap, sensorPose, angles, maxRange);

    %% 4. Compute Distance & Apply Realistic Measurement Noise
    % Range to each hit point
    dx_hit = rawPts(:, 1) - xs;
    dy_hit = rawPts(:, 2) - ys;
    rawRanges = hypot(dx_hit, dy_hit);

    % Rays with valid hits within maxRange
    validHits = ~isnan(rawRanges) & (rawRanges <= maxRange) & (rawRanges >= minRange);

    % Apply Gaussian noise: d_meas = d_true + N(0, sigma^2)
    noise = randn(numBeams, 1) * p.lidar.noiseStd;
    ranges = rawRanges;
    ranges(validHits) = rawRanges(validHits) + noise(validHits);

    % Clip to sensor operating limits
    ranges(validHits) = max(minRange, min(maxRange, ranges(validHits)));

    % Where no obstacle was struck, return maxRange (or NaN)
    ranges(~validHits) = NaN;

    %% 5. Recompute Global Hit Coordinates from Noisy Ranges
    scanPtsGlobal = NaN(numBeams, 2);
    rayGlobalAngles = thetas + angles;
    scanPtsGlobal(validHits, 1) = xs + ranges(validHits) .* cos(rayGlobalAngles(validHits));
    scanPtsGlobal(validHits, 2) = ys + ranges(validHits) .* sin(rayGlobalAngles(validHits));
end
