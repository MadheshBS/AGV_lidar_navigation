function nextPose = diffdrive_odometry(currentPose, d_sL, d_sR, trackWidth)
% DIFFDRIVE_ODOMETRY Dead-reckoning pose integration for differential drive.
%
% Inputs:
%   currentPose - 1x3 or 3x1 vector [x, y, theta] (m, m, rad)
%   d_sL        - Distance traveled by left wheel over time step (m)
%                 OR left wheel velocity (m/s) if dt is implicit in step
%   d_sR        - Distance traveled by right wheel over time step (m)
%   trackWidth  - (Optional) Effective distance between drive wheels (m)
%
% Output:
%   nextPose    - 1x3 vector [x_new, y_new, theta_new] (m, m, rad)
%
% Integration Method:
%   Mid-point / 2nd-order Runge-Kutta arc integration:
%   delta_s     = (d_sR + d_sL) / 2
%   delta_theta = (d_sR - d_sL) / trackWidth
%   theta_mid   = theta + delta_theta / 2
%   x_new       = x + delta_s * cos(theta_mid)
%   y_new       = y + delta_s * sin(theta_mid)
%   theta_new   = atan2(sin(theta + delta_theta), cos(theta + delta_theta))

    if nargin < 4 || isempty(trackWidth)
        p = agv_params();
        trackWidth = p.trackWidth;
    end

    x = currentPose(1);
    y = currentPose(2);
    theta = currentPose(3);

    % Incremental displacements
    delta_s = (d_sR + d_sL) / 2.0;
    delta_theta = (d_sR - d_sL) / trackWidth;

    % Mid-point integration for higher numerical accuracy
    theta_mid = theta + (delta_theta / 2.0);

    x_new = x + delta_s * cos(theta_mid);
    y_new = y + delta_s * sin(theta_mid);
    theta_new = atan2(sin(theta + delta_theta), cos(theta + delta_theta));

    nextPose = [x_new, y_new, theta_new];
end
