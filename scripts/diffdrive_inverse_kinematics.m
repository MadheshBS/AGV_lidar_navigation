function [vL, vR, wL, wR] = diffdrive_inverse_kinematics(v, omega, trackWidth, wheelRadius)
% DIFFDRIVE_INVERSE_KINEMATICS Computes wheel velocities from desired (v, omega).
%
% Inputs:
%   v           - Desired linear velocity along robot x-axis (m/s)
%   omega       - Desired angular velocity about robot z-axis (rad/s)
%   trackWidth  - (Optional) Distance between drive wheels (m)
%   wheelRadius - (Optional) Drive wheel radius (m)
%
% Outputs:
%   vL          - Left wheel linear velocity (m/s)
%   vR          - Right wheel linear velocity (m/s)
%   wL          - Left wheel angular velocity (rad/s)
%   wR          - Right wheel angular velocity (rad/s)
%
% Formulae:
%   vL = v - (omega * trackWidth / 2)
%   vR = v + (omega * trackWidth / 2)
%   wL = vL / wheelRadius
%   wR = vR / wheelRadius

    if nargin < 4
        p = agv_params();
        if nargin < 3 || isempty(trackWidth)
            trackWidth = p.trackWidth;
        end
        wheelRadius = p.wheelRadius;
    end

    vL = v - (omega * trackWidth / 2.0);
    vR = v + (omega * trackWidth / 2.0);

    wL = vL / wheelRadius;
    wR = vR / wheelRadius;
end
