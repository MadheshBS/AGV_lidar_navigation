function [v, omega] = diffdrive_forward_kinematics(vL, vR, trackWidth)
% DIFFDRIVE_FORWARD_KINEMATICS Computes linear and angular velocity
% from left and right wheel linear velocities.
%
% Inputs:
%   vL         - Left wheel linear velocity (m/s)
%   vR         - Right wheel linear velocity (m/s)
%   trackWidth - Distance between the two drive wheels (m)
%
% Outputs:
%   v          - Vehicle body linear velocity (m/s)
%   omega      - Vehicle body angular velocity (rad/s)
%
% Formulae:
%   v = (vR + vL) / 2
%   omega = (vR - vL) / trackWidth

    if nargin < 3
        p = agv_params();
        trackWidth = p.trackWidth;
    end

    v = (vR + vL) / 2;
    omega = (vR - vL) / trackWidth;
end
