function T_link = build_single_link_transform(arc_length, curvature, delta_phi)
% BUILD_SINGLE_LINK_TRANSFORM
% ------------------------------------------------------------
% Purpose:
%   Build the homogeneous transformation matrix for one constant-curvature
%   link using the same simplified PCC model as in Robot.m.
%
% Inputs:
%   arc_length - link length
%   curvature  - link curvature
%   delta_phi  - relative bend-plane change for this link
%
% Output:
%   T_link     - 4x4 homogeneous transformation matrix
%
% Notes:
%   - This file is primarily used by sample_ctr_centerline.m.
%   - You do not need to modify this file.
% ------------------------------------------------------------

    tol = 1e-12;

    if abs(curvature) < tol
        T_link = eye(4);
        T_link(3,4) = arc_length;
        return;
    end

    bend_angle = curvature * arc_length;

    c_phi = cos(delta_phi);
    s_phi = sin(delta_phi);
    c_theta = cos(bend_angle);
    s_theta = sin(bend_angle);

    T_link = [ ...
        c_phi * c_theta,   -s_phi,   c_phi * s_theta,   c_phi * (1 - c_theta) / curvature; ...
        s_phi * c_theta,    c_phi,   s_phi * s_theta,   s_phi * (1 - c_theta) / curvature; ...
        -s_theta,              0,              c_theta,               s_theta / curvature; ...
        0,                     0,                    0,                                1];
end