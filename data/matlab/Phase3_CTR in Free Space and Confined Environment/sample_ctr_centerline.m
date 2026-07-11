function Pctr = sample_ctr_centerline(robot, q_var, ptsPerLink)
% SAMPLE_CTR_CENTERLINE
% ------------------------------------------------------------
% Purpose:
%   Generate the full CTR backbone centerline points using the simplified
%   PCC-based CTR model already implemented in Robot.m.
%
% Inputs:
%   robot      - Robot object
%   q_var      - 1x6 joint vector [rho1 rho2 rho3 theta1 theta2 theta3]
%   ptsPerLink - number of sample points per link (default: 30)
%
% Output:
%   Pctr       - Nx3 array of 3D points along the CTR centerline
%
% Notes:
%   - This function first calls robot.fkin(q_var) so that the internal
%     fields robot.lls, robot.phi, robot.kappa are updated.
%   - This function is a geometry utility. You should use it for
%     backbone sampling and visualization.
% ------------------------------------------------------------

    if nargin < 3 || isempty(ptsPerLink)
        ptsPerLink = 30;
    end

    % Update robot internal quantities
    robot.fkin(q_var);

    s = robot.lls;
    phi = robot.phi;
    K = robot.kappa;

    T_cum = eye(4);
    nLinks = length(s);
    if nLinks == 0
        Pctr = zeros(0, 3);
        return;
    end

    nPoints = nLinks * ptsPerLink - (nLinks - 1);
    Pctr = zeros(nPoints, 3);
    pointIdx = 0;

    prev_phi = 0;
    tol = 1e-12;

    for j = 1:nLinks
        arc_length = s(j);
        curvature = K(j);

        % Relative bend-plane change between adjacent curved links.
        % Straight links have no bend plane and do not update prev_phi.
        delta_phi = phi(j) - prev_phi;

        % Local sampling parameter
        u_samples = linspace(0, arc_length, ptsPerLink);

        % Avoid duplicate points at link boundaries
        if j > 1
            u_samples = u_samples(2:end);
        end

        P_local = zeros(length(u_samples), 3);

        if abs(curvature) < tol
            % Straight link
            for k = 1:length(u_samples)
                u = u_samples(k);
                P_local(k,:) = [0, 0, u];
            end
        else
            % Constant-curvature link
            Rz = [cos(delta_phi), -sin(delta_phi), 0;
                  sin(delta_phi),  cos(delta_phi), 0;
                               0,               0, 1];

            for k = 1:length(u_samples)
                u = u_samples(k);
                bend_angle = curvature * u;

                x_local = (1 - cos(bend_angle)) / curvature;
                y_local = 0;
                z_local = sin(bend_angle) / curvature;

                p_rot = Rz * [x_local; y_local; z_local];
                P_local(k,:) = p_rot.';
            end
        end

        % Transform sampled local points to global frame
        for k = 1:size(P_local,1)
            p_h = [P_local(k,:), 1].';
            p_global = T_cum * p_h;
            pointIdx = pointIdx + 1;
            Pctr(pointIdx,:) = p_global(1:3).';
        end

        % Advance cumulative transform by one full link
        T_link = build_single_link_transform(arc_length, curvature, delta_phi);
        T_cum = T_cum * T_link;

        if abs(curvature) >= tol
            prev_phi = phi(j);
        end
    end

    Pctr = Pctr(1:pointIdx,:);
end
