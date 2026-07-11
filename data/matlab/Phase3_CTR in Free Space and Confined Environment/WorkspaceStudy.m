clear; clc; close all;

%% WorkspaceStudy.m
% Phase 3 Part A: free-space theoretical workspace analysis.
%
% This script samples the simplified 3-tube CTR model from Phase 2,
% stores all sampled q values and tip positions, identifies representative
% most-straight and most-bent configurations using full-backbone metrics,
% and visualizes their backbones with sample_ctr_centerline.m.
%
% Geometry-only note:
% This is a free-space geometric reachability study. It does not model
% contact, friction, tissue deformation, insertion force, or collision.

%% Parameter setting
% q = [rho1 rho2 rho3 theta1 theta2 theta3]
% rho values are entered in mm, theta values are entered in degrees.
%
% In the current Robot.m implementation, rho1 is the translation reference:
% rho_i_internal = (rho_i - rho1) * 1e-3 for i = 2,3. Keeping rho1 fixed
% matches the Phase 2 starter template and avoids redundant equivalent
% samples.
rho1_vals = 0;                  % mm
rho2_vals = 0:20:60;            % mm, 4 samples
rho3_vals = 0:20:80;            % mm, 5 samples
theta1_vals = -90:45:90;        % deg, 5 samples
theta2_vals = -90:45:90;        % deg, 5 samples
theta3_vals = -90:45:90;        % deg, 5 samples

ptsPerLinkBackbone = 40;
workspaceMatFile = 'WorkspaceSamples.mat';

%% CTR model initialization
[robot, ~] = initialize_ctr_model();

%% Main computation: sample free-space tip workspace
nSamples = numel(rho1_vals) * numel(rho2_vals) * numel(rho3_vals) * ...
    numel(theta1_vals) * numel(theta2_vals) * numel(theta3_vals);

q_log = zeros(nSamples, 6);
tip_points = zeros(nSamples, 3);          % m
lateral_offset = zeros(nSamples, 1);      % m
backbone_rms_lateral = zeros(nSamples, 1);% m
backbone_max_lateral = zeros(nSamples, 1);% m
backbone_arc_length = zeros(nSamples, 1); % m
max_link_curvature = zeros(nSamples, 1);  % 1/m

sampleIdx = 0;
for rho1 = rho1_vals
    for rho2 = rho2_vals
        for rho3 = rho3_vals
            for theta1 = theta1_vals
                for theta2 = theta2_vals
                    for theta3 = theta3_vals
                        sampleIdx = sampleIdx + 1;

                        q = [rho1, rho2, rho3, theta1, theta2, theta3];
                        T = robot.fkin(q);
                        p_tip = T(1:3,4).';
                        P_backbone = sample_ctr_centerline(robot, q, ptsPerLinkBackbone);
                        [rms_lateral, max_lateral, arc_length] = ...
                            backbone_lateral_metrics(P_backbone);

                        q_log(sampleIdx,:) = q;
                        tip_points(sampleIdx,:) = p_tip;
                        lateral_offset(sampleIdx) = sqrt(p_tip(1)^2 + p_tip(2)^2);
                        backbone_rms_lateral(sampleIdx) = rms_lateral;
                        backbone_max_lateral(sampleIdx) = max_lateral;
                        backbone_arc_length(sampleIdx) = arc_length;
                        max_link_curvature(sampleIdx) = max(robot.kappa);
                    end
                end
            end
        end
    end
end

% Phase 3 allows several defensible definitions of "straight" and "bent".
% Use full-backbone lateral deviation so a curve that bends away and returns
% near the z-axis is not mislabeled as straight.
[metric_straight, idx_straight] = min(backbone_rms_lateral);
[metric_bent, idx_bent] = max(backbone_max_lateral);

q_straight = q_log(idx_straight,:);
q_bent = q_log(idx_bent,:);
p_straight = tip_points(idx_straight,:);
p_bent = tip_points(idx_bent,:);

P_straight = sample_ctr_centerline(robot, q_straight, ptsPerLinkBackbone);
P_bent = sample_ctr_centerline(robot, q_bent, ptsPerLinkBackbone);

%% Visualization: tip workspace scatter plot
figure('Name','Phase 3 Part A - Free-space tip workspace');
scatter3(tip_points(:,1), tip_points(:,2), tip_points(:,3), ...
    18, lateral_offset, 'filled');
hold on;
plot3(p_straight(1), p_straight(2), p_straight(3), ...
    'ko', 'MarkerSize', 9, 'LineWidth', 1.6);
plot3(p_bent(1), p_bent(2), p_bent(3), ...
    'kp', 'MarkerSize', 12, 'LineWidth', 1.6);
axis equal; grid on; colorbar;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Free-space tip workspace of the simplified 3-tube CTR');
legend('Sampled tips', 'Most straight tip', 'Most bent tip', 'Location', 'best');

%% Visualization: representative backbone plot
figure('Name','Phase 3 Part A - Representative backbones');
plot3(P_straight(:,1), P_straight(:,2), P_straight(:,3), ...
    'b-', 'LineWidth', 2.2);
hold on;
plot3(P_bent(:,1), P_bent(:,2), P_bent(:,3), ...
    'r-', 'LineWidth', 2.2);
plot3(P_straight(end,1), P_straight(end,2), P_straight(end,3), ...
    'bo', 'MarkerFaceColor', 'b');
plot3(P_bent(end,1), P_bent(end,2), P_bent(end,3), ...
    'ro', 'MarkerFaceColor', 'r');
axis equal; grid on;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('Representative CTR backbones in free space');
legend('Most straight backbone', 'Most bent backbone', ...
    'Most straight tip', 'Most bent tip', 'Location', 'best');

%% Summary output
summaryTable = table( ...
    ["Most straight"; "Most bent"], ...
    [q_straight(1); q_bent(1)], ...
    [q_straight(2); q_bent(2)], ...
    [q_straight(3); q_bent(3)], ...
    [q_straight(4); q_bent(4)], ...
    [q_straight(5); q_bent(5)], ...
    [q_straight(6); q_bent(6)], ...
    [metric_straight; backbone_rms_lateral(idx_bent)], ...
    [backbone_max_lateral(idx_straight); metric_bent], ...
    [lateral_offset(idx_straight); lateral_offset(idx_bent)], ...
    [backbone_arc_length(idx_straight); backbone_arc_length(idx_bent)], ...
    [max_link_curvature(idx_straight); max_link_curvature(idx_bent)], ...
    [p_straight(1); p_bent(1)], ...
    [p_straight(2); p_bent(2)], ...
    [p_straight(3); p_bent(3)], ...
    'VariableNames', {'Case','rho1_mm','rho2_mm','rho3_mm', ...
    'theta1_deg','theta2_deg','theta3_deg', ...
    'backbone_rms_lateral_m','backbone_max_lateral_m', ...
    'tip_lateral_offset_m','backbone_arc_length_m', ...
    'max_link_curvature_1_per_m', ...
    'tip_x_m','tip_y_m','tip_z_m'});

workspaceData = struct();
workspaceData.q_log = q_log;
workspaceData.tip_points_m = tip_points;
workspaceData.lateral_offset_m = lateral_offset;
workspaceData.backbone_rms_lateral_m = backbone_rms_lateral;
workspaceData.backbone_max_lateral_m = backbone_max_lateral;
workspaceData.backbone_arc_length_m = backbone_arc_length;
workspaceData.max_link_curvature_1_per_m = max_link_curvature;
workspaceData.q_straight = q_straight;
workspaceData.q_bent = q_bent;
workspaceData.P_straight_m = P_straight;
workspaceData.P_bent_m = P_bent;
workspaceData.summaryTable = summaryTable;
save(workspaceMatFile, 'workspaceData');

fprintf('\nPhase 3 Part A workspace sampling complete.\n');
fprintf('Number of sampled configurations: %d\n', nSamples);
fprintf('Saved sampled data to %s\n\n', workspaceMatFile);
disp('Summary table:');
disp(summaryTable);

%% Local helper functions
function [robot, tubes] = initialize_ctr_model()
    % Same tube parameter set as Phase 2 / WorkspaceSamplingTemplate.m.
    tube1 = Tube(3.046e-3, 3.3e-3, 1/17,  90e-3, 50e-3, 1935e6);
    tube2 = Tube(2.386e-3, 2.64e-3, 1/22, 170e-3, 50e-3, 1935e6);
    tube3 = Tube(1.726e-3, 1.98e-3, 1/29, 250e-3, 50e-3, 1935e6);

    tubes = [tube1, tube2, tube3];
    robot = Robot(tubes, false);
end

function [rms_lateral, max_lateral, arc_length] = backbone_lateral_metrics(P)
    lateral = hypot(P(:,1), P(:,2));
    max_lateral = max(lateral);

    if size(P,1) < 2
        rms_lateral = lateral(1);
        arc_length = 0;
        return;
    end

    ds = vecnorm(diff(P, 1, 1), 2, 2);
    arc_length = sum(ds);

    if arc_length <= 0
        rms_lateral = lateral(1);
        return;
    end

    lateral_mid = 0.5 * (lateral(1:end-1) + lateral(2:end));
    rms_lateral = sqrt(sum((lateral_mid.^2) .* ds) / arc_length);
end
