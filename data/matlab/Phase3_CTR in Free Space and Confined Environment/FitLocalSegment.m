clear; clc; close all;

%% FitLocalSegment.m
% Phase 3 Part C: local vessel segment fitting near the maximum-curvature
% region.
%
% Geometry-only note:
% The best-fit result is only a simplified PCC CTR backbone fit to a local
% centerline segment. It does not prove that a real robot can pass through
% the vessel because contact, friction, tissue deformation, true insertion
% force, vessel wall collision, and clinical constraints are not modeled.

%% Parameter setting
centerlineFile = find_project_file({'*Centerline*curve*.fcsv', '*centerline*curve*.fcsv'});

numCenterlineSamples = 300;
smoothingWindow = 15;
maxCurvatureSearchMargin = 12;
showAnalyzeCenterlinePlots = false;

% The Phase 2 tube parameter set uses d = 50 mm for each curved section.
% A 50 mm vessel segment therefore gives a local target length comparable
% to the effective precurved bending length in the simplified model.
segmentLength_mm = 50;
N = 80;                         % common resampling count for vessel and CTR
ptsPerLinkBackbone = 45;

% Coarse grid search. Rho is entered in mm, theta in degrees.
rho1_vals = 0;                  % reference translation in current Robot.m
rho2_vals = 0:20:60;
rho3_vals = 0:20:80;
theta1_vals = -180:60:180;
theta2_vals = -180:60:180;
theta3_vals = -180:60:180;

resultsMatFile = 'FitLocalSegmentResults.mat';

%% CTR model initialization
[robot, ~] = initialize_ctr_model();

%% Main computation: prepare local vessel segment
centerline = AnalyzeCenterline(centerlineFile, numCenterlineSamples, ...
    smoothingWindow, maxCurvatureSearchMargin, showAnalyzeCenterlinePlots);

[Pseg_raw, idx_start, idx_end] = ExtractLocalSegment(centerline.P_smooth, ...
    centerline.s_uniform, centerline.idx_max, segmentLength_mm);

s_seg_raw = centerline.s_uniform(idx_start:idx_end);
s_seg_local = s_seg_raw - s_seg_raw(1);
s_fit = linspace(0, s_seg_local(end), N).';

P_vessel = resample_curve(Pseg_raw, s_seg_local, s_fit);   % mm
vessel_tangent0 = initial_tangent(P_vessel);

%% Main computation: coarse grid search over CTR configurations
nSamples = numel(rho1_vals) * numel(rho2_vals) * numel(rho3_vals) * ...
    numel(theta1_vals) * numel(theta2_vals) * numel(theta3_vals);

searchLog = zeros(nSamples, 9);
best = struct();
best.J = inf;
best.q = nan(1,6);
best.meanError = inf;
best.maxError = inf;
best.pointErrors = [];
best.P_ctr_local = [];
best.P_ctr_aligned = [];

sampleIdx = 0;
for rho1 = rho1_vals
    for rho2 = rho2_vals
        for rho3 = rho3_vals
            for theta1 = theta1_vals
                for theta2 = theta2_vals
                    for theta3 = theta3_vals
                        sampleIdx = sampleIdx + 1;

                        q = [rho1, rho2, rho3, theta1, theta2, theta3];
                        [J, P_ctr_local, P_ctr_aligned, pointErrors] = ...
                            fitting_cost(robot, q, P_vessel, s_fit, ...
                            vessel_tangent0, ptsPerLinkBackbone);

                        meanError = mean(pointErrors);
                        maxError = max(pointErrors);
                        searchLog(sampleIdx,:) = [q, J, meanError, maxError];

                        if J < best.J
                            best.J = J;
                            best.q = q;
                            best.meanError = meanError;
                            best.maxError = maxError;
                            best.pointErrors = pointErrors;
                            best.P_ctr_local = P_ctr_local;
                            best.P_ctr_aligned = P_ctr_aligned;
                        end
                    end
                end
            end
        end
    end
end

%% Visualization: overlay local vessel segment and best-fit CTR backbone
figure('Name','Phase 3 Part C - Local segment fit');
plot3(P_vessel(:,1), P_vessel(:,2), P_vessel(:,3), ...
    'b-', 'LineWidth', 2.2);
hold on;
plot3(best.P_ctr_aligned(:,1), best.P_ctr_aligned(:,2), best.P_ctr_aligned(:,3), ...
    'r--', 'LineWidth', 2.2);
plot3(P_vessel(1,1), P_vessel(1,2), P_vessel(1,3), ...
    'ko', 'MarkerFaceColor', 'k', 'MarkerSize', 6);
axis equal; grid on;
xlabel('X (mm)');
ylabel('Y (mm)');
zlabel('Z (mm)');
title('Local vessel segment and best-fit CTR backbone');
legend('Vessel local segment', 'Best-fit CTR backbone', ...
    'Shared start point', 'Location', 'best');

%% Visualization: pointwise fitting error
figure('Name','Phase 3 Part C - Pointwise fitting error');
plot(s_fit, best.pointErrors, 'LineWidth', 1.6);
grid on;
xlabel('Local arc length (mm)');
ylabel('Pointwise error (mm)');
title('Best-fit pointwise error along local segment');

%% Summary output
bestFitTable = table(best.q(1), best.q(2), best.q(3), ...
    best.q(4), best.q(5), best.q(6), best.meanError, ...
    best.maxError, best.J, segmentLength_mm, centerline.idx_max, ...
    idx_start, idx_end, ...
    'VariableNames', {'rho1_mm','rho2_mm','rho3_mm', ...
    'theta1_deg','theta2_deg','theta3_deg','mean_error_mm', ...
    'max_error_mm','J_best_mm2','segment_length_mm', ...
    'idx_max_curvature','idx_segment_start','idx_segment_end'});

fitResults = struct();
fitResults.centerlineFile = centerlineFile;
fitResults.segmentLength_mm = segmentLength_mm;
fitResults.N = N;
fitResults.q_best = best.q;
fitResults.mean_error_mm = best.meanError;
fitResults.max_error_mm = best.maxError;
fitResults.J_best_mm2 = best.J;
fitResults.P_vessel_mm = P_vessel;
fitResults.P_ctr_local_mm = best.P_ctr_local;
fitResults.P_ctr_aligned_mm = best.P_ctr_aligned;
fitResults.pointErrors_mm = best.pointErrors;
fitResults.searchLog = searchLog;
fitResults.bestFitTable = bestFitTable;
fitResults.centerline = centerline;
save(resultsMatFile, 'fitResults');

fprintf('\nPhase 3 Part C local segment fitting complete.\n');
fprintf('Centerline file: %s\n', centerlineFile);
fprintf('Number of sampled CTR configurations: %d\n', nSamples);
fprintf('Saved results to %s\n\n', resultsMatFile);
disp('Best-fit summary table:');
disp(bestFitTable);
disp('Important limitation: this is a simplified geometric centerline fit only, not proof of real vessel traversal.');

%% Local helper functions
function [J, P_ctr_local_mm, P_ctr_aligned_mm, pointErrors] = fitting_cost( ...
    robot, q, P_vessel_mm, s_fit_mm, vessel_tangent0, ptsPerLinkBackbone)
    % Generate the CTR backbone in meters, then convert to mm for vessel
    % comparison.
    P_ctr_mm = sample_ctr_centerline(robot, q, ptsPerLinkBackbone) * 1000;
    s_ctr_mm = [0; cumsum(vecnorm(diff(P_ctr_mm, 1, 1), 2, 2))];

    if s_ctr_mm(end) < s_fit_mm(end)
        % This should rarely happen for the current Phase 2 parameters.
        % Penalize short candidates while still returning a comparable curve.
        s_query = linspace(0, s_ctr_mm(end), numel(s_fit_mm)).';
        lengthPenalty = (s_fit_mm(end) - s_ctr_mm(end))^2;
    else
        s_query = s_fit_mm;
        lengthPenalty = 0;
    end

    % Take the CTR local backbone with the same arc-length support as the
    % vessel segment and resample both with the same N points.
    P_ctr_local_mm = resample_curve(P_ctr_mm, s_ctr_mm, s_query);

    ctr_tangent0 = initial_tangent(P_ctr_local_mm);
    R = rotation_from_vectors(ctr_tangent0, vessel_tangent0);

    % Align initial tangent, then translate the CTR start point onto the
    % vessel segment start point.
    P_ctr_zero = P_ctr_local_mm - P_ctr_local_mm(1,:);
    P_ctr_aligned_mm = (R * P_ctr_zero.').';
    P_ctr_aligned_mm = P_ctr_aligned_mm + P_vessel_mm(1,:);

    pointErrors = vecnorm(P_ctr_aligned_mm - P_vessel_mm, 2, 2);
    J = mean(pointErrors.^2) + lengthPenalty;
end

function tangent = initial_tangent(P)
    tangent = P(2,:) - P(1,:);
    if norm(tangent) < 1e-12 && size(P,1) > 2
        tangent = P(3,:) - P(1,:);
    end
    if norm(tangent) < 1e-12
        error('FitLocalSegment:DegenerateTangent', ...
            'Cannot compute a valid initial tangent.');
    end
end

function Pq = resample_curve(P, s, sq)
    [s_unique, ia] = unique(s, 'stable');
    P_unique = P(ia,:);

    Pq = [ ...
        interp1(s_unique, P_unique(:,1), sq, 'linear', 'extrap'), ...
        interp1(s_unique, P_unique(:,2), sq, 'linear', 'extrap'), ...
        interp1(s_unique, P_unique(:,3), sq, 'linear', 'extrap')];
end

function [robot, tubes] = initialize_ctr_model()
    % Same tube parameter set as Phase 2 / WorkspaceSamplingTemplate.m.
    tube1 = Tube(3.046e-3, 3.3e-3, 1/17,  90e-3, 50e-3, 1935e6);
    tube2 = Tube(2.386e-3, 2.64e-3, 1/22, 170e-3, 50e-3, 1935e6);
    tube3 = Tube(1.726e-3, 1.98e-3, 1/29, 250e-3, 50e-3, 1935e6);

    tubes = [tube1, tube2, tube3];
    robot = Robot(tubes, false);
end

function file = find_project_file(patterns)
    % Locate the Slicer centerline file without hard-coding spaces or suffixes.
    for i = 1:numel(patterns)
        hits = dir(patterns{i});
        hits = hits(~[hits.isdir]);
        if ~isempty(hits)
            [~, order] = max([hits.bytes]);
            file = hits(order).name;
            return;
        end
    end

    error('FitLocalSegment:MissingCenterline', ...
        'Could not find a vessel centerline .fcsv file.');
end
