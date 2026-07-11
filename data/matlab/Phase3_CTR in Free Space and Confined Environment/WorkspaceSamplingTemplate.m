clc; clear; close all;

%% WORKSPACESAMPLINGTEMPLATE
% ------------------------------------------------------------
% Purpose:
%   Sample the simplified 3-tube CTR model in free space and visualize the
%   tip workspace as a 3D point cloud.
%
% What this script already does:
%   - defines the tube parameters
%   - constructs the Robot object
%   - samples a recommended subset of the joint space
%   - stores all tip positions
%   - plots the resulting free-space tip workspace
%
% What You need to do here:
%   - define and justify "most straight", "most bent",
%     "farthest tip", and "closest tip"
%   - identify representative configurations
%   - visualize representative CTR backbones using sample_ctr_centerline()
%
% Notes:
%   - This file is a framework, not a complete solution.
%   - You should clearly report any changes you make to the
%     sampling ranges below.
% ------------------------------------------------------------

%% -----------------------------------------------------------
% Define tubes (same parameter set as in perform_kinematics.m)
%% -----------------------------------------------------------
tube1 = Tube(3.046e-3, 3.3e-3, 1/17,  90e-3, 50e-3, 1935e6);
tube2 = Tube(2.386e-3, 2.64e-3, 1/22, 170e-3, 50e-3, 1935e6);
tube3 = Tube(1.726e-3, 1.98e-3, 1/29, 250e-3, 50e-3, 1935e6);

tubes = [tube1, tube2, tube3];
robot = Robot(tubes, false);

%% -----------------------------------------------------------
% Recommended joint sampling ranges
% You may adjust these ranges if clearly documented.
%% -----------------------------------------------------------
rho1_vals = 0;                  % keep benchmark-compatible reference
rho2_vals = [0, 20, 40, 60];
rho3_vals = [0, 20, 40, 60, 80];

th1_vals = [-90, -45, 0, 45, 90];
th2_vals = [-90, -45, 0, 45, 90];
th3_vals = [-90, -45, 0, 45, 90];

tip_points = [];
q_log = [];

%% -----------------------------------------------------------
% Sample joint space and collect tip positions
%% -----------------------------------------------------------
for rho1 = rho1_vals
    for rho2 = rho2_vals
        for rho3 = rho3_vals
            for th1 = th1_vals
                for th2 = th2_vals
                    for th3 = th3_vals

                        q_var = [rho1, rho2, rho3, th1, th2, th3];

                        T = robot.fkin(q_var);
                        p_tip = T(1:3,4).';

                        tip_points(end+1,:) = p_tip;
                        q_log(end+1,:) = q_var;
                    end
                end
            end
        end
    end
end

%% -----------------------------------------------------------
% Plot free-space tip workspace
%% -----------------------------------------------------------
figure;
scatter3(tip_points(:,1), tip_points(:,2), tip_points(:,3), 18, 'filled');
axis equal; grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('Free-space tip workspace of the simplified 3-tube CTR');

%% -----------------------------------------------------------
% Save sampled data for later analysis
%% -----------------------------------------------------------
workspaceData = struct();
workspaceData.tip_points = tip_points;
workspaceData.q_log = q_log;

save('WorkspaceSamples.mat', 'workspaceData');

disp('Workspace sampling completed.');
disp(['Number of sampled configurations = ', num2str(size(q_log,1))]);

%% -----------------------------------------------------------
% YOUR TASKS FROM THIS POINT FORWARD:
% 1. Define "most straight", "most bent".
% 2. Select representative q values from q_log.
% 3. Use sample_ctr_centerline(robot, q_var, ptsPerLink) to visualize
%    the corresponding backbones.
%% -----------------------------------------------------------