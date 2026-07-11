% This function is used to perform the kinematics of CTR, calculating the
% mathematical model.
% Use given physical parameters of tubes and drive inpit q_val to calculate
% the end pose matric
classdef Robot < handle

    properties
        % Save vector of tubes and size of vector
        tubes = []
        num_tubes = 0

        % Save link lengths, phi values, and kappa values vectors (1 x num
        % links)
        lls = []
        phi = []
        kappa = []
        active_tubes = {}
        curved_tubes = {}
        tip_theta = 0
    end

    methods
        % Constructor. This creates an instance of the robot class 
        % Must pass in a vector of tubes
        function self = Robot(tubes, ~)
            self.tubes = tubes;
            self.num_tubes = length(tubes);
        end

        % Here we calculate the kinematics of a full CTR
        % Pass in the raw joint variables
        % Return transformation matrix that goes from home frame to end
        % effector frame
        % See functions below for each step
        function T = fkin(self, q_var)  
            % First we get the rho and theta avlues from q_var
            rho = get_rho_values(self, q_var);
            theta = get_theta(self, q_var);

            % Next, we use rho to get the link lengths
            self.lls = get_links(self, rho);

            % Now we calculate the phi and kappa values
            [self.phi,self.kappa] = calculate_phi_and_kappa(self, theta);

            % Finally we calculate the base to end effector transform
            T = calculate_transform(self, self.lls, self.phi, self.kappa);
        end

        % Function to get rho values from joint positions (get translation in 'm')
        % Return rho (1 x i vector, i is num tubes)
        function rho = get_rho_values(self, q_var)
            if numel(q_var) ~= 2 * self.num_tubes
                error('Robot:InvalidJointVector', ...
                    'Expected q_var = [rho1 rho2 rho3 theta1 theta2 theta3].');
            end

            rho = zeros([1 self.num_tubes]);

            % The Phase 2/3 template keeps rho1 as the insertion reference.
            % Internal rho_i values are curved-section start positions
            % relative to tube 1 and are converted from mm to m.
            for i=2:self.num_tubes
                rho(i) = (q_var(i) - q_var(1)) * 10^-3; 
            end

        end

        % Function to get theta values (get rotation gngles in 'rad')
        % Returns theta (1 x j vector where j is num links)
        function theta = get_theta(self, q_var)
            % Here we extract theta values from the q_var vector

            % Initialize a vector to hold the result
            theta = zeros([1 self.num_tubes]);

            for i=1:self.num_tubes
                theta(i) = deg2rad(q_var(i+self.num_tubes));
            end
        end


        % Function to find the link lengths, in order
        % Returns link lengths (1 x j vector, where j is num of links)
        function s = get_links(self, rho)
            if self.num_tubes ~= 3
                error('This template implementation expects exactly 3 tubes.');
            end

            tol = 1e-12;
            transition_points = 0;

            for i = 1:self.num_tubes
                % Per the Phase 3 guideline, each tube contributes
                % transition points at rho_i and rho_i + d_i. The total
                % tube length l is stored in Tube.m but is not used by this
                % simplified benchmark model.
                transition_points(end + 1) = rho(i); %#ok<AGROW>
                transition_points(end + 1) = rho(i) + self.tubes(i).d; %#ok<AGROW>
            end

            % Only model the exposed robot backbone from the base onward.
            transition_points(abs(transition_points) < tol) = 0;
            transition_points = transition_points(transition_points >= 0);
            transition_points = unique(sort(transition_points));

            s = [];
            self.active_tubes = {};
            self.curved_tubes = {};

            for j = 1:(length(transition_points) - 1)
                link_start = transition_points(j);
                link_end = transition_points(j + 1);
                current_length = link_end - link_start;

                if current_length > tol
                    midpoint = 0.5 * (link_start + link_end);
                    present_tubes = [];
                    curved_tubes_in_link = [];

                    for i = 1:self.num_tubes
                        curved_end = rho(i) + self.tubes(i).d;

                        % A tube is physically present from the common base
                        % to the end of its curved section in this model.
                        if midpoint <= curved_end + tol
                            present_tubes(end + 1) = i; %#ok<AGROW>
                        end

                        if midpoint >= rho(i) - tol && midpoint <= curved_end + tol
                            curved_tubes_in_link(end + 1) = i; %#ok<AGROW>
                        end
                    end

                    if ~isempty(present_tubes)
                        s(end + 1) = current_length; %#ok<AGROW>
                        self.active_tubes{end + 1} = present_tubes;
                        self.curved_tubes{end + 1} = curved_tubes_in_link;
                    end
                end
            end
        end


        % Function to calcualte phi (rotation) and K (curvature) 
        % Should return phi (1 x j vector, where j is num links)
        % and K (1 x j vector)
        function [phi,K] = calculate_phi_and_kappa(self, theta)
            num_links = length(self.lls);
            phi = zeros(1, num_links);
            K = zeros(1, num_links);
            self.tip_theta = theta(end);

            for j = 1:num_links
                present_tubes = self.active_tubes{j};
                curved_tubes_in_link = self.curved_tubes{j};

                stiffness_sum = 0;
                chi = 0;
                gamma = 0;

                for tube_idx = present_tubes  
                    tube = self.tubes(tube_idx);
                    EI = tube.E * tube.I;
                    stiffness_sum = stiffness_sum + EI;
                end

                for tube_idx = curved_tubes_in_link
                    tube = self.tubes(tube_idx);
                    EI = tube.E * tube.I;
                    curvature = tube.k;

                    chi = chi + EI * curvature * cos(theta(tube_idx));
                    gamma = gamma + EI * curvature * sin(theta(tube_idx));
                end

                if stiffness_sum <= 0
                    error('Invalid link stiffness for link %d.', j);
                end

                chi = chi / stiffness_sum;
                gamma = gamma / stiffness_sum;

                K(j) = sqrt(chi^2 + gamma^2);
                phi(j) = atan2(gamma, chi);
            end
        end

        % Take in all robot dependent parameters (lls, phi, kappa) and
        % compelte the robot independent constant curvature kinamtatics
        % Returns a 4x4 transformation matrix from base frame to end
        % effector
        function T = calculate_transform(~, s, phi, K)
            T = eye(4);
            tol = 1e-9;
            previous_phi = 0;

            for j = 1:length(s)
                arc_length = s(j);
                curvature = K(j);
                % phi is computed in the base/material frame; each arc
                % transform is local to the previous link frame.
                bend_plane = phi(j) - previous_phi;

                if abs(curvature) < tol
                    T_link = eye(4);
                    T_link(3,4) = arc_length;
                else
                    bend_angle = curvature * arc_length;

                    c_phi = cos(bend_plane);
                    s_phi = sin(bend_plane);
                    c_theta = cos(bend_angle);
                    s_theta = sin(bend_angle);

                    T_link = [ ...
                        c_phi * c_theta,      -s_phi,  c_phi * s_theta,  c_phi * (1 - c_theta) / curvature; ...
                        s_phi * c_theta,       c_phi,  s_phi * s_theta,  s_phi * (1 - c_theta) / curvature; ...
                        -s_theta,                 0,               c_theta,           s_theta / curvature; ...
                        0,                                0,                              0,                 1];
                end

                T = T * T_link;

                % A straight link has no bend plane, so it should not reset
                % the bend-plane reference used by the next curved link.
                if abs(curvature) >= tol
                    previous_phi = phi(j);
                end
            end

            
        end
    end
end
