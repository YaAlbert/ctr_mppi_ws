classdef Drive < handle
    % Drive class for CTR hardware control through the Octopus board.
    %
    % This revised version preserves the original position-based interface,
    % but also adds a pseudo-PMC streaming method that sends many small
    % absolute G-code moves at a fixed refresh rate.

    properties
        % The COM port assigned by the PC to the Octopus board.
        COMPort = "COM13";
        BaudRate = 250000;
        ser;

        % Home and current poses are tracked on the MATLAB side.
        homePose = Pose(0,0,0,0,0,0)
        currPose = Pose(0,0,0,0,0,0)

        % Conversion used by Pose.m for linear axes.
        linearScale = 16;

        % Safety / protocol settings.
        minFeedrateCmdUnitsPerMin = 120;
    end

    methods
        function self = Drive(currPose, comPort, baudRate)
            if nargin < 1 || isempty(currPose)
                currPose = Pose(0,0,0,0,0,0);
            end
            if nargin >= 2 && ~isempty(comPort)
                self.COMPort = string(comPort);
            end
            if nargin >= 3 && ~isempty(baudRate)
                self.BaudRate = baudRate;
            end

            self.currPose = currPose;

            % serialport opens immediately in modern MATLAB.
            self.ser = serialport(self.COMPort, self.BaudRate);
            configureTerminator(self.ser, "LF");
            flush(self.ser);
            pause(2.0); % Give firmware time to finish reset and USB enumeration.

            % Force absolute positioning and initialize the controller-side pose.
            self.send_command("G90");
            pause(0.05);
            self.set_current_pose(currPose);
        end

        function delete(self)
            try
                if ~isempty(self.ser)
                    delete(self.ser);
                end
            catch
            end
        end

        function set_current_pose(self, pose)
            self.currPose = pose;
            command = "G92 " + pose.get_gcode_for_pose();
            self.send_command(command);
        end

        function set_current_pose_as_home(self)
            command = "G92 " + self.homePose.get_gcode_for_pose();
            self.send_command(command);
        end

        function set_home_as_pose(self, pose)
            self.set_current_pose(self.homePose);
            invPose = Pose( ...
                self.currPose.lin1 - pose.lin1, ...
                self.currPose.lin2 - pose.lin2, ...
                self.currPose.lin3 - pose.lin3, ...
                self.currPose.rot1 - pose.rot1, ...
                self.currPose.rot2 - pose.rot2, ...
                self.currPose.rot3 - pose.rot3);
            self.set_current_pose(invPose);
        end

        function travel_for(self, lin1, lin2, lin3, rot1, rot2, rot3)
            self.currPose = Pose( ...
                self.currPose.lin1 + lin1, ...
                self.currPose.lin2 + lin2, ...
                self.currPose.lin3 + lin3, ...
                self.currPose.rot1 + rot1, ...
                self.currPose.rot2 + rot2, ...
                self.currPose.rot3 + rot3);

            command = "G0 " + self.currPose.get_gcode_for_pose();
            self.send_command(command);
        end

        function travel_to(self, lin1, lin2, lin3, rot1, rot2, rot3)
            self.currPose = Pose(lin1, lin2, lin3, rot1, rot2, rot3);
            command = "G0 " + self.currPose.get_gcode_for_pose();
            self.send_command(command);
        end

        function stream_velocity(self, linVel, rotVel, dt)
            % Pseudo-PMC streaming:
            % integrate velocity for one control period, then send a tiny
            % absolute G1 move with an approximate feedrate.
            %
            % linVel : [v1 v2 v3] in translation units / s
            % rotVel : [w1 w2 w3] in rotation units / s
            % dt     : control period in s

            if nargin < 4 || dt <= 0
                error('dt must be positive.');
            end
            if numel(linVel) ~= 3 || numel(rotVel) ~= 3
                error('linVel and rotVel must each have 3 elements.');
            end

            linVel = double(linVel(:)).';
            rotVel = double(rotVel(:)).';

            if all(abs([linVel rotVel]) < eps)
                return;
            end

            dLin = linVel * dt;
            dRot = rotVel * dt;

            self.currPose = Pose( ...
                self.currPose.lin1 + dLin(1), ...
                self.currPose.lin2 + dLin(2), ...
                self.currPose.lin3 + dLin(3), ...
                self.currPose.rot1 + dRot(1), ...
                self.currPose.rot2 + dRot(2), ...
                self.currPose.rot3 + dRot(3));

            % Convert commanded increments to the same command-space units
            % used by the G-code string, then estimate a feedrate.
            dCmd = [dLin * self.linearScale, dRot];
            cmdSpeed = max(abs(dCmd)) / dt; % command-units / second
            feedrate = max(self.minFeedrateCmdUnitsPerMin, cmdSpeed * 60);

            command = "G1 " + self.currPose.get_gcode_for_pose() + ...
                " F" + num2str(feedrate, '%.3f');
            self.send_command(command);
        end

        function send_command(self, command)
            if isempty(self.ser)
                error('Serial port is not open.');
            end

            command = string(command);
            command = strip(command);
            writeline(self.ser, command);
        end
    end
end
