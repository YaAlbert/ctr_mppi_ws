classdef CTRControlApp_PseudoPMC < handle
    properties
        %% ===== Robot / Hardware =====
        driveBot = []
        isConnected = false
        robotPort = "COM6"
        robotBaud = 250000

        %% ===== Arduino Sensor =====
        sensorSerial = []
        sensorConnected = false
        sensorPort = 'COM5'
        sensorBaud = 115200
        latestPressure1 = NaN
        latestPressure2 = NaN
        alarmActive = false

        %% ===== Joystick / 3-tube teleop =====
        latestJoyX = [NaN NaN NaN]
        latestJoyY = [NaN NaN NaN]
        latestJoySW = [0 0 0]
        lastJoySW = [0 0 0]
        joyCenterV = 2.5
        joyDeadzoneV = 0.25
        transSpeedLevels = [0.2 1 2]
        rotSpeedLevels = [15 30 45]
        transSpeedIdx = 2
        rotSpeedIdx = 2
        estopLatched = false

        %% ===== Figure =====
        fig

        %% ===== Robot connection controls =====
        txtConnection
        btnConnect
        btnDisconnect
        editRobotPort
        editRobotBaud

        %% ===== Teleop settings =====
        editLinSpeed
        editRotSpeed
        editLoopHz
        btnApplyTeleop
        btnStopTeleop
        txtTeleopStatus
        txtActiveKeys

        %% ===== Pulse buttons =====
        btnL1Plus
        btnL1Minus
        btnR1Plus
        btnR1Minus
        btnL2Plus
        btnL2Minus
        btnR2Plus
        btnR2Minus
        btnL3Plus
        btnL3Minus
        btnR3Plus
        btnR3Minus

        %% ===== Absolute pose inputs =====
        editLin1
        editLin2
        editLin3
        editRot1
        editRot2
        editRot3
        btnMoveTo
        btnSetCurrentPose
        btnFillCurrentToInputs

        %% ===== Current pose display =====
        txtCurrLin1
        txtCurrLin2
        txtCurrLin3
        txtCurrRot1
        txtCurrRot2
        txtCurrRot3
        txtCurrPoseString

        %% ===== Robot plots =====
        axLin
        axRot
        lineLin1
        lineLin2
        lineLin3
        lineRot1
        lineRot2
        lineRot3

        %% ===== Pressure panel controls =====
        txtSensorConnection
        editSensorPort
        editSensorBaud
        editPressureThreshold
        btnSensorConnect
        btnSensorDisconnect
        txtPressureValue1
        txtPressureValue2
        txtAlarmStatus
        editJoyDeadzone
        txtSpeedModes
        txtEStop
        txtJoy1
        txtJoy2
        txtJoy3

        %% ===== Pressure plot =====
        axPressure
        linePressure1
        linePressure2

        %% ===== Pseudo heatmap =====
        panelHeatmap
        axHeatmap
        heatmapImg
        editHeatmapMin
        editHeatmapMax

        %% ===== Message log =====
        listMsg

        %% ===== Buffers =====
        tBuf = []
        lin1Buf = []
        lin2Buf = []
        lin3Buf = []
        rot1Buf = []
        rot2Buf = []
        rot3Buf = []
        pressureTimeBuf = []
        pressureBuf1 = []
        pressureBuf2 = []
        maxBufLen = 400
        ticId

        %% ===== Timer =====
        timerObj

        %% ===== Pseudo-PMC state =====
        loopPeriod = 0.008            % 125 Hz default
        linSpeedDefault = 1        % translation units / s
        rotSpeedDefault = 15.0       % rotation units / s
        pulseDuration = 0.15         % seconds for button tap motion
        keyState
        lastTeleopWasActive = false
    end

    methods
        function app = CTRControlApp_PseudoPMC()
            app.keyState = struct( ...
                'q', false, 'w', false, 'e', false, 'r', false, ...
                'a', false, 's', false, 'd', false, 'f', false, ...
                'z', false, 'x', false, 'c', false, 'v', false);
            app.buildUI();
            app.createTimer();
            app.ticId = tic;
            app.appendMessage('Pseudo-PMC CTR control app initialized.');
            app.appendMessage('Keyboard teleop: Tube1=q/w/e/r, Tube2=a/s/d/f, Tube3=z/x/c/v');
            app.updateRobotControlEnable('off');
            app.updatePseudoHeatmap(NaN, NaN);
            app.updateTeleopStatus();
            app.updateJoystickStatus();
        end

        function delete(app)
            try
                app.zeroAllTeleopKeys();
            catch
            end
            try
                if ~isempty(app.timerObj) && isvalid(app.timerObj)
                    stop(app.timerObj);
                    delete(app.timerObj);
                end
            catch
            end
            try
                app.disconnectSensor();
            catch
            end
            try
                app.disconnectHardware();
            catch
            end
            try
                if ~isempty(app.fig) && ishghandle(app.fig)
                    delete(app.fig);
                end
            catch
            end
        end
    end

    methods (Access = private)
        function buildUI(app)
            app.fig = figure( ...
                'Name', 'CTR Hardware + Pressure Console (Pseudo-PMC)', ...
                'NumberTitle', 'off', ...
                'MenuBar', 'none', ...
                'ToolBar', 'none', ...
                'Position', [40 30 1500 880], ...
                'CloseRequestFcn', @(src, evt) app.onClose(), ...
                'WindowKeyPressFcn', @(src, evt) app.onKeyPress(evt), ...
                'WindowKeyReleaseFcn', @(src, evt) app.onKeyRelease(evt));

            %% ===== Robot Connection =====
            uipanel('Parent', app.fig, 'Title', 'Robot Connection', ...
                'Units', 'pixels', 'Position', [20 735 330 105]);

            uicontrol(app.fig, 'Style', 'text', 'String', 'Port:', ...
                'Position', [35 792 40 20], 'HorizontalAlignment', 'left');
            app.editRobotPort = uicontrol(app.fig, 'Style', 'edit', ...
                'String', char(app.robotPort), ...
                'Position', [75 790 70 24]);

            uicontrol(app.fig, 'Style', 'text', 'String', 'Baud:', ...
                'Position', [155 792 40 20], 'HorizontalAlignment', 'left');
            app.editRobotBaud = uicontrol(app.fig, 'Style', 'edit', ...
                'String', num2str(app.robotBaud), ...
                'Position', [195 790 65 24]);

            app.btnConnect = uicontrol(app.fig, 'Style', 'pushbutton', ...
                'String', 'Connect Robot', ...
                'Position', [35 755 120 28], ...
                'Callback', @(src, evt) app.onConnectRobot());

            app.btnDisconnect = uicontrol(app.fig, 'Style', 'pushbutton', ...
                'String', 'Disconnect', ...
                'Position', [170 755 100 28], ...
                'Callback', @(src, evt) app.onDisconnectRobot());

            app.txtConnection = uicontrol(app.fig, 'Style', 'text', ...
                'String', 'Robot: Disconnected', ...
                'ForegroundColor', [0.8 0 0], ...
                'Position', [35 730 250 20], ...
                'HorizontalAlignment', 'left');

            %% ===== Teleop Settings =====
            uipanel('Parent', app.fig, 'Title', 'Pseudo-PMC Settings', ...
                'Units', 'pixels', 'Position', [20 610 330 110]);

            uicontrol(app.fig, 'Style', 'text', 'String', 'Linear speed:', ...
                'Position', [35 682 80 20], 'HorizontalAlignment', 'left');
            app.editLinSpeed = uicontrol(app.fig, 'Style', 'edit', ...
                'String', num2str(app.linSpeedDefault), ...
                'Position', [120 680 60 24]);

            uicontrol(app.fig, 'Style', 'text', 'String', 'Rot speed:', ...
                'Position', [190 682 70 20], 'HorizontalAlignment', 'left');
            app.editRotSpeed = uicontrol(app.fig, 'Style', 'edit', ...
                'String', num2str(app.rotSpeedDefault), ...
                'Position', [260 680 55 24]);

            uicontrol(app.fig, 'Style', 'text', 'String', 'Loop Hz:', ...
                'Position', [35 652 55 20], 'HorizontalAlignment', 'left');
            app.editLoopHz = uicontrol(app.fig, 'Style', 'edit', ...
                'String', num2str(1 / app.loopPeriod), ...
                'Position', [90 650 50 24]);

            app.btnApplyTeleop = uicontrol(app.fig, 'Style', 'pushbutton', ...
                'String', 'Apply', ...
                'Position', [155 650 70 24], ...
                'Callback', @(src, evt) app.onApplyTeleopSettings());

            app.btnStopTeleop = uicontrol(app.fig, 'Style', 'pushbutton', ...
                'String', 'STOP', ...
                'ForegroundColor', [0.8 0 0], ...
                'FontWeight', 'bold', ...
                'Position', [240 648 75 28], ...
                'Callback', @(src, evt) app.onEmergencyStop());

            app.txtTeleopStatus = uicontrol(app.fig, 'Style', 'text', ...
                'String', 'Teleop: IDLE', ...
                'Position', [35 625 280 20], ...
                'HorizontalAlignment', 'left', ...
                'ForegroundColor', [0 0.4 0.8]);

            %% ===== Tube Teleop / Pulse Control =====
            uipanel('Parent', app.fig, 'Title', 'Keyboard Teleop + Pulse Buttons', ...
                'Units', 'pixels', 'Position', [20 380 330 210]);

            uicontrol(app.fig, 'Style', 'text', 'String', 'Tube', ...
                'Position', [35 555 45 20], 'FontWeight', 'bold');
            uicontrol(app.fig, 'Style', 'text', 'String', 'Lin +', ...
                'Position', [95 555 45 20], 'FontWeight', 'bold');
            uicontrol(app.fig, 'Style', 'text', 'String', 'Lin -', ...
                'Position', [145 555 45 20], 'FontWeight', 'bold');
            uicontrol(app.fig, 'Style', 'text', 'String', 'Rot +', ...
                'Position', [200 555 45 20], 'FontWeight', 'bold');
            uicontrol(app.fig, 'Style', 'text', 'String', 'Rot -', ...
                'Position', [255 555 45 20], 'FontWeight', 'bold');

            uicontrol(app.fig, 'Style', 'text', 'String', 'Tube 1', ...
                'Position', [35 525 50 20], 'HorizontalAlignment', 'left');
            app.btnL1Plus = uicontrol(app.fig, 'Style', 'pushbutton', 'String', 'Q', ...
                'Position', [95 520 40 28], 'Callback', @(src,evt)app.onPulseMove(1,'lin',+1));
            app.btnL1Minus = uicontrol(app.fig, 'Style', 'pushbutton', 'String', 'W', ...
                'Position', [145 520 40 28], 'Callback', @(src,evt)app.onPulseMove(1,'lin',-1));
            app.btnR1Plus = uicontrol(app.fig, 'Style', 'pushbutton', 'String', 'E', ...
                'Position', [200 520 40 28], 'Callback', @(src,evt)app.onPulseMove(1,'rot',+1));
            app.btnR1Minus = uicontrol(app.fig, 'Style', 'pushbutton', 'String', 'R', ...
                'Position', [255 520 40 28], 'Callback', @(src,evt)app.onPulseMove(1,'rot',-1));

            uicontrol(app.fig, 'Style', 'text', 'String', 'Tube 2', ...
                'Position', [35 480 50 20], 'HorizontalAlignment', 'left');
            app.btnL2Plus = uicontrol(app.fig, 'Style', 'pushbutton', 'String', 'A', ...
                'Position', [95 475 40 28], 'Callback', @(src,evt)app.onPulseMove(2,'lin',+1));
            app.btnL2Minus = uicontrol(app.fig, 'Style', 'pushbutton', 'String', 'S', ...
                'Position', [145 475 40 28], 'Callback', @(src,evt)app.onPulseMove(2,'lin',-1));
            app.btnR2Plus = uicontrol(app.fig, 'Style', 'pushbutton', 'String', 'D', ...
                'Position', [200 475 40 28], 'Callback', @(src,evt)app.onPulseMove(2,'rot',+1));
            app.btnR2Minus = uicontrol(app.fig, 'Style', 'pushbutton', 'String', 'F', ...
                'Position', [255 475 40 28], 'Callback', @(src,evt)app.onPulseMove(2,'rot',-1));

            uicontrol(app.fig, 'Style', 'text', 'String', 'Tube 3', ...
                'Position', [35 435 50 20], 'HorizontalAlignment', 'left');
            app.btnL3Plus = uicontrol(app.fig, 'Style', 'pushbutton', 'String', 'Z', ...
                'Position', [95 430 40 28], 'Callback', @(src,evt)app.onPulseMove(3,'lin',+1));
            app.btnL3Minus = uicontrol(app.fig, 'Style', 'pushbutton', 'String', 'X', ...
                'Position', [145 430 40 28], 'Callback', @(src,evt)app.onPulseMove(3,'lin',-1));
            app.btnR3Plus = uicontrol(app.fig, 'Style', 'pushbutton', 'String', 'C', ...
                'Position', [200 430 40 28], 'Callback', @(src,evt)app.onPulseMove(3,'rot',+1));
            app.btnR3Minus = uicontrol(app.fig, 'Style', 'pushbutton', 'String', 'V', ...
                'Position', [255 430 40 28], 'Callback', @(src,evt)app.onPulseMove(3,'rot',-1));

            app.txtActiveKeys = uicontrol(app.fig, 'Style', 'text', ...
                'String', 'Active keys: none', ...
                'Position', [35 395 280 22], ...
                'HorizontalAlignment', 'left');

            %% ===== Absolute Pose Command =====
            uipanel('Parent', app.fig, 'Title', 'Absolute Pose Command', ...
                'Units', 'pixels', 'Position', [20 130 330 230]);

            y0 = 310; dy = 32;
            uicontrol(app.fig, 'Style', 'text', 'String', 'lin1', ...
                'Position', [35 y0 40 20], 'HorizontalAlignment', 'left');
            app.editLin1 = uicontrol(app.fig, 'Style', 'edit', 'String', '0', ...
                'Position', [80 y0 60 24]);
            uicontrol(app.fig, 'Style', 'text', 'String', 'lin2', ...
                'Position', [160 y0 40 20], 'HorizontalAlignment', 'left');
            app.editLin2 = uicontrol(app.fig, 'Style', 'edit', 'String', '0', ...
                'Position', [205 y0 60 24]);
            uicontrol(app.fig, 'Style', 'text', 'String', 'lin3', ...
                'Position', [35 y0-dy 40 20], 'HorizontalAlignment', 'left');
            app.editLin3 = uicontrol(app.fig, 'Style', 'edit', 'String', '0', ...
                'Position', [80 y0-dy 60 24]);
            uicontrol(app.fig, 'Style', 'text', 'String', 'rot1', ...
                'Position', [160 y0-dy 40 20], 'HorizontalAlignment', 'left');
            app.editRot1 = uicontrol(app.fig, 'Style', 'edit', 'String', '0', ...
                'Position', [205 y0-dy 60 24]);
            uicontrol(app.fig, 'Style', 'text', 'String', 'rot2', ...
                'Position', [35 y0-2*dy 40 20], 'HorizontalAlignment', 'left');
            app.editRot2 = uicontrol(app.fig, 'Style', 'edit', 'String', '0', ...
                'Position', [80 y0-2*dy 60 24]);
            uicontrol(app.fig, 'Style', 'text', 'String', 'rot3', ...
                'Position', [160 y0-2*dy 40 20], 'HorizontalAlignment', 'left');
            app.editRot3 = uicontrol(app.fig, 'Style', 'edit', 'String', '0', ...
                'Position', [205 y0-2*dy 60 24]);

            app.btnMoveTo = uicontrol(app.fig, 'Style', 'pushbutton', ...
                'String', 'Move To Pose', ...
                'Position', [35 185 110 30], ...
                'Callback', @(src,evt)app.onMoveToPose());
            app.btnSetCurrentPose = uicontrol(app.fig, 'Style', 'pushbutton', ...
                'String', 'Set Current Pose', ...
                'Position', [155 185 110 30], ...
                'Callback', @(src,evt)app.onSetCurrentPose());
            app.btnFillCurrentToInputs = uicontrol(app.fig, 'Style', 'pushbutton', ...
                'String', 'Fill Current -> Inputs', ...
                'Position', [35 145 230 28], ...
                'Callback', @(src,evt)app.fillInputsFromCurrent());

            %% ===== Current Commanded Pose =====
            uipanel('Parent', app.fig, 'Title', 'Current Commanded Pose', ...
                'Units', 'pixels', 'Position', [370 625 420 215]);
            xs1 = 390; xs2 = 460; xs3 = 590; xs4 = 660;
            yA = 770; dyy = 35;
            uicontrol(app.fig, 'Style', 'text', 'String', 'lin1', ...
                'Position', [xs1 yA 50 20], 'HorizontalAlignment', 'left');
            app.txtCurrLin1 = uicontrol(app.fig, 'Style', 'text', 'String', '0', ...
                'Position', [xs2 yA 90 20], 'HorizontalAlignment', 'left');
            uicontrol(app.fig, 'Style', 'text', 'String', 'lin2', ...
                'Position', [xs3 yA 50 20], 'HorizontalAlignment', 'left');
            app.txtCurrLin2 = uicontrol(app.fig, 'Style', 'text', 'String', '0', ...
                'Position', [xs4 yA 90 20], 'HorizontalAlignment', 'left');
            uicontrol(app.fig, 'Style', 'text', 'String', 'lin3', ...
                'Position', [xs1 yA-dyy 50 20], 'HorizontalAlignment', 'left');
            app.txtCurrLin3 = uicontrol(app.fig, 'Style', 'text', 'String', '0', ...
                'Position', [xs2 yA-dyy 90 20], 'HorizontalAlignment', 'left');
            uicontrol(app.fig, 'Style', 'text', 'String', 'rot1', ...
                'Position', [xs3 yA-dyy 50 20], 'HorizontalAlignment', 'left');
            app.txtCurrRot1 = uicontrol(app.fig, 'Style', 'text', 'String', '0', ...
                'Position', [xs4 yA-dyy 90 20], 'HorizontalAlignment', 'left');
            uicontrol(app.fig, 'Style', 'text', 'String', 'rot2', ...
                'Position', [xs1 yA-2*dyy 50 20], 'HorizontalAlignment', 'left');
            app.txtCurrRot2 = uicontrol(app.fig, 'Style', 'text', 'String', '0', ...
                'Position', [xs2 yA-2*dyy 90 20], 'HorizontalAlignment', 'left');
            uicontrol(app.fig, 'Style', 'text', 'String', 'rot3', ...
                'Position', [xs3 yA-2*dyy 50 20], 'HorizontalAlignment', 'left');
            app.txtCurrRot3 = uicontrol(app.fig, 'Style', 'text', 'String', '0', ...
                'Position', [xs4 yA-2*dyy 90 20], 'HorizontalAlignment', 'left');
            uicontrol(app.fig, 'Style', 'text', 'String', 'Pose String:', ...
                'Position', [390 665 80 20], 'HorizontalAlignment', 'left');
            app.txtCurrPoseString = uicontrol(app.fig, 'Style', 'text', ...
                'String', '(0,0,0,0,0,0)', ...
                'Position', [470 665 300 20], ...
                'HorizontalAlignment', 'left');

            %% ===== Translation Plot =====
            panelLin = uipanel('Parent', app.fig, 'Title', 'Translation History', ...
                'Units', 'pixels', 'Position', [370 360 460 220]);
            app.axLin = axes('Parent', panelLin, 'Units', 'normalized', ...
                'Position', [0.10 0.18 0.85 0.72]);
            hold(app.axLin, 'on'); grid(app.axLin, 'on');
            title(app.axLin, 'lin1 / lin2 / lin3'); xlabel(app.axLin, 'Time (s)'); ylabel(app.axLin, 'Translation');
            app.lineLin1 = plot(app.axLin, nan, nan, 'LineWidth', 1.5);
            app.lineLin2 = plot(app.axLin, nan, nan, 'LineWidth', 1.5);
            app.lineLin3 = plot(app.axLin, nan, nan, 'LineWidth', 1.5);
            legend(app.axLin, {'lin1','lin2','lin3'}, 'Location', 'best');

            %% ===== Rotation Plot =====
            panelRot = uipanel('Parent', app.fig, 'Title', 'Rotation History', ...
                'Units', 'pixels', 'Position', [370 100 460 220]);
            app.axRot = axes('Parent', panelRot, 'Units', 'normalized', ...
                'Position', [0.10 0.18 0.85 0.72]);
            hold(app.axRot, 'on'); grid(app.axRot, 'on');
            title(app.axRot, 'rot1 / rot2 / rot3'); xlabel(app.axRot, 'Time (s)'); ylabel(app.axRot, 'Rotation');
            app.lineRot1 = plot(app.axRot, nan, nan, 'LineWidth', 1.5);
            app.lineRot2 = plot(app.axRot, nan, nan, 'LineWidth', 1.5);
            app.lineRot3 = plot(app.axRot, nan, nan, 'LineWidth', 1.5);
            legend(app.axRot, {'rot1','rot2','rot3'}, 'Location', 'best');

            %% ===== Pressure Monitor =====
            uipanel('Parent', app.fig, 'Title', 'Pressure Sensor Monitor', ...
                'Units', 'pixels', 'Position', [850 360 610 480]);
            uicontrol(app.fig, 'Style', 'text', 'String', 'Sensor Port:', ...
                'Position', [870 790 75 20], 'HorizontalAlignment', 'left');
            app.editSensorPort = uicontrol(app.fig, 'Style', 'edit', ...
                'String', app.sensorPort, ...
                'Position', [945 790 70 24]);
            uicontrol(app.fig, 'Style', 'text', 'String', 'Baud:', ...
                'Position', [1025 790 40 20], 'HorizontalAlignment', 'left');
            app.editSensorBaud = uicontrol(app.fig, 'Style', 'edit', ...
                'String', num2str(app.sensorBaud), ...
                'Position', [1065 790 65 24]);
            app.btnSensorConnect = uicontrol(app.fig, 'Style', 'pushbutton', ...
                'String', 'Connect Sensor', ...
                'Position', [1145 788 110 28], ...
                'Callback', @(src,evt)app.onConnectSensor());
            app.btnSensorDisconnect = uicontrol(app.fig, 'Style', 'pushbutton', ...
                'String', 'Disconnect', ...
                'Position', [1265 788 90 28], ...
                'Callback', @(src,evt)app.onDisconnectSensor());
            app.txtSensorConnection = uicontrol(app.fig, 'Style', 'text', ...
                'String', 'Sensor: Disconnected', ...
                'ForegroundColor', [0.8 0 0], ...
                'Position', [870 762 260 20], ...
                'HorizontalAlignment', 'left');

            uicontrol(app.fig, 'Style', 'text', 'String', 'Pressure 1 (V):', ...
                'Position', [870 730 90 20], 'HorizontalAlignment', 'left', 'FontWeight', 'bold');
            app.txtPressureValue1 = uicontrol(app.fig, 'Style', 'text', ...
                'String', '--', ...
                'Position', [965 728 90 24], 'HorizontalAlignment', 'left', ...
                'FontSize', 14, 'ForegroundColor', [0 0 0.8]);

            uicontrol(app.fig, 'Style', 'text', 'String', 'Pressure 2 (V):', ...
                'Position', [1060 730 90 20], 'HorizontalAlignment', 'left', 'FontWeight', 'bold');
            app.txtPressureValue2 = uicontrol(app.fig, 'Style', 'text', ...
                'String', '--', ...
                'Position', [1155 728 90 24], 'HorizontalAlignment', 'left', ...
                'FontSize', 14, 'ForegroundColor', [0 0.45 0]);

            uicontrol(app.fig, 'Style', 'text', 'String', 'Alarm Threshold (V):', ...
                'Position', [1250 730 110 20], 'HorizontalAlignment', 'left');
            app.editPressureThreshold = uicontrol(app.fig, 'Style', 'edit', ...
                'String', '2.5', ...
                'Position', [1360 728 60 24]);

            uicontrol(app.fig, 'Style', 'text', 'String', 'Alarm:', ...
                'Position', [1250 700 45 20], 'HorizontalAlignment', 'left');
            app.txtAlarmStatus = uicontrol(app.fig, 'Style', 'text', ...
                'String', 'NORMAL', ...
                'Position', [1300 698 120 24], 'ForegroundColor', [0 0.6 0], 'FontWeight', 'bold');

            uicontrol(app.fig, 'Style', 'text', 'String', 'Joy deadzone (V):', ...
                'Position', [870 670 110 20], 'HorizontalAlignment', 'left');
            app.editJoyDeadzone = uicontrol(app.fig, 'Style', 'edit', ...
                'String', num2str(app.joyDeadzoneV), ...
                'Position', [985 668 65 24]);

            app.txtSpeedModes = uicontrol(app.fig, 'Style', 'text', ...
                'String', '', ...
                'Position', [1070 668 240 24], ...
                'HorizontalAlignment', 'left', ...
                'ForegroundColor', [0.15 0.35 0.9], ...
                'FontWeight', 'bold');

            app.txtEStop = uicontrol(app.fig, 'Style', 'text', ...
                'String', '', ...
                'Position', [1320 668 110 24], ...
                'HorizontalAlignment', 'left', ...
                'ForegroundColor', [0 0.6 0], ...
                'FontWeight', 'bold');

            app.txtJoy1 = uicontrol(app.fig, 'Style', 'text', ...
                'String', 'Joy1 -> Tube1 | X=-- Y=-- SW=0 | IDLE', ...
                'Position', [870 640 560 22], ...
                'HorizontalAlignment', 'left');
            app.txtJoy2 = uicontrol(app.fig, 'Style', 'text', ...
                'String', 'Joy2 -> Tube2 | X=-- Y=-- SW=0 | IDLE', ...
                'Position', [870 610 560 22], ...
                'HorizontalAlignment', 'left');
            app.txtJoy3 = uicontrol(app.fig, 'Style', 'text', ...
                'String', 'Joy3 -> Tube3 | X=-- Y=-- SW=0 | IDLE', ...
                'Position', [870 580 560 22], ...
                'HorizontalAlignment', 'left');

            panelPressure = uipanel('Parent', app.fig, 'Title', 'Pressure History (2 Sensors)', ...
                'Units', 'pixels', 'Position', [865 420 585 150]);
            app.axPressure = axes('Parent', panelPressure, 'Units', 'normalized', ...
                'Position', [0.08 0.18 0.88 0.72]);
            grid(app.axPressure, 'on');
            title(app.axPressure, 'Pressure Voltage');
            xlabel(app.axPressure, 'Time (s)');
            ylabel(app.axPressure, 'Voltage (V)');
            hold(app.axPressure, 'on');
            app.linePressure1 = plot(app.axPressure, nan, nan, 'LineWidth', 1.5);
            app.linePressure2 = plot(app.axPressure, nan, nan, 'LineWidth', 1.5);
            legend(app.axPressure, {'Sensor 1','Sensor 2'}, 'Location', 'best');

            %% ===== Pseudo Heatmap =====
            app.panelHeatmap = uipanel('Parent', app.fig, 'Title', 'Pseudo Heatmap (2 Sensors)', ...
                'Units', 'pixels', 'Position', [850 215 610 155]);
            uicontrol('Parent', app.panelHeatmap, 'Style', 'text', 'String', 'Min V:', ...
                'Position', [20 110 45 20], 'HorizontalAlignment', 'left');
            app.editHeatmapMin = uicontrol('Parent', app.panelHeatmap, 'Style', 'edit', ...
                'String', '0.0', 'Position', [65 108 55 24]);
            uicontrol('Parent', app.panelHeatmap, 'Style', 'text', 'String', 'Max V:', ...
                'Position', [140 110 45 20], 'HorizontalAlignment', 'left');
            app.editHeatmapMax = uicontrol('Parent', app.panelHeatmap, 'Style', 'edit', ...
                'String', '5.0', 'Position', [185 108 55 24]);
            app.axHeatmap = axes('Parent', app.panelHeatmap, 'Units', 'pixels', ...
                'Position', [300 20 230 80]);
            img0 = zeros(40, 80);
            app.heatmapImg = imagesc(app.axHeatmap, img0);
            axis(app.axHeatmap, 'image');
            axis(app.axHeatmap, 'off');
            colormap(app.axHeatmap, jet);
            caxis(app.axHeatmap, [0 1]);
            title(app.axHeatmap, 'Pressure Intensity');
            colorbar(app.axHeatmap);

            %% ===== Message Log =====
            uipanel('Parent', app.fig, 'Title', 'Message Log', ...
                'Units', 'pixels', 'Position', [850 40 610 180]);
            app.listMsg = uicontrol(app.fig, 'Style', 'listbox', ...
                'Position', [865 55 580 140], 'String', {'System log...'}, 'Max', 2, 'Min', 0);
        end

        function createTimer(app)
            app.timerObj = timer( ...
                'ExecutionMode', 'fixedSpacing', ...
                'Period', app.loopPeriod, ...
                'BusyMode', 'drop', ...
                'TimerFcn', @(~,~) app.onTimerTick());
        end

        function onConnectRobot(app)
            try
                if app.isConnected
                    app.appendMessage('Robot already connected.');
                    return;
                end

                app.robotPort = string(get(app.editRobotPort, 'String'));
                app.robotBaud = str2double(get(app.editRobotBaud, 'String'));
                if strlength(app.robotPort) == 0 || isnan(app.robotBaud)
                    error('Robot port or baud is invalid.');
                end

                startPose = Pose(0,0,0,0,0,0);
                app.driveBot = Drive(startPose, app.robotPort, app.robotBaud);
                app.isConnected = true;
                set(app.txtConnection, 'String', ['Robot: Connected (' char(app.robotPort) ')'], ...
                    'ForegroundColor', [0 0.6 0]);

                app.resetRobotBuffers();
                app.refreshCurrentPoseDisplay();
                app.fillInputsFromCurrent();
                app.updateRobotControlEnable('on');
                app.onApplyTeleopSettings(false);

                if strcmp(app.timerObj.Running, 'off')
                    start(app.timerObj);
                end

                app.appendMessage('Robot connected successfully.');
                app.appendMessage('Click the figure background once if keyboard teleop does not respond immediately.');
            catch ME
                app.appendMessage(['Robot connect failed: ' ME.message]);
                errordlg(ME.message, 'Robot Connect Error');
            end
        end

        function onDisconnectRobot(app)
            app.zeroAllTeleopKeys();
            app.disconnectHardware();
            app.isConnected = false;
            set(app.txtConnection, 'String', 'Robot: Disconnected', 'ForegroundColor', [0.8 0 0]);
            app.updateRobotControlEnable('off');
            app.updateTeleopStatus();
            app.appendMessage('Robot disconnected.');
            app.stopTimerIfIdle();
        end

        function disconnectHardware(app)
            if isempty(app.driveBot)
                return;
            end
            try
                delete(app.driveBot);
            catch
            end
            app.driveBot = [];
        end

        function onApplyTeleopSettings(app, showMessage)
            if nargin < 2
                showMessage = true;
            end

            linSpeed = str2double(get(app.editLinSpeed, 'String'));
            rotSpeed = str2double(get(app.editRotSpeed, 'String'));
            loopHz = str2double(get(app.editLoopHz, 'String'));

            if isnan(linSpeed) || linSpeed <= 0 || isnan(rotSpeed) || rotSpeed <= 0 || isnan(loopHz) || loopHz <= 0
                errordlg('Linear speed, rotation speed, and loop Hz must all be positive numbers.', 'Teleop Config Error');
                return;
            end

            app.linSpeedDefault = linSpeed;
            app.rotSpeedDefault = rotSpeed;
            app.loopPeriod = 1 / loopHz;

            if ~isempty(app.timerObj) && isvalid(app.timerObj)
                wasRunning = strcmp(app.timerObj.Running, 'on');
                if wasRunning
                    stop(app.timerObj);
                end
                app.timerObj.Period = app.loopPeriod;
                if wasRunning
                    start(app.timerObj);
                end
            end

            if showMessage
                app.appendMessage(sprintf('Teleop settings updated: linSpeed=%.3f, rotSpeed=%.3f, loopHz=%.2f', ...
                    app.linSpeedDefault, app.rotSpeedDefault, loopHz));
            end
        end

        function onEmergencyStop(app)
            app.zeroAllTeleopKeys();
            app.estopLatched = true;
            app.updateTeleopStatus();
            app.updateJoystickStatus();
            app.appendMessage('STOP pressed: E-STOP latched and all pseudo-PMC key commands cleared.');
        end

        function onPulseMove(app, tubeIdx, motionType, direction)
            if ~app.isConnected || isempty(app.driveBot)
                app.appendMessage('Robot not connected.');
                return;
            end

            app.onApplyTeleopSettings(false);
            linVel = [0 0 0];
            rotVel = [0 0 0];
            switch motionType
                case 'lin'
                    linVel(tubeIdx) = direction * app.linSpeedDefault;
                case 'rot'
                    rotVel(tubeIdx) = direction * app.rotSpeedDefault;
                otherwise
                    return;
            end

            try
                app.driveBot.stream_velocity(linVel, rotVel, app.pulseDuration);
                app.refreshCurrentPoseDisplay();
                app.appendMessage(sprintf('Pulse move: tube=%d type=%s dir=%+d duration=%.2f s', ...
                    tubeIdx, motionType, direction, app.pulseDuration));
            catch ME
                app.appendMessage(['Pulse move failed: ' ME.message]);
                errordlg(ME.message, 'Pulse Move Error');
            end
        end

        function onMoveToPose(app)
            if ~app.isConnected || isempty(app.driveBot)
                app.appendMessage('Robot not connected.');
                return;
            end
            vals = app.readPoseInputs();
            if isempty(vals)
                return;
            end
            try
                app.driveBot.travel_to(vals(1), vals(2), vals(3), vals(4), vals(5), vals(6));
                app.refreshCurrentPoseDisplay();
                app.appendMessage(sprintf('travel_to -> [%.3f %.3f %.3f %.3f %.3f %.3f]', vals));
            catch ME
                app.appendMessage(['Move-to command failed: ' ME.message]);
                errordlg(ME.message, 'Move Error');
            end
        end

        function onSetCurrentPose(app)
            if ~app.isConnected || isempty(app.driveBot)
                app.appendMessage('Robot not connected.');
                return;
            end
            vals = app.readPoseInputs();
            if isempty(vals)
                return;
            end
            try
                p = Pose(vals(1), vals(2), vals(3), vals(4), vals(5), vals(6));
                app.driveBot.set_current_pose(p);
                app.refreshCurrentPoseDisplay();
                app.appendMessage(sprintf('set_current_pose -> [%.3f %.3f %.3f %.3f %.3f %.3f]', vals));
            catch ME
                app.appendMessage(['Set current pose failed: ' ME.message]);
                errordlg(ME.message, 'Pose Error');
            end
        end

        function fillInputsFromCurrent(app)
            if isempty(app.driveBot) || isempty(app.driveBot.currPose)
                vals = zeros(1,6);
            else
                vals = app.driveBot.currPose.get_pose();
            end
            set(app.editLin1, 'String', num2str(vals(1)));
            set(app.editLin2, 'String', num2str(vals(2)));
            set(app.editLin3, 'String', num2str(vals(3)));
            set(app.editRot1, 'String', num2str(vals(4)));
            set(app.editRot2, 'String', num2str(vals(5)));
            set(app.editRot3, 'String', num2str(vals(6)));
        end

        function vals = readPoseInputs(app)
            vals = zeros(1,6);
            handles = {app.editLin1, app.editLin2, app.editLin3, app.editRot1, app.editRot2, app.editRot3};
            for i = 1:6
                vals(i) = str2double(get(handles{i}, 'String'));
                if isnan(vals(i))
                    errordlg('One or more pose inputs are invalid.', 'Input Error');
                    vals = [];
                    return;
                end
            end
        end

        function refreshCurrentPoseDisplay(app)
            if isempty(app.driveBot) || isempty(app.driveBot.currPose)
                return;
            end
            p = app.driveBot.currPose;
            set(app.txtCurrLin1, 'String', sprintf('%.3f', p.lin1));
            set(app.txtCurrLin2, 'String', sprintf('%.3f', p.lin2));
            set(app.txtCurrLin3, 'String', sprintf('%.3f', p.lin3));
            set(app.txtCurrRot1, 'String', sprintf('%.3f', p.rot1));
            set(app.txtCurrRot2, 'String', sprintf('%.3f', p.rot2));
            set(app.txtCurrRot3, 'String', sprintf('%.3f', p.rot3));
            set(app.txtCurrPoseString, 'String', char(p.get_string_for_pose()));
        end

        function updateRobotPlots(app)
            if isempty(app.driveBot) || isempty(app.driveBot.currPose)
                return;
            end
            p = app.driveBot.currPose;
            t = toc(app.ticId);
            app.tBuf(end+1) = t;
            app.lin1Buf(end+1) = p.lin1;
            app.lin2Buf(end+1) = p.lin2;
            app.lin3Buf(end+1) = p.lin3;
            app.rot1Buf(end+1) = p.rot1;
            app.rot2Buf(end+1) = p.rot2;
            app.rot3Buf(end+1) = p.rot3;
            if numel(app.tBuf) > app.maxBufLen
                idx = numel(app.tBuf)-app.maxBufLen+1 : numel(app.tBuf);
                app.tBuf = app.tBuf(idx);
                app.lin1Buf = app.lin1Buf(idx);
                app.lin2Buf = app.lin2Buf(idx);
                app.lin3Buf = app.lin3Buf(idx);
                app.rot1Buf = app.rot1Buf(idx);
                app.rot2Buf = app.rot2Buf(idx);
                app.rot3Buf = app.rot3Buf(idx);
            end
            set(app.lineLin1, 'XData', app.tBuf, 'YData', app.lin1Buf);
            set(app.lineLin2, 'XData', app.tBuf, 'YData', app.lin2Buf);
            set(app.lineLin3, 'XData', app.tBuf, 'YData', app.lin3Buf);
            set(app.lineRot1, 'XData', app.tBuf, 'YData', app.rot1Buf);
            set(app.lineRot2, 'XData', app.tBuf, 'YData', app.rot2Buf);
            set(app.lineRot3, 'XData', app.tBuf, 'YData', app.rot3Buf);
            if ~isempty(app.tBuf)
                x1 = max(0, app.tBuf(end)-20);
                x2 = max(20, app.tBuf(end));
                xlim(app.axLin, [x1 x2]);
                xlim(app.axRot, [x1 x2]);
            end
        end

        function resetRobotBuffers(app)
            app.tBuf = [];
            app.lin1Buf = [];
            app.lin2Buf = [];
            app.lin3Buf = [];
            app.rot1Buf = [];
            app.rot2Buf = [];
            app.rot3Buf = [];
            app.ticId = tic;
        end

        function updateRobotControlEnable(app, state)
            ctrls = { ...
                app.editLinSpeed, app.editRotSpeed, app.editLoopHz, app.btnApplyTeleop, app.btnStopTeleop, ...
                app.btnL1Plus, app.btnL1Minus, app.btnR1Plus, app.btnR1Minus, ...
                app.btnL2Plus, app.btnL2Minus, app.btnR2Plus, app.btnR2Minus, ...
                app.btnL3Plus, app.btnL3Minus, app.btnR3Plus, app.btnR3Minus, ...
                app.editLin1, app.editLin2, app.editLin3, ...
                app.editRot1, app.editRot2, app.editRot3, ...
                app.btnMoveTo, app.btnSetCurrentPose, app.btnFillCurrentToInputs ...
            };
            for i = 1:numel(ctrls)
                try
                    set(ctrls{i}, 'Enable', state);
                catch
                end
            end
        end

        function onKeyPress(app, evt)
            key = lower(evt.Key);
            validKeys = fieldnames(app.keyState);
            if any(strcmp(key, validKeys))
                app.keyState.(key) = true;
                app.updateTeleopStatus();
            elseif strcmp(key, 'escape')
                app.onEmergencyStop();
            end
        end

        function onKeyRelease(app, evt)
            key = lower(evt.Key);
            validKeys = fieldnames(app.keyState);
            if any(strcmp(key, validKeys))
                app.keyState.(key) = false;
                app.updateTeleopStatus();
            end
        end

        function zeroAllTeleopKeys(app)
            validKeys = fieldnames(app.keyState);
            for i = 1:numel(validKeys)
                app.keyState.(validKeys{i}) = false;
            end
        end

        function [linVel, rotVel, activeNames] = computeTeleopVelocity(app)
            linVel = [0 0 0];
            rotVel = [0 0 0];
            activeNames = {};
            app.onApplyTeleopSettings(false);

            if ~app.estopLatched
                if app.keyState.q, linVel(1) = linVel(1) + app.linSpeedDefault; activeNames{end+1} = 'q'; end
                if app.keyState.w, linVel(1) = linVel(1) - app.linSpeedDefault; activeNames{end+1} = 'w'; end
                if app.keyState.e, rotVel(1) = rotVel(1) + app.rotSpeedDefault; activeNames{end+1} = 'e'; end
                if app.keyState.r, rotVel(1) = rotVel(1) - app.rotSpeedDefault; activeNames{end+1} = 'r'; end

                if app.keyState.a, linVel(2) = linVel(2) + app.linSpeedDefault; activeNames{end+1} = 'a'; end
                if app.keyState.s, linVel(2) = linVel(2) - app.linSpeedDefault; activeNames{end+1} = 's'; end
                if app.keyState.d, rotVel(2) = rotVel(2) + app.rotSpeedDefault; activeNames{end+1} = 'd'; end
                if app.keyState.f, rotVel(2) = rotVel(2) - app.rotSpeedDefault; activeNames{end+1} = 'f'; end

                if app.keyState.z, linVel(3) = linVel(3) + app.linSpeedDefault; activeNames{end+1} = 'z'; end
                if app.keyState.x, linVel(3) = linVel(3) - app.linSpeedDefault; activeNames{end+1} = 'x'; end
                if app.keyState.c, rotVel(3) = rotVel(3) + app.rotSpeedDefault; activeNames{end+1} = 'c'; end
                if app.keyState.v, rotVel(3) = rotVel(3) - app.rotSpeedDefault; activeNames{end+1} = 'v'; end
            end

            [linJoy, rotJoy, joyLabels] = app.computeJoystickVelocity();
            linVel = linVel + linJoy;
            rotVel = rotVel + rotJoy;
            for i = 1:3
                if ~strcmp(joyLabels{i}, 'IDLE')
                    activeNames{end+1} = sprintf('joy%d:%s', i, joyLabels{i});
                end
            end
        end

        function updateTeleopStatus(app)
            [linVel, rotVel, activeNames] = app.computeTeleopVelocity();
            if app.estopLatched
                set(app.txtTeleopStatus, 'String', 'Teleop: E-STOP LATCHED', 'ForegroundColor', [0.8 0 0]);
                set(app.txtActiveKeys, 'String', 'Active keys: blocked by E-stop');
            elseif isempty(activeNames)
                set(app.txtTeleopStatus, 'String', 'Teleop: IDLE', 'ForegroundColor', [0 0.4 0.8]);
                set(app.txtActiveKeys, 'String', 'Active keys: none');
            else
                set(app.txtTeleopStatus, 'String', sprintf('Teleop: ACTIVE | lin=[%.2f %.2f %.2f] rot=[%.2f %.2f %.2f]', ...
                    linVel(1), linVel(2), linVel(3), rotVel(1), rotVel(2), rotVel(3)), ...
                    'ForegroundColor', [0 0.55 0]);
                set(app.txtActiveKeys, 'String', ['Active keys: ' strjoin(activeNames, ', ')]);
            end
        end

        function runPseudoPMCStep(app)
            if ~app.isConnected || isempty(app.driveBot)
                return;
            end
            [linVel, rotVel, activeNames] = app.computeTeleopVelocity();
            isActiveNow = ~isempty(activeNames);
            if isActiveNow
                app.driveBot.stream_velocity(linVel, rotVel, app.loopPeriod);
            end
            if isActiveNow ~= app.lastTeleopWasActive
                if isActiveNow
                    app.appendMessage(['Teleop active: ' strjoin(activeNames, ', ')]);
                else
                    app.appendMessage('Teleop idle: all keys released.');
                end
            end
            app.lastTeleopWasActive = isActiveNow;
        end

        %% ===== Sensor methods =====
        function onConnectSensor(app)
            try
                if app.sensorConnected
                    app.appendMessage('Sensor already connected.');
                    return;
                end
                app.sensorPort = get(app.editSensorPort, 'String');
                app.sensorBaud = str2double(get(app.editSensorBaud, 'String'));
                if isnan(app.sensorBaud) || isempty(app.sensorPort)
                    errordlg('Sensor port or baud is invalid.', 'Sensor Config Error');
                    return;
                end
                app.sensorSerial = serialport(app.sensorPort, app.sensorBaud);
                configureTerminator(app.sensorSerial, "LF");
                flush(app.sensorSerial);
                app.sensorConnected = true;
                set(app.txtSensorConnection, 'String', ['Sensor: Connected (' app.sensorPort ')'], 'ForegroundColor', [0 0.6 0]);
                app.resetPressureBuffers();
                app.latestPressure1 = NaN;
                app.latestPressure2 = NaN;
                app.alarmActive = false;
                app.updatePressureDisplay(NaN, NaN);
                app.updatePseudoHeatmap(NaN, NaN);
                if strcmp(app.timerObj.Running, 'off')
                    start(app.timerObj);
                end
                app.appendMessage(['Sensor connected successfully on ' app.sensorPort '.']);
                app.appendMessage('Expected Arduino format: p1,p2,j1x,j1y,j2x,j2y,j3x,j3y,sw1,sw2,sw3');
                app.updateJoystickStatus();
            catch ME
                app.appendMessage(['Sensor connect failed: ' ME.message]);
                errordlg(ME.message, 'Sensor Connect Error');
            end
        end

        function onDisconnectSensor(app)
            app.disconnectSensor();
            app.sensorConnected = false;
            set(app.txtSensorConnection, 'String', 'Sensor: Disconnected', 'ForegroundColor', [0.8 0 0]);
            app.updatePressureDisplay(NaN, NaN);
            app.updatePseudoHeatmap(NaN, NaN);
            app.latestJoyX = [NaN NaN NaN];
            app.latestJoyY = [NaN NaN NaN];
            app.latestJoySW = [0 0 0];
            app.lastJoySW = [0 0 0];
            app.updateJoystickStatus();
            app.appendMessage('Sensor disconnected.');
            app.stopTimerIfIdle();
        end

        function disconnectSensor(app)
            if isempty(app.sensorSerial)
                return;
            end
            try
                delete(app.sensorSerial);
            catch
            end
            app.sensorSerial = [];
        end

        function pollSensor(app)
            if ~app.sensorConnected || isempty(app.sensorSerial)
                return;
            end
            newest = [];
            try
                loopCount = 0;
                while app.sensorSerial.NumBytesAvailable > 0 && loopCount < 20
                    line = readline(app.sensorSerial);
                    loopCount = loopCount + 1;
                    if isstring(line), line = char(line); end
                    parts = strsplit(strtrim(line), ',');
                    n = numel(parts);
                    if ~(n == 11 || n == 5)
                        continue;
                    end
                    vals = nan(1,n);
                    ok = true;
                    for k = 1:n
                        vals(k) = str2double(strtrim(parts{k}));
                        if isnan(vals(k))
                            ok = false;
                            break;
                        end
                    end
                    if ok
                        newest = vals;
                    end
                end
            catch ME
                app.appendMessage(['Sensor read error: ' ME.message]);
                return;
            end
            if ~isempty(newest)
                if numel(newest) == 11
                    app.latestPressure1 = newest(1);
                    app.latestPressure2 = newest(2);
                    app.latestJoyX = [newest(3) newest(5) newest(7)];
                    app.latestJoyY = [newest(4) newest(6) newest(8)];
                    app.latestJoySW = round([newest(9) newest(10) newest(11)]);
                elseif numel(newest) == 5
                    % legacy one-joystick stream: p1,p2,j1x,j1y,sw1
                    app.latestPressure1 = newest(1);
                    app.latestPressure2 = newest(2);
                    app.latestJoyX = [newest(3) NaN NaN];
                    app.latestJoyY = [newest(4) NaN NaN];
                    app.latestJoySW = round([newest(5) 0 0]);
                end
                app.handleJoystickSwitchEdges();
                app.updatePressureDisplay(app.latestPressure1, app.latestPressure2);
                app.updatePressurePlot(app.latestPressure1, app.latestPressure2);
                app.updatePseudoHeatmap(app.latestPressure1, app.latestPressure2);
                app.checkPressureAlarm(app.latestPressure1, app.latestPressure2);
                app.updateJoystickStatus();
            end
        end

        function updatePressureDisplay(app, val1, val2)
            if isnan(val1) || isnan(val2)
                set(app.txtPressureValue1, 'String', '--', 'ForegroundColor', [0 0 0.8]);
                set(app.txtPressureValue2, 'String', '--', 'ForegroundColor', [0 0.45 0]);
                set(app.txtAlarmStatus, 'String', 'NORMAL', 'ForegroundColor', [0 0.6 0]);
                return;
            end
            set(app.txtPressureValue1, 'String', sprintf('%.3f', val1));
            set(app.txtPressureValue2, 'String', sprintf('%.3f', val2));
        end

        function updatePressurePlot(app, val1, val2)
            t = toc(app.ticId);
            app.pressureTimeBuf(end+1) = t;
            app.pressureBuf1(end+1) = val1;
            app.pressureBuf2(end+1) = val2;
            if numel(app.pressureTimeBuf) > app.maxBufLen
                idx = numel(app.pressureTimeBuf)-app.maxBufLen+1 : numel(app.pressureTimeBuf);
                app.pressureTimeBuf = app.pressureTimeBuf(idx);
                app.pressureBuf1 = app.pressureBuf1(idx);
                app.pressureBuf2 = app.pressureBuf2(idx);
            end
            set(app.linePressure1, 'XData', app.pressureTimeBuf, 'YData', app.pressureBuf1);
            set(app.linePressure2, 'XData', app.pressureTimeBuf, 'YData', app.pressureBuf2);
            if ~isempty(app.pressureTimeBuf)
                x1 = max(0, app.pressureTimeBuf(end)-20);
                x2 = max(20, app.pressureTimeBuf(end));
                xlim(app.axPressure, [x1 x2]);

                visibleMask = app.pressureTimeBuf >= x1;
                yVis = [app.pressureBuf1(visibleMask), app.pressureBuf2(visibleMask)];
                yVis = yVis(~isnan(yVis));
                if ~isempty(yVis)
                    yMin = min(yVis);
                    yMax = max(yVis);
                    yRange = yMax - yMin;
                    minSpan = 0.05;
                    if yRange < minSpan
                        yCenter = 0.5 * (yMin + yMax);
                        yMinPlot = yCenter - minSpan / 2;
                        yMaxPlot = yCenter + minSpan / 2;
                    else
                        pad = 0.10 * yRange;
                        yMinPlot = yMin - pad;
                        yMaxPlot = yMax + pad;
                    end
                    if yMinPlot < 0
                        yMinPlot = 0;
                    end
                    if yMaxPlot <= yMinPlot
                        yMaxPlot = yMinPlot + minSpan;
                    end
                    ylim(app.axPressure, [yMinPlot yMaxPlot]);
                    yticks(app.axPressure, linspace(yMinPlot, yMaxPlot, 5));
                end
            end
        end

        function resetPressureBuffers(app)
            app.pressureTimeBuf = [];
            app.pressureBuf1 = [];
            app.pressureBuf2 = [];
        end

        function checkPressureAlarm(app, val1, val2)
            threshold = str2double(get(app.editPressureThreshold, 'String'));
            if isnan(threshold), threshold = inf; end
            peakVal = max([val1, val2]);
            if peakVal > threshold
                set(app.txtAlarmStatus, 'String', 'ALARM', 'ForegroundColor', [0.8 0 0]);
                if val1 > threshold
                    set(app.txtPressureValue1, 'ForegroundColor', [0.8 0 0]);
                else
                    set(app.txtPressureValue1, 'ForegroundColor', [0 0 0.8]);
                end
                if val2 > threshold
                    set(app.txtPressureValue2, 'ForegroundColor', [0.8 0 0]);
                else
                    set(app.txtPressureValue2, 'ForegroundColor', [0 0.45 0]);
                end
                if ~app.alarmActive
                    app.alarmActive = true;
                    app.appendMessage(sprintf('Pressure alarm triggered: max(%.3f, %.3f) V > %.3f V', val1, val2, threshold));
                end
            else
                set(app.txtAlarmStatus, 'String', 'NORMAL', 'ForegroundColor', [0 0.6 0]);
                set(app.txtPressureValue1, 'ForegroundColor', [0 0 0.8]);
                set(app.txtPressureValue2, 'ForegroundColor', [0 0.45 0]);
                if app.alarmActive
                    app.alarmActive = false;
                    app.appendMessage(sprintf('Pressure alarm cleared: max(%.3f, %.3f) V <= %.3f V', val1, val2, threshold));
                end
            end
        end

        function updatePseudoHeatmap(app, val1, val2)
            if isempty(app.axHeatmap) || ~ishandle(app.axHeatmap)
                return;
            end
            vmin = str2double(get(app.editHeatmapMin, 'String'));
            vmax = str2double(get(app.editHeatmapMax, 'String'));
            if isnan(vmin), vmin = 0.0; end
            if isnan(vmax) || vmax <= vmin, vmax = vmin + 1.0; end

            if isnan(val1), n1 = 0; else, n1 = max(0, min(1, (val1 - vmin) / (vmax - vmin))); end
            if isnan(val2), n2 = 0; else, n2 = max(0, min(1, (val2 - vmin) / (vmax - vmin))); end

            img = zeros(40, 80);
            img(:, 1:40) = n1;
            img(:, 41:80) = n2;
            set(app.heatmapImg, 'CData', img);
            title(app.axHeatmap, sprintf('Pressure Intensity | S1 %.1f%%  S2 %.1f%%', n1 * 100, n2 * 100));
            drawnow limitrate;
        end

        function handleJoystickSwitchEdges(app)
            sw = app.latestJoySW;
            for i = 1:3
                if sw(i) == 1 && app.lastJoySW(i) == 0
                    switch i
                        case 1
                            app.transSpeedIdx = app.transSpeedIdx + 1;
                            if app.transSpeedIdx > numel(app.transSpeedLevels), app.transSpeedIdx = 1; end
                            app.appendMessage(sprintf('Translation speed switched to %.1f mm/s', app.transSpeedLevels(app.transSpeedIdx)));
                        case 2
                            app.rotSpeedIdx = app.rotSpeedIdx + 1;
                            if app.rotSpeedIdx > numel(app.rotSpeedLevels), app.rotSpeedIdx = 1; end
                            app.appendMessage(sprintf('Rotation speed switched to %.1f deg/s', app.rotSpeedLevels(app.rotSpeedIdx)));
                        case 3
                            app.estopLatched = ~app.estopLatched;
                            if app.estopLatched
                                app.zeroAllTeleopKeys();
                                app.appendMessage('E-STOP latched by joystick SW3.');
                            else
                                app.appendMessage('E-STOP cleared by joystick SW3.');
                            end
                    end
                end
            end
            app.lastJoySW = sw;
        end

        function [linJoy, rotJoy, labels] = computeJoystickVelocity(app)
            linJoy = [0 0 0];
            rotJoy = [0 0 0];
            labels = {'IDLE','IDLE','IDLE'};
            if app.estopLatched
                return;
            end
            dz = str2double(get(app.editJoyDeadzone, 'String'));
            if isnan(dz) || dz < 0
                dz = app.joyDeadzoneV;
            else
                app.joyDeadzoneV = dz;
            end
            tv = app.transSpeedLevels(app.transSpeedIdx);
            rv = app.rotSpeedLevels(app.rotSpeedIdx);
            for i = 1:3
                x = app.latestJoyX(i);
                y = app.latestJoyY(i);
                state = 'IDLE';
                if ~isnan(x)
                    dx = x - app.joyCenterV;
                    if dx > dz
                        linJoy(i) = tv;
                        state = 'FWD';
                    elseif dx < -dz
                        linJoy(i) = -tv;
                        state = 'REV';
                    end
                end
                if ~isnan(y)
                    dy = y - app.joyCenterV;
                    if dy > dz
                        rotJoy(i) = rv;
                        if strcmp(state,'IDLE'), state = 'CW'; else, state = [state '+CW']; end
                    elseif dy < -dz
                        rotJoy(i) = -rv;
                        if strcmp(state,'IDLE'), state = 'CCW'; else, state = [state '+CCW']; end
                    end
                end
                labels{i} = state;
            end
        end

        function updateJoystickStatus(app)
            tv = app.transSpeedLevels(app.transSpeedIdx);
            rv = app.rotSpeedLevels(app.rotSpeedIdx);
            set(app.txtSpeedModes, 'String', sprintf('Speeds: Trans=%.1f mm/s | Rot=%.1f deg/s', tv, rv));
            if app.estopLatched
                set(app.txtEStop, 'String', 'E-STOP: ON', 'ForegroundColor', [0.8 0 0]);
            else
                set(app.txtEStop, 'String', 'E-STOP: OFF', 'ForegroundColor', [0 0.6 0]);
            end
            [~,~,labels] = app.computeJoystickVelocity();
            x = app.latestJoyX; y = app.latestJoyY; sw = app.latestJoySW;
            for i = 1:3
                if isnan(x(i)), xs='--'; else, xs=sprintf('%.2f',x(i)); end
                if isnan(y(i)), ys='--'; else, ys=sprintf('%.2f',y(i)); end
                msg = sprintf('Joy%d -> Tube%d | X=%s Y=%s SW=%d | %s', i, i, xs, ys, sw(i), labels{i});
                switch i
                    case 1, set(app.txtJoy1, 'String', msg);
                    case 2, set(app.txtJoy2, 'String', msg);
                    case 3, set(app.txtJoy3, 'String', msg);
                end
            end
        end

        %% ===== Timer / utility =====
        function onTimerTick(app)
            try
                if app.isConnected && ~isempty(app.driveBot)
                    app.runPseudoPMCStep();
                    app.refreshCurrentPoseDisplay();
                    app.updateRobotPlots();
                end
                if app.sensorConnected && ~isempty(app.sensorSerial)
                    app.pollSensor();
                end
                drawnow limitrate;
            catch ME
                app.appendMessage(['Timer error: ' ME.message]);
            end
        end

        function stopTimerIfIdle(app)
            if isempty(app.timerObj) || ~isvalid(app.timerObj)
                return;
            end
            if ~app.isConnected && ~app.sensorConnected && strcmp(app.timerObj.Running, 'on')
                stop(app.timerObj);
            end
        end

        function appendMessage(app, msg)
            try
                old = get(app.listMsg, 'String');
                if ischar(old), old = {old}; end
                stamp = datestr(now, 'HH:MM:SS');
                old{end+1} = sprintf('[%s] %s', stamp, msg);
                if numel(old) > 300
                    old = old(end-299:end);
                end
                set(app.listMsg, 'String', old, 'Value', numel(old));
            catch
            end
        end

        function onClose(app)
            delete(app);
        end
    end
end
