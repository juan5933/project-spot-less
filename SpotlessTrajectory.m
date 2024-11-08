classdef SpotlessTrajectory < handle
    % This class manages the robot's movement trajectory for picking up and
    % dropping off plates, including inverse kinematics computation, error
    % tolerance checks, and robot animation.

    properties
        robot1              % First robot instance
        robot2              % Second robot instance
        stepsSixteen        % Number of steps for smooth trajectory for robot1
        stepsThree          % Number of steps for smooth trajectory for robot2
        qStartSixteen       % Initial joint configuration for robot1
        qStartThree         % Initial joint configuration for robot2
        plates              % Instance of the Plates class for plate positions
        qEndSixteen         % Final joint configuration for robot1 from IK
        qEndThree           % Final joint configuration for robot2
        transformedVertices % Transformed plate vertices during movement
        plateVertices       % Original plate vertices
        plates_h            % Handles to the graphical plate objects
        app                 % GUI application interface
        eStopFlag           % Emergency stop flag
        arduinoObj          % Arduino object for serial communication
    end

    methods
        function self = SpotlessTrajectory(robot2, robot1, plates, app)
            % Constructor: Initializes robots, steps, configurations, and plate data
            self.robot1 = robot1;  % Assign first robot instance
            self.robot2 = robot2;  % Assign second robot instance
            self.stepsSixteen = 60;  % Set steps for smooth trajectory for robot1
            self.stepsThree = 60;  % Set steps for smooth trajectory for robot2
            self.qStartSixteen = [0 -135*pi/180 pi/4 pi/2 0 0];  % Initial joint config for robot1
            self.qStartThree = [-pi -pi/2 0 pi 0 0];  % Initial joint config for robot2
            self.plates = plates;  % Assign plates instance
            self.app = app;  % Assign GUI application
            self.eStopFlag = 0;  % Set emergency stop flag to inactive
        end

        function Run(self)
            % Executes the full trajectory for picking up and placing plates
            disp("STATUS: INITIALIZING ACTION");
            fprintf('\n');
            self.arduinoObj = serialport("COM3", 9600);  % Initialize serial port for Arduino
            configureTerminator(self.arduinoObj, "LF");  % Configure line feed terminator

            self.StandbyMode();  % Set initial standby mode

            % Iterate through all plates for pick-and-place actions
            for plateIndex = 1:3
                self.InfrontOfTap();  % Move to the initial position above the tap
                self.AbovePickPointPlateless();  % Position above the plate before pickup
                self.PickRotatedUp(plateIndex);  % Pick up the plate
                self.AbovePickPointPlate(plateIndex);  % Return to position above the pickup point
                self.RinsingRotatedUp(plateIndex);  % Move to rinsing position
                self.RinsingRotatedDown(plateIndex);  % Lower for rinsing action
                self.UR3eRotatedDown(plateIndex);  % Adjust robot2 to a rotated down state

                % Perform position checks and move in sequential actions
                self.StandbyMode();
                self.POC1();
                self.POC2();
                self.POC3();
                self.StandbyMode();

                self.UR3eRotatedUp(plateIndex);

                self.POC1();
                self.POC2();
                self.POC3();
                self.StandbyMode();

                % Continue with rinsing and returning home actions
                self.RinsingRotatedUp(plateIndex);
                self.RinsingRotatedDown(plateIndex);
                self.UR3eRotatedDown(plateIndex);
                self.UR3eRotatedUp(plateIndex);
                self.ReturnHome();

                % Move to the drop-off position and place the plate
                self.AboveSensorPointPlate(plateIndex);
                self.HeadingToDropPlate(plateIndex);
                self.AboveDropPointPlate(plateIndex);
                self.DropRotatedUp(plateIndex);

                % Final adjustments after dropping the plate
                self.AboveDropPointPlateless();
                self.HeadingToDropPlateless();
                self.AboveSensorPointPlateless();
            end

            self.Home();  % Return to the home position after completing the task
            disp("STATUS: ACTION FINISHED");
            fprintf('\n');
        end

        function InfrontOfTap(self)
            % Move to a waypoint in front of the tap position
            disp("STATUS: MOVING TO HOME POSITION");
            fprintf('\n');

            t1 = transl([-1.01 0.394 1.668]) * rpy2tr(90, 0, 180, 'deg');  % Define target pose
            self.qEndSixteen = self.robot1.model.ikine(t1);  % Compute joint configuration

            % Generate and animate the trajectory
            s = lspb(0, 1, self.stepsSixteen);  % Time vector for trajectory
            path1 = nan(self.stepsSixteen, 6);  % Preallocate path array

            % Animate each step and handle emergency stop checks
            for j = 1:self.stepsSixteen
                path1(j, :) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;  % Interpolate joint config

                data = readline(self.arduinoObj);  % Read data from Arduino
                Estop = int32(str2double(data));  % Convert data to integer

                if self.app.eStopFlag == 1 || Estop == 1
                    currentstep = self.robot1.model.getpos();  % Get current joint position
                    self.robot1.model.plot(currentstep);  % Hold position
                else
                    self.robot1.model.animate(path1(j, :));  % Animate robot
                end

                axis equal;  % Maintain axis proportions
                drawnow();  % Refresh plot
                pause(0.01);  % Control animation speed
            end

            % Update starting configuration for the next movement
            self.qStartSixteen = self.qEndSixteen;
        end

        function AbovePickPointPlateless(self)
            % Move above the pickup point without a plate
            disp("STATUS: MOVING TO HOME POSITION");
            fprintf('\n');

            t1 = transl([-1.2, 0.4, 1.668]) * rpy2tr(90, 0, 180, 'deg');  % Define target pose
            self.qEndSixteen = self.robot1.model.ikine(t1);  % Compute joint configuration

            % Generate and animate the trajectory
            s = lspb(0, 1, self.stepsSixteen);
            path1 = nan(self.stepsSixteen, 6);  % Preallocate path array

            for j = 1:self.stepsSixteen
                path1(j, :) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;  % Interpolate joint config

                data = readline(self.arduinoObj);
                Estop = int32(str2double(data));

                if self.app.eStopFlag == 1 || Estop == 1
                    currentstep = self.robot1.model.getpos();
                    self.robot1.model.plot(currentstep);
                else
                    self.robot1.model.animate(path1(j, :));  % Animate robot
                end

                axis equal;
                drawnow();
                pause(0.01);
            end

            % Update starting configuration for the next movement
            self.qStartSixteen = self.qEndSixteen;
        end


function RinsingRotatedUp(self, plateIndex)
    % Move the robot to a waypoint above the plate for rinsing.
    disp("STATUS: MOVING TO HOME POSITION")
    fprintf('\n');

    % Define the transformation matrix for the target pose.
    t1 = transl(-1.065, 0.7, 1.5) * rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0,0,0,0,0,0];
    self.qEndSixteen = self.robot1.model.ikine(t1);

    % Generate a smooth trajectory using LSPB.
    s = lspb(0, 1, self.stepsSixteen);  % Time vector for trajectory.
    path1 = nan(self.stepsSixteen, 6);  % Preallocate path array.

    % Animate the robot through each step.
    for j = 1:self.stepsSixteen
        path1(j, :) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;

        data = readline(self.arduinoObj);  % Read data from Arduino.
        Estop = int32(str2double(data));  % Convert data to integer.

        if self.app.eStopFlag == 1 || Estop == 1
            currentstep = self.robot1.model.getpos();  % Get current joint positions.
            self.robot1.model.plot(currentstep);  % Hold the robot at current position.

            % Transform and update plate vertices.
            tr = self.robot1.model.fkine(currentstep);
            rotation = trotx(-pi/2);
            offset = transl(0, -0.1, 0);
            trEdgeGrasp = tr.T * rotation * offset;
            self.transformedVertices = [self.plates.plateVertices{plateIndex}, ones(size(self.plates.plateVertices{plateIndex},1), 1)] * trEdgeGrasp';
            set(self.plates.plates_h{plateIndex}, 'Vertices', self.transformedVertices(:, 1:3));
        else
            % Animate robot movement.
            self.robot1.model.animate(path1(j, :));

            % Update plate vertices during movement.
            tr = self.robot1.model.fkine(path1(j, :));
            rotation = trotx(-pi/2);
            offset = transl(0, -0.1, 0);
            trEdgeGrasp = tr.T * rotation * offset;
            self.transformedVertices = [self.plates.plateVertices{plateIndex}, ones(size(self.plates.plateVertices{plateIndex},1), 1)] * trEdgeGrasp';
            set(self.plates.plates_h{plateIndex}, 'Vertices', self.transformedVertices(:, 1:3));
        end

        axis equal;  % Maintain axis proportions.
        drawnow();  % Update plot.
        pause(0.01);  % Control animation speed.
    end

    % Update starting joint configuration for the next movement.
    self.qStartSixteen = self.qEndSixteen;
end

function RinsingRotatedDown(self, plateIndex)
    % Move the robot down to a lower rinsing position.
    disp("STATUS: MOVING TO HOME POSITION")
    fprintf('\n');

    % Update end joint configuration with rotation.
    self.qEndSixteen = self.qStartSixteen + deg2rad([0, 0, 0, 0, 0, 180]);

    % Generate trajectory and animate the robot.
    s = lspb(0, 1, self.stepsSixteen);
    path1 = nan(self.stepsSixteen, 6);

    for j = 1:self.stepsSixteen
        path1(j, :) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;

        data = readline(self.arduinoObj);
        Estop = int32(str2double(data));

        if self.app.eStopFlag == 1 || Estop == 1
            currentstep = self.robot1.model.getpos();
            self.robot1.model.plot(currentstep);
        else
            self.robot1.model.animate(path1(j, :));
        end

        % Update plate vertices during movement.
        tr = self.robot1.model.fkine(path1(j, :));
        rotation = trotx(-pi/2);
        offset = transl(0, -0.1, 0);
        trEdgeGrasp = tr.T * rotation * offset;
        self.transformedVertices = [self.plates.plateVertices{plateIndex}, ones(size(self.plates.plateVertices{plateIndex},1), 1)] * trEdgeGrasp';
        set(self.plates.plates_h{plateIndex}, 'Vertices', self.transformedVertices(:, 1:3));

        drawnow();
    end

    % Update starting configuration for the next move.
    self.qStartSixteen = self.qEndSixteen + deg2rad([0, 0, 0, 0, 0, 0]);
end

function UR3eRotatedDown(self, plateIndex)
    % Rotate robot2 down to a specified position.
    disp("STATUS: MOVING TO HOME POSITION")
    fprintf('\n');

    % Define the target pose and compute joint configuration.
    t1 = transl(-0.8, 0.7800, 1.4) * rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0, 0, 0, 0, 0, 0];
    self.qEndSixteen = self.robot1.model.ikine(t1) + deg2rad([0, 0, 0, 0, 0, 180]);

    % Generate trajectory and animate the robot.
    s = lspb(0, 1, self.stepsSixteen);
    path1 = nan(self.stepsSixteen, 6);

    for j = 1:self.stepsSixteen
        path1(j, :) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;

        data = readline(self.arduinoObj);
        Estop = int32(str2double(data));

        if self.app.eStopFlag == 1 || Estop == 1
            currentstep = self.robot1.model.getpos();
            self.robot1.model.plot(currentstep);
        else
            self.robot1.model.animate(path1(j, :));
        end

        % Update plate vertices during movement.
        tr = self.robot1.model.fkine(path1(j, :));
        rotation = trotx(-pi/2);
        offset = transl(0, -0.1, 0);
        trEdgeGrasp = tr.T * rotation * offset;
        self.transformedVertices = [self.plates.plateVertices{plateIndex}, ones(size(self.plates.plateVertices{plateIndex},1), 1)] * trEdgeGrasp';
        set(self.plates.plates_h{plateIndex}, 'Vertices', self.transformedVertices(:, 1:3));

        axis equal;
        drawnow();
        pause(0.01);
    end

    % Update starting configuration for the next move.
    self.qStartSixteen = self.qEndSixteen + deg2rad([0, 0, 0, 0, 0, 0]);
end

function UR3eRotatedUp(self, plateIndex)
    % Rotate robot2 up to a specified position.
    disp("STATUS: MOVING TO HOME POSITION")
    fprintf('\n');

    % Update end joint configuration with rotation.
    self.qEndSixteen = self.qStartSixteen + deg2rad([0, 0, 0, 0, 0, -180]);

    % Generate trajectory and animate the robot.
    s = lspb(0, 1, self.stepsSixteen);
    path1 = nan(self.stepsSixteen, 6);

    for j = 1:self.stepsSixteen
        path1(j, :) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;

        data = readline(self.arduinoObj);
        Estop = int32(str2double(data));

        if self.app.eStopFlag == 1 || Estop == 1
            currentstep = self.robot1.model.getpos();
            self.robot1.model.plot(currentstep);
        else
            self.robot1.model.animate(path1(j, :));
        end

        % Update plate vertices during movement.
        tr = self.robot1.model.fkine(path1(j, :));
        rotation = trotx(-pi/2);
        offset = transl(0, -0.1, 0);
        trEdgeGrasp = tr.T * rotation * offset;
        self.transformedVertices = [self.plates.plateVertices{plateIndex}, ones(size(self.plates.plateVertices{plateIndex},1), 1)] * trEdgeGrasp';
        set(self.plates.plates_h{plateIndex}, 'Vertices', self.transformedVertices(:, 1:3));

        axis equal;
        drawnow();
        pause(0.01);
    end

    % Update starting configuration for the next move.
    self.qStartSixteen = self.qEndSixteen + deg2rad([0, 0, 0, 0, 0, 0]);
end












        function AboveSensorPointPlate(self, plateIndex)
            % Move to the first waypoint above the plate pickup point.
            disp("STATUS: MOVING TO HOME POSITION")
            fprintf('\n');

            % Define the desired transformation matrix (pose) above the pickup point

            % t1 = transl(-0.683, 0.7, 1.8)*rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0,0,0,0,0,0];
            % self.qEndSixteen = self.robot1.model.ikine(t1) + deg2rad([0,0,0,0,0,0]);

            self.qEndSixteen = self.qStartSixteen + deg2rad([-50,45,-30,0,0,0]);

            % Generate a smooth trajectory between the current and desired joint configurations
            s = lspb(0, 1, self.stepsSixteen);  % Generate a time vector using linear segment with parabolic blends (LSPB)
            path1 = nan(self.stepsSixteen, 6);  % Preallocate the path array for the trajectory

            % Interpolate joint angles between qStart and qEnd for each step
            % for j = 1:self.stepsSixteen
            %     path1(j, :) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;  % Linear interpolation for each joint angle
            % end

            % Animate the robot and gripper along the interpolated trajectory
            for j = 1:self.stepsSixteen

                path1(j, :) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;  % Linear interpolation for each joint angle

                data = readline(self.arduinoObj);  % Read line from the Arduino serial port
                Estop = int32(str2double(data));  % Convert to integer if valid

                if self.app.eStopFlag == 1 || Estop == 1

                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position

                else
                    % Animate the robot movement for the j-th step
                    self.robot1.model.animate(path1(j, :));
                end

                tr = self.robot1.model.fkine(path1(j,:));

                rotation = trotx(-pi/2);

                offset = transl(0,-0.1,0);

                trEdgeGrasp = tr.T * rotation * offset;

                self.transformedVertices = [self.plates.plateVertices{plateIndex}, ones(size(self.plates.plateVertices{plateIndex},1),1)] * trEdgeGrasp';

                set(self.plates.plates_h{plateIndex}, 'Vertices', self.transformedVertices(:,1:3));

                axis equal  % Keep equal axis proportions for proper visualization
                drawnow();  % Update the figure window in real-time
                pause(0.01);  % Add a small pause to control the animation speed

            end

            % Update the starting joint configuration to the final configuration for the next motion
            self.qStartSixteen = self.qEndSixteen+ deg2rad([0,0,0,0,0,0]);
        end



function AboveSensorPointPlateless(self)
    % Move the robot to a position above the sensor point without a plate.
    disp("STATUS: MOVING TO HOME POSITION")
    fprintf('\n');

    % Define the target transformation matrix for the sensor point.
    t1 = transl(-0.683, 0.7, 1.668) * rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0,0,0,0,0,0];
    self.qEndSixteen = self.robot1.model.ikine(t1);

    % Generate a trajectory using LSPB for smooth motion.
    s = lspb(0, 1, self.stepsSixteen);
    path1 = nan(self.stepsSixteen, 6);

    % Animate the robot along the interpolated trajectory.
    for j = 1:self.stepsSixteen
        path1(j, :) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;
        data = readline(self.arduinoObj);  % Read data from Arduino.
        Estop = int32(str2double(data));

        if self.app.eStopFlag == 1 || Estop == 1
            currentstep = self.robot1.model.getpos();  % Get current position.
            self.robot1.model.plot(currentstep);  % Hold at current position.
        else
            self.robot1.model.animate(path1(j, :));  % Animate movement.
        end

        axis equal;  % Maintain axis proportions.
        drawnow();  % Update the figure.
        pause(0.01);  % Control animation speed.
    end

    % Update the starting configuration for the next motion.
    self.qStartSixteen = self.qEndSixteen;
end

function HeadingToDropPlate(self, plateIndex)
    % Move the robot to the drop-off waypoint for the plate.
    disp("STATUS: MOVING TO HOME POSITION")
    fprintf('\n');

    % Define the transformation matrix for the drop-off waypoint.
    t1 = transl(0, 0.5, 1.6) * rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0,0,0,0,0,0];
    self.qEndSixteen = self.robot1.model.ikine(t1);

    % Generate the trajectory using LSPB.
    s = lspb(0, 1, self.stepsSixteen);
    path1 = nan(self.stepsSixteen, 6);

    % Animate the robot along the trajectory.
    for j = 1:self.stepsSixteen
        path1(j, :) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;
        data = readline(self.arduinoObj);
        Estop = int32(str2double(data));

        if self.app.eStopFlag == 1 || Estop == 1
            currentstep = self.robot1.model.getpos();
            self.robot1.model.plot(currentstep);
        else
            self.robot1.model.animate(path1(j, :));
        end

        % Adjust the plate's vertices during the movement.
        tr = self.robot1.model.fkine(path1(j, :));
        rotation = trotx(-pi/2);
        offset = transl(0, -0.1, 0);
        trEdgeGrasp = tr.T * rotation * offset;
        self.transformedVertices = [self.plates.plateVertices{plateIndex}, ones(size(self.plates.plateVertices{plateIndex}, 1), 1)] * trEdgeGrasp';
        set(self.plates.plates_h{plateIndex}, 'Vertices', self.transformedVertices(:, 1:3));

        axis equal;
        drawnow();
        pause(0.01);
    end

    % Update the starting configuration for the next move.
    self.qStartSixteen = self.qEndSixteen;
end

function HeadingToDropPlateless(self)
    % Move the robot to the drop-off waypoint without a plate.
    disp("STATUS: MOVING TO HOME POSITION")
    fprintf('\n');

    % Define the transformation matrix for the drop-off waypoint.
    t1 = transl(-0.3, 0.35, 1.6) * rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0,0,0,0,0,0];
    self.qEndSixteen = self.robot1.model.ikine(t1);

    % Generate the trajectory using LSPB.
    s = lspb(0, 1, self.stepsSixteen);
    path1 = nan(self.stepsSixteen, 6);

    % Animate the robot along the trajectory.
    for j = 1:self.stepsSixteen
        path1(j, :) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;
        data = readline(self.arduinoObj);
        Estop = int32(str2double(data));

        if self.app.eStopFlag == 1 || Estop == 1
            currentstep = self.robot1.model.getpos();
            self.robot1.model.plot(currentstep);
        else
            self.robot1.model.animate(path1(j, :));
        end

        axis equal;
        drawnow();
        pause(0.01);
    end

    % Update the starting configuration for the next move.
    self.qStartSixteen = self.qEndSixteen;
end

function PickRotatedUp(self, plateIndex)
    % Move to the plate's position and pick it up.
    disp("STATUS: MOVING TO PLATE");
    fprintf('\n');

    % Define the transformation for the plate's position.
    t2 = transl([self.plates.platesGrab(plateIndex, :)]) * rpy2tr(90, 0, 180, 'deg');
    self.qEndSixteen = self.robot1.model.ikcon(t2, self.qStartSixteen);

    % Generate the trajectory using LSPB.
    s = lspb(0, 1, self.stepsSixteen);
    path2 = nan(self.stepsSixteen, 6);

    % Animate the robot along the trajectory.
    for j = 1:self.stepsSixteen
        path2(j, :) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;
        data = readline(self.arduinoObj);
        Estop = int32(str2double(data));

        if self.app.eStopFlag == 1 || Estop == 1
            currentstep = self.robot1.model.getpos();
            self.robot1.model.plot(currentstep);
        else
            self.robot1.model.animate(path2(j, :));
        end

        axis equal;
        drawnow();
        pause(0.01);
    end

    % Update the starting configuration for the next move.
    self.qStartSixteen = self.qEndSixteen;
end

function AboveDropPointPlate(self, plateIndex)
    % Move above the drop-off point for the plate.
    disp("STATUS: MOVING TO DROP OFF ZONE");
    fprintf('\n');

    % Define the transformation matrix for the drop-off point.
    t3 = transl(0.05, 0.35, 1.4) * rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0,0,0,0,0,0];
    self.qEndSixteen = self.robot1.model.ikine(t3);

    % Generate the trajectory using LSPB.
    s = lspb(0, 1, self.stepsSixteen);
    path3 = nan(self.stepsSixteen, 6);

    % Animate the robot along the trajectory.
    for j = 1:self.stepsSixteen
        path3(j, :) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;
        data = readline(self.arduinoObj);
        Estop = int32(str2double(data));

        if self.app.eStopFlag == 1 || Estop == 1
            currentstep = self.robot1.model.getpos();
            self.robot1.model.plot(currentstep);
        else
            self.robot1.model.animate(path3(j, :));
        end

        % Update the plate's position during the movement.
        tr = self.robot1.model.fkine(path3(j, :));
        rotation = trotx(-pi/2);
        offset = transl(0, -0.1, 0);
        trEdgeGrasp = tr.T * rotation * offset;
        self.transformedVertices = [self.plates.plateVertices{plateIndex}, ones(size(self.plates.plateVertices{plateIndex}, 1), 1)] * trEdgeGrasp';
        set(self.plates.plates_h{plateIndex}, 'Vertices', self.transformedVertices(:, 1:3));

        drawnow();
    end

    % Update the starting configuration for the next move.
    self.qStartSixteen = self.qEndSixteen;
end

function AboveDropPointPlateless(self)
    % Move above the drop-off point without a plate.
    disp("STATUS: MOVING TO DROP OFF ZONE");
    fprintf('\n');

    % Define the transformation matrix for the drop-off point.
    t3 = transl(0.05, 0.35, 1.4) * rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0,0,0,0,0,0];
    self.qEndSixteen = self.robot1.model.ikine(t3);

    % Generate the trajectory using LSPB.
    s = lspb(0, 1, self.stepsSixteen);
    path3 = nan(self.stepsSixteen, 6);

    % Animate the robot along the trajectory.
    for j = 1:self.stepsSixteen
        path3(j, :) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;
        data = readline(self.arduinoObj);
        Estop = int32(str2double(data));

        if self.app.eStopFlag == 1 || Estop == 1
            currentstep = self.robot1.model.getpos();
            self.robot1.model.plot(currentstep);
        else
            self.robot1.model.animate(path3(j, :));
        end

        drawnow();
    end

    % Update the starting configuration for the next move.
    self.qStartSixteen = self.qEndSixteen;
end

function DropRotatedUp(self, plateIndex)
    % Move to the drop-off position and release the plate.
    disp("STATUS: PLACING PLATE AT DROP OFF ZONE");
    fprintf('\n');

    % Define the transformation matrix for placing the plate.
    t4 = transl([self.plates.platesEnd(plateIndex, :)]) * rpy2tr(90, 0, 90, 'deg'), 'q0', self.qStartSixteen, 'mask', [0,0,0,0,0,0];
    self.qEndSixteen = self.robot1.model.ikine(t4);

    % Generate the trajectory using LSPB.
    s = lspb(0, 1, self.stepsSixteen);
    path4 = nan(self.stepsSixteen, 6);

    % Animate the robot along the trajectory.
    for j = 1:self.stepsSixteen
        path4(j, :) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;
        data = readline(self.arduinoObj);
        Estop = int32(str2double(data));

        if self.app.eStopFlag == 1 || Estop == 1
            currentstep = self.robot1.model.getpos();
            self.robot1.model.plot(currentstep);

            % Update plate's position at the current configuration.
            tr = self.robot1.model.fkine(path4(j, :));
            rotation = trotx(-pi/2);
            offset = transl(0, -0.1, 0);
            trEdgeGrasp = tr.T * rotation * offset;
            self.transformedVertices = [self.plates.plateVertices{plateIndex}, ones(size(self.plates.plateVertices{plateIndex}, 1), 1)] * trEdgeGrasp';
            set(self.plates.plates_h{plateIndex}, 'Vertices', self.transformedVertices(:, 1:3));
        else
            self.robot1.model.animate(path4(j, :));

            % Update plate's position during the movement.
            tr = self.robot1.model.fkine(path4(j, :));
            rotation = trotx(-pi/2);
            offset = transl(0, -0.1, 0);
            trEdgeGrasp = tr.T * rotation * offset;
            self.transformedVertices = [self.plates.plateVertices{plateIndex}, ones(size(self.plates.plateVertices{plateIndex}, 1), 1)] * trEdgeGrasp';
            set(self.plates.plates_h{plateIndex}, 'Vertices', self.transformedVertices(:, 1:3));
        end

        axis equal;
        drawnow();
        pause(0.01);
    end

    % Update the starting configuration for the next move.
    self.qStartSixteen = self.qEndSixteen;
end

function Home(self)
    % Return the robot to its home position.
    disp("STATUS: RETURNING HOME");
    fprintf('\n');

    % Define the transformation matrix for the home position.
    t5 = transl(0.05, 0.603, 1.8) * rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0,0,0,0,0,0];
    self.qEndSixteen = self.robot1.model.ikine(t5);

    % Generate the trajectory using LSPB.
    s = lspb(0, 1, self.stepsSixteen);
    path5 = nan(self.stepsSixteen, 6);

    % Animate the robot along the trajectory.
    for j = 1:self.stepsSixteen
        path5(j, :) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;
        data = readline(self.arduinoObj);
        Estop = int32(str2double(data));

        if self.app.eStopFlag == 1 || Estop == 1
            currentstep = self.robot1.model.getpos();
            self.robot1.model.plot(currentstep);
        else
            self.robot1.model.animate(path5(j, :));
        end

        axis equal;
        drawnow();
        pause(0.01);
    end

    % Update the starting configuration for the next move.
    self.qStartSixteen = self.qEndSixteen;
end

function StandbyMode(self)
    % Move the robot to standby mode.
    disp("STATUS: MOVING TO HOME POSITION")
    fprintf('\n');

    % Define the target transformation matrix for standby mode.
    t1 = transl([-0.8, 0.7800, 1.8]) * rpy2tr(180, 0, 0, 'deg');
    self.qEndThree = self.robot2.model.ikcon(t1);

    % Generate the trajectory using LSPB.
    s = lspb(0, 1, self.stepsThree);
    path1 = nan(self.stepsThree, 6);

    % Animate the robot along the trajectory.
    for j = 1:self.stepsThree
        path1(j, :) = (1-s(j)) * self.qStartThree + s(j) * self.qEndThree;
        data = readline(self.arduinoObj);
        Estop = int32(str2double(data));

        if self.app.eStopFlag == 1 || Estop == 1
            currentstep = self.robot1.model.getpos();
            self.robot1.model.plot(currentstep);
        else
            self.robot2.model.animate(path1(j, :));
        end

        axis equal;
        drawnow();
        pause(0.01);
    end

    % Update the starting configuration for the next move.
    self.qStartThree = self.qEndThree;
end




     function POC1(self)
    % Move to the first designated drop-off zone
    disp("STATUS: MOVING TO DROP OFF ZONE");
    fprintf('\n');  % Print newline for cleaner output

    % Define the transformation matrix for the drop-off point
    t3 = transl([-0.8 0.7800 1.45]) * rpy2tr(180,0,0,'deg');
    self.qEndThree = self.robot2.model.ikcon(t3);  % Compute joint angles using inverse kinematics

    % Generate a smooth trajectory using LSPB
    s = lspb(0, 1, self.stepsThree);
    path3 = nan(self.stepsThree, 6);  % Preallocate trajectory array

    % Animate the robot's movement along the computed trajectory
    for j = 1:self.stepsThree
        path3(j,:) = (1-s(j)) * self.qStartThree + s(j) * self.qEndThree;  % Interpolate joint positions

        data = readline(self.arduinoObj);  % Read data from Arduino
        Estop = int32(str2double(data));  % Convert to integer if valid

        if self.app.eStopFlag == 1 || Estop == 1
            % Emergency stop: hold the robot in current position
            currentstep = self.robot1.model.getpos();
            self.robot1.model.plot(currentstep);
        else
            % Animate robot to the next step
            self.robot2.model.animate(path3(j, :));
        end

        axis equal;  % Maintain equal axis proportions for visualization
        drawnow();  % Update the plot in real-time
        pause(0.01);  % Control animation speed
    end

    % Update starting joint configuration for the next move
    self.qStartThree = self.qEndThree;
end

function POC2(self)
    % Move to the second designated drop-off zone
    disp("STATUS: MOVING TO DROP OFF ZONE");
    fprintf('\n');

    % Define the transformation matrix for the second drop-off point
    t3 = transl([-0.9 0.7800 1.45]) * rpy2tr(180,0,0,'deg');
    self.qEndThree = self.robot2.model.ikcon(t3);

    % Generate a smooth trajectory using LSPB
    s = lspb(0, 1, self.stepsThree);
    path3 = nan(self.stepsThree, 6);

    % Animate the robot's movement along the computed trajectory
    for j = 1:self.stepsThree
        path3(j,:) = (1-s(j)) * self.qStartThree + s(j) * self.qEndThree;

        data = readline(self.arduinoObj);
        Estop = int32(str2double(data));

        if self.app.eStopFlag == 1 || Estop == 1
            currentstep = self.robot1.model.getpos();
            self.robot1.model.plot(currentstep);
        else
            self.robot2.model.animate(path3(j, :));
        end

        axis equal;
        drawnow();
        pause(0.01);
    end

    % Update starting joint configuration for the next move
    self.qStartThree = self.qEndThree;
end

function POC3(self)
    % Move to the third designated drop-off zone
    disp("STATUS: MOVING TO DROP OFF ZONE");
    fprintf('\n');

    % Define the transformation matrix for the third drop-off point
    t3 = transl([-0.7 0.900 1.45]) * rpy2tr(180,0,0,'deg');
    self.qEndThree = self.robot2.model.ikcon(t3);

    % Generate a smooth trajectory using LSPB
    s = lspb(0, 1, self.stepsThree);
    path3 = nan(self.stepsThree, 6);

    % Animate the robot's movement along the computed trajectory
    for j = 1:self.stepsThree
        path3(j,:) = (1-s(j)) * self.qStartThree + s(j) * self.qEndThree;

        data = readline(self.arduinoObj);
        Estop = int32(str2double(data));

        if self.app.eStopFlag == 1 || Estop == 1
            currentstep = self.robot1.model.getpos();
            self.robot1.model.plot(currentstep);
        else
            self.robot2.model.animate(path3(j, :));
        end

        axis equal;
        drawnow();
        pause(0.01);
    end

    % Update starting joint configuration for the next move
    self.qStartThree = self.qEndThree;
end

function ReturnHome(self)
    % Return the robot to its home position after task completion
    disp("STATUS: RETURNING HOME");
    fprintf('\n');

    % Compute home position configuration
    self.qEndThree = self.qStartThree + deg2rad([0,-30,0,0,0,0]);

    % Generate a smooth trajectory using LSPB
    s = lspb(0, 1, self.stepsThree);
    path5 = nan(self.stepsThree, 6);

    % Animate the robot's movement along the return home trajectory
    for j = 1:self.stepsThree
        path5(j,:) = (1-s(j)) * self.qStartThree + s(j) * self.qEndThree;

        data = readline(self.arduinoObj);
        Estop = int32(str2double(data));

        if self.app.eStopFlag == 1 || Estop == 1
            currentstep = self.robot1.model.getpos();
            self.robot1.model.plot(currentstep);
        else
            self.robot2.model.animate(path5(j, :));
        end

        axis equal;
        drawnow();
        pause(0.01);
    end

    % Update starting joint configuration for the next move
    self.qStartThree = self.qEndThree;
end



    end
end


