classdef SpotlessTrajectory < handle
    % This class handles the robot's movement trajectory for picking up and
    % dropping off plates. It computes inverse kinematics, checks position error
    % tolerance, and animates both the robot and gripper.

    properties
        robot1              % The robot instance
        robot2
        stepsSixteen             % Number of steps for smooth trajectory
        stepsThree
        qStartSixteen              % Initial joint configuration
        qStartThree
        plates             % Instance of the Plates class for access to plate positions
        qEndSixteen               % Final joint configuration from inverse kinematics
        qEndThree
        transformedVertices
        plateVertices
        plates_h
        app;
        eStopFlag;
        arduinoObj

    end

    methods
        function self = SpotlessTrajectory(robot2, robot1, plates, app)

            % Constructor to initialize the robot, number of steps, starting configuration, plates, and gripper
            self.robot1 = robot1;  % Assign the robot instance
            self.robot2 = robot2;  % Assign the robot instance
            self.stepsSixteen = 60;  % Set the number of steps for trajectory interpolation
            self.stepsThree = 60;
            self.qStartSixteen = [0 -135*pi/180 pi/4 pi/2 0 0];  % Initial joint configuration
            self.qStartThree = [-pi -pi/2 0 pi 0 0];
            self.plates = plates;  % Store the Plates instance
            self.app = app;
            self.eStopFlag = 0;   % zero = E-stop in inactive
        end

        function Run(self)
            % Main function to perform the full trajectory of picking up and dropping off plates.
            disp("STATUS: INITIALIZING ACTION")
            fprintf('\n');
            self.arduinoObj = serialport("COM3", 9600);  % Initialize with port and baud rate
            configureTerminator(self.arduinoObj, "LF");

            self.StandbyMode(); % 3

            % Loop over all plates to pick and place them
            for plateIndex = 1:3

                % self.Home();

                self.InfrontOfTap();
                self.AbovePickPointPlateless();  % Move to the point above the plate's pickup position
                self.PickRotatedUp(plateIndex);  % Move to and pick up the plate
                self.AbovePickPointPlate(plateIndex);  % Return to above the pickup point
                self.RinsingRotatedUp(plateIndex);
                self.RinsingRotatedDown(plateIndex);
                self.UR3eRotatedDown(plateIndex);

                self.StandbyMode(); % 3
                self.POC1();% 3
                self.POC2();% 3
                self.POC3(); % 3
                self.StandbyMode(); % 3

                self.UR3eRotatedUp(plateIndex);

                self.POC1();% 3
                self.POC2();% 3
                self.POC3(); % 3
                self.StandbyMode(); % 3

                self.RinsingRotatedUp(plateIndex);
                self.RinsingRotatedDown(plateIndex);
                self.UR3eRotatedDown(plateIndex);
                self.UR3eRotatedUp(plateIndex);
                self.ReturnHome(); %3

                self.AboveSensorPointPlate(plateIndex);
                self.HeadingToDropPlate(plateIndex);
                self.AboveDropPointPlate(plateIndex);  % Move to the point above the drop-off zone
                self.DropRotatedUp(plateIndex);  % Drop the plate in the designated location

                self.AboveDropPointPlateless();
                self.HeadingToDropPlateless();
                self.AboveSensorPointPlateless();
            end

            self.Home();  % Return the robot to the home position

            disp("STATUS: ACTION FINISHED")
            fprintf('\n');
        end



        function InfrontOfTap(self)
            % Move to the first waypoint above the plate pickup point.
            disp("STATUS: MOVING TO HOME POSITION");
            fprintf('\n');

            t1 = transl([-1.01 0.394 1.668]) * rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [1,1,0,0,0,0];
            self.qEndSixteen = self.robot1.model.ikine(t1);



            % Generate a smooth trajectory between the current and desired joint configurations
            s = lspb(0, 1, self.stepsSixteen);  % Generate a time vector using linear segment with parabolic blends (LSPB)
            path1 = nan(self.stepsSixteen, 6);  % Preallocate the path array for the trajectory

            % Interpolate joint angles between qStart and qEnd for each step
            % for j = 1:self.stepsSixteen
            %     path1(j, :) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;  % Linear interpolation for each joint angle
            % end

            % Animate the robot and gripper along the interpolated trajectory
            for j = 1:self.stepsSixteen


                path1(j, :) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;

                data = readline(self.arduinoObj);  % Read line from the Arduino serial port
                Estop = int32(str2double(data));  % Convert to integer if valid

                if self.app.eStopFlag == 1 || Estop == 1

                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position

                else
                    % Animate the robot movement for the j-th step
                    self.robot1.model.animate(path1(j, :));
                end



                axis equal  % Keep equal axis proportions for proper visualization
                drawnow();  % Update the figure window in real-time
                pause(0.01);  % Add a small pause to control the animation speed

                drawnow();  % Update the figure window to display the text
            end

            % Update the starting joint configuration to the final configuration for the next motion
            self.qStartSixteen = self.qEndSixteen;
        end




        function AbovePickPointPlateless(self)
            % Move to the first waypoint above the plate pickup point.
            disp("STATUS: MOVING TO HOME POSITION");
            fprintf('\n');

            t1 = transl([-1.2, 0.4, 1.668]) * rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0 0 0 0 0 0];
            self.qEndSixteen = self.robot1.model.ikine(t1);

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

                axis equal;
                drawnow();
                pause(0.01);

            end

            % Update the starting joint configuration to the final configuration for the next motion
            self.qStartSixteen = self.qEndSixteen;
        end





        function AbovePickPointPlate(self, plateIndex)
            % Move to the first waypoint above the plate pickup point.
            disp("STATUS: MOVING TO HOME POSITION");
            fprintf('\n');

            t1 = transl([-1.2, 0.4, 1.668]) * rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0 0 0 0 0 0];
            self.qEndSixteen = self.robot1.model.ikine(t1);

            % Generate a smooth trajectory between the current and desired joint configurations
            s = lspb(0, 1, self.stepsSixteen);  % Generate a time vector using linear segment with parabolic blends (LSPB)
            path1 = nan(self.stepsSixteen, 6);  % Preallocate the path array for the trajectory

            % Interpolate joint angles between qStart and qEnd for each step
            % for j = 1:self.stepsSixteen
            %     path1(j, :) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;  % Linear interpolation for each joint angle
            % end

            % Animate the robot and gripper along the interpolated trajectory
            for j = 1:self.stepsSixteen


                % animatedStep = self.stepsSixteen;

                path1(j, :) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;  % Linear interpolation for each joint angle

                data = readline(self.arduinoObj);  % Read line from the Arduino serial port
                Estop = int32(str2double(data));  % Convert to integer if valid

                if self.app.eStopFlag == 1 || Estop == 1

                    pausedStep = self.stepsSixteen;

                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position
                    tr = self.robot1.model.fkine(currentstep);

                    rotation = trotx(-pi/2);

                    offset = transl(0,-0.1,0);

                    trEdgeGrasp = tr.T * rotation * offset;

                    self.transformedVertices = [self.plates.plateVertices{plateIndex}, ones(size(self.plates.plateVertices{plateIndex},1),1)] * trEdgeGrasp';

                    set(self.plates.plates_h{plateIndex}, 'Vertices', self.transformedVertices(:,1:3));

                    self.stepsSixteen = pausedStep;
                else


                    % Animate the robot movement for the j-th step
                    self.robot1.model.animate(path1(j, :));

                    tr = self.robot1.model.fkine(path1(j,:));

                    rotation = trotx(-pi/2);

                    offset = transl(0,-0.1,0);

                    trEdgeGrasp = tr.T * rotation * offset;

                    self.transformedVertices = [self.plates.plateVertices{plateIndex}, ones(size(self.plates.plateVertices{plateIndex},1),1)] * trEdgeGrasp';

                    set(self.plates.plates_h{plateIndex}, 'Vertices', self.transformedVertices(:,1:3));
                end

                axis equal;
                drawnow();
                pause(0.01);
            end

            % Update the starting joint configuration to the final configuration for the next motion
            self.qStartSixteen = self.qEndSixteen;
        end





        function RinsingRotatedUp(self, plateIndex)
            % Move to the first waypoint above the plate pickup point.
            disp("STATUS: MOVING TO HOME POSITION")
            fprintf('\n');

            % Define the desired transformation matrix (pose) above the pickup point

            t1 = transl(-1.065, 0.7, 1.5)* rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0,0,0,0,0,0];
            self.qEndSixteen = self.robot1.model.ikine(t1);


            % Generate a smooth trajectory between the current and desired joint configurations
            s = lspb(0, 1, self.stepsSixteen)  % Generate a time vector using linear segment with parabolic blends (LSPB)
            path1 = nan(self.stepsSixteen, 6)  % Preallocate the path array for the trajectory

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

                    tr = self.robot1.model.fkine(currentstep);

                    rotation = trotx(-pi/2);

                    offset = transl(0,-0.1,0);

                    trEdgeGrasp = tr.T * rotation * offset;

                    self.transformedVertices = [self.plates.plateVertices{plateIndex}, ones(size(self.plates.plateVertices{plateIndex},1),1)] * trEdgeGrasp';

                    set(self.plates.plates_h{plateIndex}, 'Vertices', self.transformedVertices(:,1:3));

                else
                    % Animate the robot movement for the j-th step
                    self.robot1.model.animate(path1(j, :));

                    tr = self.robot1.model.fkine(path1(j,:));

                    rotation = trotx(-pi/2);

                    offset = transl(0,-0.1,0);

                    trEdgeGrasp = tr.T * rotation * offset;

                    self.transformedVertices = [self.plates.plateVertices{plateIndex}, ones(size(self.plates.plateVertices{plateIndex},1),1)] * trEdgeGrasp';

                    set(self.plates.plates_h{plateIndex}, 'Vertices', self.transformedVertices(:,1:3));

                end

                axis equal  % Keep equal axis proportions for proper visualization
                drawnow();  % Update the figure window in real-time
                pause(0.01);  % Add a small pause to control the animation speed
            end

            % Update the starting joint configuration to the final configuration for the next motion
            self.qStartSixteen = self.qEndSixteen;
        end






        function RinsingRotatedDown(self, plateIndex)
            % Move to the first waypoint above the plate pickup point.
            disp("STATUS: MOVING TO HOME POSITION")
            fprintf('\n');

            % Define the desired transformation matrix (pose) above the pickup point

            % t1 = transl(-1.065, 0.7800, 1.4)*rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0,0,0,0,0,0];
            % self.qEndSixteen = self.robot1.model.ikine(t1);

            self.qEndSixteen = self.qStartSixteen + deg2rad([0,0,0,0,0,180]);

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


                drawnow();  % Update the figure window to display the text
            end

            % Update the starting joint configuration to the final configuration for the next motion
            % self.qStartSixteen = self.qEndSixteen + deg2rad([0,0,0,0,0,-360]);
            self.qStartSixteen = self.qEndSixteen + deg2rad([0,0,0,0,0,0]);
        end





        function UR3eRotatedDown(self, plateIndex)
            % Move to the first waypoint above the plate pickup point.
            disp("STATUS: MOVING TO HOME POSITION")
            fprintf('\n');

            % Define the desired transformation matrix (pose) above the pickup point

            t1 = transl(-0.8, 0.7800, 1.4)*rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0,0,0,0,0,0];
            self.qEndSixteen = self.robot1.model.ikine(t1) + deg2rad([0,0,0,0,0,180]);

            % Generate a smooth trajectory between the current and desired joint configurations
            s = lspb(0, 1, self.stepsSixteen);  % Generate a time vector using linear segment with parabolic blends (LSPB)
            path1 = nan(self.stepsSixteen, 6); % Preallocate the path array for the trajectory

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
                    self.robot1.model.plot(currentstep); % Animate and hold the robot at the current position

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
            self.qStartSixteen = self.qEndSixteen + deg2rad([0,0,0,0,0,0]);
        end






        function UR3eRotatedUp(self, plateIndex)
            % Move to the first waypoint above the plate pickup point.
            disp("STATUS: MOVING TO HOME POSITION")
            fprintf('\n');

            % Define the desired transformation matrix (pose) above the pickup point

            % t1 = transl(-0.8, 0.7800, 1.4)*rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0,0,0,0,0,0];
            % self.qEndSixteen = self.robot1.model.ikine(t1);

            self.qEndSixteen = self.qStartSixteen + deg2rad([0,0,0,0,0,-180]);


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

                % Animate the robot movement for the j-th step

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
            self.qStartSixteen = self.qEndSixteen + deg2rad([0,0,0,0,0,0]);
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
            % Move to the first waypoint above the plate pickup point.
            disp("STATUS: MOVING TO HOME POSITION")
            fprintf('\n');

            % Define the desired transformation matrix (pose) above the pickup point

            t1 = transl(-0.683, 0.7, 1.668)*rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0,0,0,0,0,0];
            self.qEndSixteen = self.robot1.model.ikine(t1);

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

                axis equal  % Keep equal axis proportions for proper visualization
                drawnow();  % Update the figure window in real-time
                pause(0.01);  % Add a small pause to control the animation speed

            end

            % Update the starting joint configuration to the final configuration for the next motion
            self.qStartSixteen = self.qEndSixteen;
        end





        function HeadingToDropPlate(self, plateIndex)
            % Move to the first waypoint above the plate pickup point.
            disp("STATUS: MOVING TO HOME POSITION")
            fprintf('\n');

            % Define the desired transformation matrix (pose) above the pickup point

            t1 = transl(0, 0.5, 1.6)*rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0,0,0,0,0,0];
            self.qEndSixteen = self.robot1.model.ikine(t1);

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



                % Get the current transformation of the end-effector and adjust for gripper orientation
                Tr = self.robot1.model.fkine(path1(j, :)).T * troty(-pi/2);

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
            self.qStartSixteen = self.qEndSixteen;
        end






        function HeadingToDropPlateless(self)
            % Move to the first waypoint above the plate pickup point.
            disp("STATUS: MOVING TO HOME POSITION")
            fprintf('\n');

            % Define the desired transformation matrix (pose) above the pickup point

            t1 = transl(-0.3, 0.35, 1.6)*rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0,0,0,0,0,0];
            self.qEndSixteen = self.robot1.model.ikine(t1);

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

                axis equal  % Keep equal axis proportions for proper visualization
                drawnow();  % Update the figure window in real-time
                pause(0.01);  % Add a small pause to control the animation speed

            end

            % Update the starting joint configuration to the final configuration for the next motion
            self.qStartSixteen = self.qEndSixteen;
        end


        function PickRotatedUp(self, plateIndex)
            % Move to the plate's position and pick it up using platesStart from Plates class
            disp("STATUS: MOVING TO PLATE");  % Display the current action
            fprintf('\n');  % New line for cleaner output

            % Generate the transformation to move above the plate's position

            t2 = transl([self.plates.platesGrab(plateIndex,:)]) * rpy2tr(90, 0, 180, 'deg');

            self.qEndSixteen = self.robot1.model.ikcon(t2, self.qStartSixteen);  % Initial guess added here

            % Generate the trajectory using LSPB (Linear Segment with Parabolic Blend) for smooth motion
            s = lspb(0, 1, self.stepsSixteen);  % Time scaling for the trajectory
            path2 = nan(self.stepsSixteen, 6);  % Preallocate the path array

            % Interpolate joint angles between the starting and ending configurations
            % for j = 1:self.stepsSixteen
            %     path2(j,:) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;  % Interpolate joint configurations
            % end

            % Animate the robot and gripper along the trajectory
            for j = 1:self.stepsSixteen

                path2(j,:) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;  % Interpolate joint configurations

                data = readline(self.arduinoObj);  % Read line from the Arduino serial port
                Estop = int32(str2double(data));  % Convert to integer if valid

                if self.app.eStopFlag == 1 || Estop == 1

                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position

                else
                    % Animate the robot movement for the j-th step
                    self.robot1.model.animate(path2(j, :));
                end


                axis equal;  % Keep axis scaling equal for proper visualization
                drawnow();  % Update the figure window for real-time animation
                pause(0.01);  % Pause for a short duration to control animation speed
            end

            % Update the starting configuration for the next move
            self.qStartSixteen = self.qEndSixteen;

        end




        function AboveDropPointPlate(self, plateIndex)
            % Move to the dropoff zone above the target
            disp("STATUS: MOVING TO DROP OFF ZONE");  % Display the current action
            fprintf('\n');  % New line for cleaner output

            % Generate the transformation to move above the dropoff zone

            t3 = transl(0.05, 0.35, 1.4)*rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0,0,0,0,0,0];
            self.qEndSixteen = self.robot1.model.ikine(t3);

            % 0.05, 0.35, 1.35


            % Generate the trajectory using LSPB for smooth motion
            s = lspb(0, 1, self.stepsSixteen);  % Time scaling for the trajectory
            path3 = nan(self.stepsSixteen, 6);  % Preallocate the path array

            % Interpolate joint angles between the starting and ending configurations
            % for j = 1:self.stepsSixteen
            %     path3(j,:) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;  % Interpolate joint configurations
            % end

            % Animate the robot and gripper along the trajectory
            for j = 1:self.stepsSixteen

                path3(j,:) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;  % Interpolate joint configurations

                data = readline(self.arduinoObj);  % Read line from the Arduino serial port
                Estop = int32(str2double(data));  % Convert to integer if valid

                if self.app.eStopFlag == 1 || Estop == 1

                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position

                else
                    % Animate the robot movement for the j-th step
                    self.robot1.model.animate(path3(j, :));
                end

                tr = self.robot1.model.fkine(path3(j,:));

                rotation = trotx(-pi/2);

                offset = transl(0,-0.1,0);

                trEdgeGrasp = tr.T * rotation * offset;

                self.transformedVertices = [self.plates.plateVertices{plateIndex}, ones(size(self.plates.plateVertices{plateIndex},1),1)] * trEdgeGrasp';

                set(self.plates.plates_h{plateIndex}, 'Vertices', self.transformedVertices(:,1:3));

                drawnow();  % Update the figure window to show the text
            end

            % Update the starting configuration for the next move
            self.qStartSixteen = self.qEndSixteen;

        end



        function AboveDropPointPlateless(self)
            % Move to the dropoff zone above the target
            disp("STATUS: MOVING TO DROP OFF ZONE");  % Display the current action
            fprintf('\n');  % New line for cleaner output

            % Generate the transformation to move above the dropoff zone

            t3 = transl(0.05, 0.35, 1.4)*rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0,0,0,0,0,0];
            self.qEndSixteen = self.robot1.model.ikine(t3);

            % 0.05, 0.35, 1.35


            % Generate the trajectory using LSPB for smooth motion
            s = lspb(0, 1, self.stepsSixteen);  % Time scaling for the trajectory
            path3 = nan(self.stepsSixteen, 6);  % Preallocate the path array

            % Interpolate joint angles between the starting and ending configurations
            % for j = 1:self.stepsSixteen
            %     path3(j,:) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;  % Interpolate joint configurations
            % end

            % Animate the robot and gripper along the trajectory
            for j = 1:self.stepsSixteen

                path3(j,:) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;  % Interpolate joint configurations

                data = readline(self.arduinoObj);  % Read line from the Arduino serial port
                Estop = int32(str2double(data));  % Convert to integer if valid

                if self.app.eStopFlag == 1 || Estop == 1

                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position

                else
                    % Animate the robot movement for the j-th step
                    self.robot1.model.animate(path3(j, :));
                end

                drawnow();  % Update the figure window to show the text
            end

            % Update the starting configuration for the next move
            self.qStartSixteen = self.qEndSixteen;

        end




        function DropRotatedUp(self, plateIndex)
            % Release the plate at the dropoff zone using platesEnd from the Plates class
            disp("STATUS: PLACING PLATE AT DROP OFF ZONE");  % Display the status message
            fprintf('\n');  % New line for clean output formatting

            % Calculate the transformation matrix for the dropoff position
            % of the plate

            t4 = transl([self.plates.platesEnd(plateIndex,:)]) * rpy2tr(90, 0, 90, 'deg'), 'q0', self.qStartSixteen, 'mask', [0,0,0,0,0,0];
            self.qEndSixteen = self.robot1.model.ikine(t4);

            % Generate a smooth trajectory using LSPB (Linear Segment with Parabolic Blend)
            s = lspb(0, 1, self.stepsSixteen);  % Time scaling for smooth motion
            path4 = nan(self.stepsSixteen, 6);  % Preallocate memory for joint trajectory

            % Interpolate between the start and end joint configurations
            % for j = 1:self.stepsSixteen
            %     path4(j,:) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;  % Linear interpolation
            % end

            % Animate the robot and the gripper as it follows the trajectory

            for j = 1:self.stepsSixteen

                path4(j,:) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;  % Linear interpolation

                data = readline(self.arduinoObj);  % Read line from the Arduino serial port
                Estop = int32(str2double(data));  % Convert to integer if valid

                if self.app.eStopFlag == 1 || Estop == 1

                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position

                    tr = self.robot1.model.fkine(path4(j,:));

                    rotation = trotx(-pi/2);

                    offset = transl(0,-0.1,0);

                    trEdgeGrasp = tr.T * rotation * offset;

                    self.transformedVertices = [self.plates.plateVertices{plateIndex}, ones(size(self.plates.plateVertices{plateIndex},1),1)] * trEdgeGrasp';

                    set(self.plates.plates_h{plateIndex}, 'Vertices', self.transformedVertices(:,1:3));

                else
                    % Animate the robot movement for the j-th step
                    self.robot1.model.animate(path4(j, :));

                    tr = self.robot1.model.fkine(path4(j,:));

                    rotation = trotx(-pi/2);

                    offset = transl(0,-0.1,0);

                    trEdgeGrasp = tr.T * rotation * offset;

                    self.transformedVertices = [self.plates.plateVertices{plateIndex}, ones(size(self.plates.plateVertices{plateIndex},1),1)] * trEdgeGrasp';

                    set(self.plates.plates_h{plateIndex}, 'Vertices', self.transformedVertices(:,1:3));
                end

                axis equal;  % Keep axis scaling uniform
                drawnow();  % Update the plot

                % Add a small pause to control the animation speed
                pause(0.01);
            end

            % Update the starting joint configuration for the next move
            self.qStartSixteen = self.qEndSixteen;
        end



        function Home(self)
            % Return the robot to the home position after finishing the task
            disp("STATUS: RETURNING HOME");  % Display the status message
            fprintf('\n');  % New line for clean output formatting

            % Calculate the transformation matrix for the home position

            t5 = transl(0.05, 0.603, 1.8)*rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0,0,0,0,0,0];
            self.qEndSixteen = self.robot1.model.ikine(t5);

            % Generate a smooth trajectory using LSPB for smooth motion
            s = lspb(0, 1, self.stepsSixteen);  % Time scaling for smooth motion
            path5 = nan(self.stepsSixteen, 6);  % Preallocate memory for the joint trajectory

            % Interpolate between the start and end joint configurations
            % for j = 1:self.stepsSixteen
            %     path5(j,:) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;  % Linear interpolation
            % end

            % Animate the robot and gripper along the return home trajectory
            for j = 1:self.stepsSixteen

                path5(j,:) = (1-s(j)) * self.qStartSixteen + s(j) * self.qEndSixteen;  % Interpolate joint configurations

                data = readline(self.arduinoObj);  % Read line from the Arduino serial port
                Estop = int32(str2double(data));  % Convert to integer if valid

                if self.app.eStopFlag == 1 || Estop == 1

                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position

                else
                    % Animate the robot movement for the j-th step
                    self.robot1.model.animate(path5(j, :));
                end


                axis equal;  % Keep axis scaling uniform
                drawnow();  % Update the plot

                % Add a small pause to control the animation speed
                pause(0.01);
            end

            % Update the starting joint configuration for the next move
            self.qStartSixteen = self.qEndSixteen;
        end





        function StandbyMode(self)
            % Move to the first waypoint above the plate pickup point.
            disp("STATUS: MOVING TO HOME POSITION")
            fprintf('\n');

            % Define the desired transformation matrix (pose) above the pickup point
            t1 = transl([-0.8 0.7800 1.8]) * rpy2tr(180, 0, 0, 'deg');
            self.qEndThree = self.robot2.model.ikcon(t1);  % Compute joint configuration using inverse kinematics

            % Generate a smooth trajectory between the current and desired joint configurations
            s = lspb(0, 1, self.stepsThree);  % Generate a time vector using linear segment with parabolic blends (LSPB)
            path1 = nan(self.stepsThree, 6);  % Preallocate the path array for the trajectory

            % Interpolate joint angles between qStart and qEnd for each step
            % for j = 1:self.stepsThree
            %     path1(j, :) = (1-s(j)) * self.qStartThree + s(j) * self.qEndThree;  % Linear interpolation for each joint angle
            % end

            % Animate the robot and gripper along the interpolated trajectory
            for j = 1:self.stepsThree

                path1(j, :) = (1-s(j)) * self.qStartThree + s(j) * self.qEndThree;  % Linear interpolation for each joint angle

                data = readline(self.arduinoObj);  % Read line from the Arduino serial port
                Estop = int32(str2double(data));  % Convert to integer if valid

                if self.app.eStopFlag == 1 || Estop == 1

                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position

                else
                    % Animate the robot movement for the j-th step
                    self.robot2.model.animate(path1(j, :));
                end

                % Animate the robot movement for the j-th step
                self.robot2.model.animate(path1(j, :));

                axis equal  % Keep equal axis proportions for proper visualization
                drawnow();  % Update the figure window in real-time
                pause(0.01);  % Add a small pause to control the animation speed

                drawnow();  % Update the figure window to display the text
            end

            % Update the starting joint configuration to the final configuration for the next motion
            self.qStartThree = self.qEndThree;
        end





        function POC1(self)
            % Move to the dropoff zone above the target
            disp("STATUS: MOVING TO DROP OFF ZONE");  % Display the current action
            fprintf('\n');  % New line for cleaner output

            % Generate the transformation to move above the dropoff zone
            t3 = transl([-0.8 0.7800 1.45]) * rpy2tr(180,0,0,'deg');
            self.qEndThree = self.robot2.model.ikcon(t3);  % Calculate inverse kinematics to find joint configuration

            % Generate the trajectory using LSPB for smooth motion
            s = lspb(0, 1, self.stepsThree);  % Time scaling for the trajectory
            path3 = nan(self.stepsThree, 6);  % Preallocate the path array

            % Interpolate joint angles between the starting and ending configurations
            % for j = 1:self.stepsThree
            %     path3(j,:) = (1-s(j)) * self.qStartThree + s(j) * self.qEndThree;  % Interpolate joint configurations
            % end

            % Animate the robot and gripper along the trajectory
            for j = 1:self.stepsThree

                path3(j,:) = (1-s(j)) * self.qStartThree + s(j) * self.qEndThree;  % Interpolate joint configurations

                data = readline(self.arduinoObj);  % Read line from the Arduino serial port
                Estop = int32(str2double(data));  % Convert to integer if valid

                if self.app.eStopFlag == 1 || Estop == 1

                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position

                else
                    % Animate the robot movement for the j-th step
                    self.robot2.model.animate(path3(j, :));
                end

                axis equal;  % Keep axis scaling equal for proper visualization
                drawnow();  % Update the figure window for real-time animation
                pause(0.01);  % Pause for a short duration to control animation speed
            end

            % Update the starting configuration for the next move
            self.qStartThree = self.qEndThree;

        end


        function POC2(self)
            % Move to the dropoff zone above the target
            disp("STATUS: MOVING TO DROP OFF ZONE");  % Display the current action
            fprintf('\n');  % New line for cleaner output

            % Generate the transformation to move above the dropoff zone
            t3 = transl([-0.9 0.7800 1.45]) * rpy2tr(180,0,0,'deg');
            self.qEndThree = self.robot2.model.ikcon(t3);  % Calculate inverse kinematics to find joint configuration

            % Generate the trajectory using LSPB for smooth motion
            s = lspb(0, 1, self.stepsThree);  % Time scaling for the trajectory
            path3 = nan(self.stepsThree, 6);  % Preallocate the path array

            % Interpolate joint angles between the starting and ending configurations
            % for j = 1:self.stepsThree
            %     path3(j,:) = (1-s(j)) * self.qStartThree + s(j) * self.qEndThree;  % Interpolate joint configurations
            % end

            % Animate the robot and gripper along the trajectory
            for j = 1:self.stepsThree

                path3(j,:) = (1-s(j)) * self.qStartThree + s(j) * self.qEndThree;  % Interpolate joint configurations

                data = readline(self.arduinoObj);  % Read line from the Arduino serial port
                Estop = int32(str2double(data));  % Convert to integer if valid

                if self.app.eStopFlag == 1 || Estop == 1

                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position

                else
                    % Animate the robot movement for the j-th step
                    self.robot2.model.animate(path3(j, :));
                end

                self.robot2.model.animate(path3(j, :));  % Animate the robot at each step

                axis equal;  % Keep axis scaling equal for proper visualization
                drawnow();  % Update the figure window for real-time animation
                pause(0.01);  % Pause for a short duration to control animation speed
            end

            % Update the starting configuration for the next move
            self.qStartThree = self.qEndThree;

        end



        function POC3(self)
            % Move to the dropoff zone above the target
            disp("STATUS: MOVING TO DROP OFF ZONE");  % Display the current action
            fprintf('\n');  % New line for cleaner output

            % Generate the transformation to move above the dropoff zone
            t3 = transl([-0.7 0.900 1.45]) * rpy2tr(180,0,0,'deg');
            self.qEndThree = self.robot2.model.ikcon(t3);  % Calculate inverse kinematics to find joint configuration

            % Generate the trajectory using LSPB for smooth motion
            s = lspb(0, 1, self.stepsThree);  % Time scaling for the trajectory
            path3 = nan(self.stepsThree, 6);  % Preallocate the path array

            % Interpolate joint angles between the starting and ending configurations
            % for j = 1:self.stepsThree
            %     path3(j,:) = (1-s(j)) * self.qStartThree + s(j) * self.qEndThree;  % Interpolate joint configurations
            % end

            % Animate the robot and gripper along the trajectory
            for j = 1:self.stepsThree

                path3(j,:) = (1-s(j)) * self.qStartThree + s(j) * self.qEndThree;  % Interpolate joint configurations

                data = readline(self.arduinoObj);  % Read line from the Arduino serial port
                Estop = int32(str2double(data));  % Convert to integer if valid

                if self.app.eStopFlag == 1 || Estop == 1

                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position

                else
                    % Animate the robot movement for the j-th step
                    self.robot2.model.animate(path3(j, :));
                end


                axis equal;  % Keep axis scaling equal for proper visualization
                drawnow();  % Update the figure window for real-time animation
                pause(0.01);  % Pause for a short duration to control animation speed
            end

            % Update the starting configuration for the next move
            self.qStartThree = self.qEndThree;

        end




        function ReturnHome(self)
            % Return the robot to the home position after finishing the task
            disp("STATUS: RETURNING HOME");  % Display the status message
            fprintf('\n');  % New line for clean output formatting

            % % Calculate the transformation matrix for the home position
            % t5 = transl([-0.75, 0.5, 0.5]) * rpy2tr(180,0,0,'deg');
            % self.qEndThree = self.robot2.model.ikcon(t5);  % Compute the joint angles using inverse kinematics

            self.qEndThree = self.qStartThree + deg2rad([0,-30,0,0,0,0]);

            % Generate a smooth trajectory using LSPB for smooth motion
            s = lspb(0, 1, self.stepsThree);  % Time scaling for smooth motion
            path5 = nan(self.stepsThree, 6);  % Preallocate memory for the joint trajectory

            % Interpolate between the start and end joint configurations
            % for j = 1:self.stepsThree
            %     path5(j,:) = (1-s(j)) * self.qStartThree + s(j) * self.qEndThree;  % Linear interpolation
            % end

            % Animate the robot and gripper along the return home trajectory
            for j = 1:self.stepsThree

                path5(j,:) = (1-s(j)) * self.qStartThree + s(j) * self.qEndThree;  % Linear interpolation

                data = readline(self.arduinoObj);  % Read line from the Arduino serial port
                Estop = int32(str2double(data));  % Convert to integer if valid

                if self.app.eStopFlag == 1 || Estop == 1

                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position

                else
                    % Animate the robot movement for the j-th step
                    self.robot2.model.animate(path5(j, :));
                end

                axis equal;  % Keep axis scaling uniform
                drawnow();  % Update the plot

                % Add a small pause to control the animation speed
                pause(0.01);
            end

            % Update the starting joint configuration for the next move
            self.qStartThree = self.qEndThree;
        end


    end
end


