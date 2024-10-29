classdef SpotlessTrajectoryrevestopREVA < handle
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
        text_hSixteen              % Text handle for displaying pose info
        text_hThree
        gripper            % Instance of the Gripper class
        desiredPoseSixteen         % Desired pose based on transformation matrix
        desiredPoseThree
        actualPoseSixteen          % Actual pose after inverse kinematics
        actualPoseThree
        positionErrorSixteen    % Difference between desired and actual pose
        positionErrorThree
        toleranceSixteen = 0.005; % Tolerance for position error (5mm)
        toleranceThree = 0.005
        qEndSixteen               % Final joint configuration from inverse kinematics
        qEndThree
        tActualSixteen            % Transformation matrix of the actual end-effector pose
        tActualThree
        transformedVertices
        plateVertices
        plates_h
    
        app;
        eStopFlag;

    end

    methods
        function self = SpotlessTrajectoryrevestopREVA(robot2, robot1, plates, gripper, app)
            % % Constructor to initialize the robot, number of steps, starting configuration, plates, and gripper
            % self.robot1 = robot1;  % Assign the robot instance
            % self.stepsSixteen = 10;  % Set the number of steps for trajectory interpolation
            % self.qStartSixteen = [0 -135*pi/180 pi/4 pi/2 0 0];  % Initial joint configuration
            % self.plates = plates;  % Store the Plates instance
            % self.text_hSixteen = [];  % Initialize the text handle
            % self.gripper = gripper;  % Store the Gripper instance

            % Constructor to initialize the robot, number of steps, starting configuration, plates, and gripper
            self.robot1 = robot1;  % Assign the robot instance
            self.robot2 = robot2;  % Assign the robot instance
            self.stepsSixteen = 60;  % Set the number of steps for trajectory interpolation
            self.stepsThree = 60;
            self.qStartSixteen = [0 -135*pi/180 pi/4 pi/2 0 0];  % Initial joint configuration
            self.qStartThree = [-pi -pi/2 0 pi 0 0];
            self.plates = plates;  % Store the Plates instance
            self.text_hSixteen = [];  % Initialize the text handle
            self.text_hThree = [];
            self.gripper = gripper;  % Store the Gripper instance

            self.app = app;
            self.eStopFlag = 0;   % zero = E-stop in inactive


            %UR3
            %
            % self.robot2 = robot2;  % Assign the robot instance
            % self.stepsThree = 100;  % Set the number of steps for trajectory interpolation
            % self.qStartThree = [-pi -pi/2 0 pi 0 0];  % Initial joint configuration
            % self.text_hThree = [];  % Initialize the text handle
        end

        function Run(self)
            % Main function to perform the full trajectory of picking up and dropping off plates.
            disp("STATUS: INITIALIZING ACTION")
            fprintf('\n');

            % Loop over all plates to pick and place them
            for plateIndex = 1:3
                self.StandbyMode(); % 3
                % self.Home();

                self.InfrontOfTap();
                self.AbovePickPointPlateless();  % Move to the point above the plate's pickup position
                self.PickRotatedUp(plateIndex);  % Move to and pick up the plate
                self.AbovePickPointPlate(plateIndex);  % Return to above the pickup point
                self.RinsingRotatedUp(plateIndex);
                self.RinsingRotatedDown(plateIndex);
                self.UR3eRotatedDown(plateIndex);

                self.POC1();% 3
                self.POC2();% 3
                self.POC3(); % 3
                % self.ReturnHome(); %3
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
                self.AboveSensorPoint(plateIndex);
                self.HeadingToDrop(plateIndex);
                self.AboveDropPoint(plateIndex);  % Move to the point above the drop-off zone
                self.DropRotatedUp(plateIndex);  % Drop the plate in the designated location
                self.AboveDropPoint();
                self.HeadingToDrop();




            end
            self.Home();  % Return the robot to the home position

            disp("STATUS: ACTION FINISHED")
            fprintf('\n');
        end

        % Order

        % InfrontOfTap d
        % AbovePickPoint d
        % PickRotatedUp * d
        % AbovePickPoint d
        % RinsingRotatedUp
        % RinsingRotatedDown
        % ScrubbingRotatedDown
        % ScrubbingRotatedUp
        % RinsingRotatedUp
        % RinsingRotatedDown
        % SensingRotatedDown
        % SensingRotatedUp
        % AboveSensorPoint
        % HeadingToDrop
        % AboveDropPoint * d
        % DropRotatedUp * d

        % Home * d

        % Waypoints

        % InfrontOfTap *  -1.01 0.394 1.668 d
        % AbovePickPoint * -1.55 0.6  1.6 d
        % RinsingRotatedUp -1.065 0.7800 1.4 d
        % RinsingRotatedDown* -1.065 0.7800 1.4 d
        % ScrubbingRotatedDown* -0.8 0.7800 1.4 d
        % ScrubbingRotatedUp* -0.8 0.7800 1.4 d
        % SensingRotatedDown* -1.065 0.7800 1.4  d
        % SensingRotatedUp* -1.065 0.7800 1.4 d
        % AboveSensorPoint* -0.683 0.7 1.668 d
        % HeadingToDrop* -0.3 0.7 1.668 d
        % AboveDropPoint *  0.392 0.603 1.6 d
        % Home * -0.683 0.358 1.668 d

        % 2 pick/drop points + 12 waypoints = 14 points total

        function InfrontOfTap(self)
            % Move to the first waypoint above the plate pickup point.
            disp("STATUS: MOVING TO HOME POSITION")
            fprintf('\n');

            % Define the desired transformation matrix (pose) above the pickup point
            % t1 = transl([-1.01 0.394 1.668]) * rpy2tr(180, 0, 180, 'deg');
            % self.qEnd = self.robot.model.ikcon(t1);  % Compute joint configuration using inverse kinematics

            t1 = transl([-1.01 0.394 1.668]) * rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [1,1,0,0,0,0];
            self.qEndSixteen = self.robot1.model.ikine(t1);

            % t1 = transl([-1.01 0.394 1.668]) * rpy2tr(-180, 0, 0, 'deg'), 'q0', self.qStart, 'mask', [0 0 0 0 0 0];
            % self.qEnd = self.robot.model.ikine(t1);

            % nextQ = currentQ + deg2rad([180,0,0,0,0,0])
            % qMatrix = jtrak(curentQ, nextQ)

            % Compute actual joint state after inverse kinematics (ikcon)
            self.tActualSixteen = self.robot1.model.fkine(self.qEndSixteen).T;  % Get the transformation matrix of the current joint configuration
            self.desiredPoseSixteen = t1(1:3, 4);  % Extract the desired end-effector position (x, y, z)
            self.actualPoseSixteen = self.tActualSixteen(1:3, 4);  % Extract the actual end-effector position (x, y, z) from forward kinematics

            % Compute the position error between the desired and actual end-effector positions
            self.positionErrorSixteen = norm(self.desiredPoseSixteen - self.actualPoseSixteen);  % Euclidean distance between desired and actual positions

            % Check if the error is within the acceptable tolerance of 5mm
            if self.positionErrorSixteen <= self.toleranceSixteen
                disp('Pose satisfied within +- 5mm.');  % Display success message if within tolerance
            else
                disp('Pose does not meet the +-5mm tolerance.');  % Display warning if error exceeds tolerance
            end

            % Display the desired and actual poses for debugging/verification
            disp('Desired pose:');
            disp(self.desiredPoseSixteen);

            disp('Actual pose:');
            disp(self.actualPoseSixteen);

            % Display the computed position error in meters
            disp('Position error:');
            disp(self.positionErrorSixteen);

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


                if self.app.eStopFlag == 1
                    
                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position
                    
                else
                    % Animate the robot movement for the j-th step
                    self.robot1.model.animate(path1(j, :));
                end
                  

                % Get the current transformation of the end-effector and adjust for gripper orientation
                % Tr = self.robot1.model.fkine(path1(j, :)).T * troty(-pi/2);

                % Animate the gripper with no delay (gripper movement in sync with the robot)
                % self.gripper.robot.delay = 0;
                % self.gripper.animateGripper(Tr);  % Move the gripper along with the robot's end-effector
                axis equal  % Keep equal axis proportions for proper visualization
                drawnow();  % Update the figure window in real-time
                pause(0.01);  % Add a small pause to control the animation speed

                % % Delete the previously displayed text handle if it exists
                % try delete(self.text_hSixteen); end  %#ok<TRYNC>
                % 
                % % Compute and display the final transformation matrix at the last step
                % Tr1 = self.robot1.model.fkine(path1(size(self.stepsSixteen, 1), :)).T;  % Get the transformation matrix at the final step
                % try delete(Tr1); end  %#ok<TRYNC>  % Try to delete Tr1 if needed
                % 
                % % Create a message displaying the final pose information
                % message = sprintf([num2str(round(Tr1(1, :), 2, 'significant')), '\n' ...
                %     num2str(round(Tr1(2, :), 2, 'significant')), '\n' ...
                %     num2str(round(Tr1(3, :), 2, 'significant'))]);
                % 
                % % Display the final pose as text on the plot
                % self.text_hSixteen = text(0, 0, 3, message, 'FontSize', 10, 'Color', [.6 .2 .6]);  % Text display in purple color

                drawnow();  % Update the figure window to display the text
            end

            % Update the starting joint configuration to the final configuration for the next motion
            self.qStartSixteen = self.qEndSixteen;
        end




        function AbovePickPointPlateless(self)
            % Move to the first waypoint above the plate pickup point.
            disp("STATUS: MOVING TO HOME POSITION");
            fprintf('\n');

            % % Define the desired transformation matrix above the pickup point
            % t1 = transl([-1.2, 0.4, 1.668]) * rpy2tr(90, 0, 180, 'deg');
            % 
            % % Use ikcon with an initial guess for smoother results
            % self.qEndSixteen = self.robot1.model.ikcon(t1, self.qStartSixteen);  % Initial guess added here

            t1 = transl([-1.2, 0.4, 1.668]) * rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0 0 0 0 0 0];
            self.qEndSixteen = self.robot1.model.ikine(t1);

            % Display final joint configuration for debugging
            disp("Computed final joint configuration:");
            disp(self.qEndSixteen);


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

                if self.app.eStopFlag == 1
                    
                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position
                    
                else
                    % Animate the robot movement for the j-th step
                    self.robot1.model.animate(path1(j, :));
                end
                % % Calculate the end-effector transformation for the gripper
                % Tr = self.robot1.model.fkine(path1(j, :)).T;


                % Animate the gripper with no delay (gripper movement in sync with the robot)
                % self.gripper.robot.delay = 0;
                % self.gripper.animateGripper(Tr);
                axis equal;
                drawnow();
                pause(0.01);

                % % Delete the previously displayed text handle if it exists
                % try delete(self.text_hSixteen); end %#ok<TRYNC>
                % 
                % % Display final pose at the end for debugging
                % if j == self.stepsSixteen
                %     disp("Final pose of the end-effector:");
                %     disp(Tr);
                %     message = sprintf([num2str(round(Tr(1, :), 2, 'significant')), '\n' ...
                %         num2str(round(Tr(2, :), 2, 'significant')), '\n' ...
                %         num2str(round(Tr(3, :), 2, 'significant'))]);
                %     self.text_hSixteen = text(0, 0, 3, message, 'FontSize', 10, 'Color', [.6 .2 .6]);
                %     drawnow();
                % end
            end

            % Update the starting joint configuration to the final configuration for the next motion
            self.qStartSixteen = self.qEndSixteen;
        end



        function AbovePickPointPlate(self, plateIndex)
            % Move to the first waypoint above the plate pickup point.
            disp("STATUS: MOVING TO HOME POSITION");
            fprintf('\n');

            % % Define the desired transformation matrix above the pickup point
            % t1 = transl([-1.2, 0.4, 1.668]) * rpy2tr(90, 0, 180, 'deg');
            % 
            % % Use ikcon with an initial guess for smoother results
            % self.qEndSixteen = self.robot1.model.ikcon(t1, self.qStartSixteen);  % Initial guess added here

            t1 = transl([-1.2, 0.4, 1.668]) * rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0 0 0 0 0 0];
            self.qEndSixteen = self.robot1.model.ikine(t1);

            % Display final joint configuration for debugging
            disp("Computed final joint configuration:");
            disp(self.qEndSixteen);



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

                if self.app.eStopFlag == 1

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

                if self.app.eStopFlag == 1
                    
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

            % % Compute actual joint state after inverse kinematics (ikcon)
            % self.tActualSixteen = self.robot1.model.fkine(self.qEndSixteen).T;  % Get the transformation matrix of the current joint configuration
            % self.desiredPoseSixteen = t1(1:3, 4);  % Extract the desired end-effector position (x, y, z)
            % self.actualPoseSixteen = self.tActualSixteen(1:3, 4);  % Extract the actual end-effector position (x, y, z) from forward kinematics
            % 
            % % Compute the position error between the desired and actual end-effector positions
            % self.positionErrorSixteen = norm(self.desiredPoseSixteen - self.actualPoseSixteen);  % Euclidean distance between desired and actual positions
            % 
            % % Check if the error is within the acceptable tolerance of 5mm
            % if self.positionErrorSixteen <= self.toleranceSixteen
            %     disp('Pose satisfied within +- 5mm.');  % Display success message if within tolerance
            % else
            %     disp('Pose does not meet the +-5mm tolerance.');  % Display warning if error exceeds tolerance
            % end
            % 
            % % Display the desired and actual poses for debugging/verification
            % disp('Desired pose:');
            % disp(self.desiredPoseSixteen);
            % 
            % disp('Actual pose:');
            % disp(self.actualPoseSixteen);
            % 
            % % Display the computed position error in meters
            % disp('Position error:');
            % disp(self.positionErrorSixteen);

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

                if self.app.eStopFlag == 1

                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position

                else
                    % Animate the robot movement for the j-th step
                    self.robot1.model.animate(path1(j, :));
                end
                % Get the current transformation of the end-effector and adjust for gripper orientation
                % Tr = self.robot1.model.fkine(path1(j, :)).T * troty(-pi/2);

                tr = self.robot1.model.fkine(path1(j,:));

                rotation = trotx(-pi/2);

                offset = transl(0,-0.1,0);

                trEdgeGrasp = tr.T * rotation * offset;

                self.transformedVertices = [self.plates.plateVertices{plateIndex}, ones(size(self.plates.plateVertices{plateIndex},1),1)] * trEdgeGrasp';

                set(self.plates.plates_h{plateIndex}, 'Vertices', self.transformedVertices(:,1:3));


                % Animate the gripper with no delay (gripper movement in sync with the robot)
                % self.gripper.robot.delay = 0;
                % self.gripper.animateGripper(Tr);  % Move the gripper along with the robot's end-effector
                axis equal  % Keep equal axis proportions for proper visualization
                drawnow();  % Update the figure window in real-time
                pause(0.01);  % Add a small pause to control the animation speed

                % % Delete the previously displayed text handle if it exists
                % try delete(self.text_hSixteen); end  %#ok<TRYNC>
                % 
                % % Compute and display the final transformation matrix at the last step
                % Tr1 = self.robot1.model.fkine(path1(size(self.stepsSixteen, 1), :)).T;  % Get the transformation matrix at the final step
                % try delete(Tr1); end  %#ok<TRYNC>  % Try to delete Tr1 if needed
                % 
                % % Create a message displaying the final pose information
                % message = sprintf([num2str(round(Tr1(1, :), 2, 'significant')), '\n' ...
                %     num2str(round(Tr1(2, :), 2, 'significant')), '\n' ...
                %     num2str(round(Tr1(3, :), 2, 'significant'))]);
                % 
                % % Display the final pose as text on the plot
                % self.text_hSixteen = text(0, 0, 3, message, 'FontSize', 10, 'Color', [.6 .2 .6]);  % Text display in purple color

                drawnow();  % Update the figure window to display the text
            end

            % Update the starting joint configuration to the final configuration for the next motion
            self.qStartSixteen = self.qEndSixteen + deg2rad([0,0,0,0,0,-180]);
        end





        function UR3eRotatedUp(self, plateIndex)
            % Move to the first waypoint above the plate pickup point.
            disp("STATUS: MOVING TO HOME POSITION")
            fprintf('\n');

            % Define the desired transformation matrix (pose) above the pickup point

            % t1 = transl(-0.8, 0.7800, 1.4)*rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0,0,0,0,0,0];
            % self.qEndSixteen = self.robot1.model.ikine(t1);

            self.qEndSixteen = self.qStartSixteen + deg2rad([0,0,0,0,0,-180]);

            % % Compute actual joint state after inverse kinematics (ikcon)
            % self.tActualSixteen = self.robot1.model.fkine(self.qEndSixteen).T;  % Get the transformation matrix of the current joint configuration
            % self.desiredPoseSixteen = t1(1:3, 4);  % Extract the desired end-effector position (x, y, z)
            % self.actualPoseSixteen = self.tActualSixteen(1:3, 4);  % Extract the actual end-effector position (x, y, z) from forward kinematics
            % 
            % % Compute the position error between the desired and actual end-effector positions
            % self.positionErrorSixteen = norm(self.desiredPoseSixteen - self.actualPoseSixteen);  % Euclidean distance between desired and actual positions
            % 
            % % Check if the error is within the acceptable tolerance of 5mm
            % if self.positionErrorSixteen <= self.toleranceSixteen
            %     disp('Pose satisfied within +- 5mm.');  % Display success message if within tolerance
            % else
            %     disp('Pose does not meet the +-5mm tolerance.');  % Display warning if error exceeds tolerance
            % end
            % 
            % % Display the desired and actual poses for debugging/verification
            % disp('Desired pose:');
            % disp(self.desiredPoseSixteen);
            % 
            % disp('Actual pose:');
            % disp(self.actualPoseSixteen);
            % 
            % % Display the computed position error in meters
            % disp('Position error:');
            % disp(self.positionErrorSixteen);

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

                if self.app.eStopFlag == 1
                    
                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position
                    
                else
                    % Animate the robot movement for the j-th step
                    self.robot1.model.animate(path1(j, :));
                end

                % Animate the robot movement for the j-th step
         

                % Get the current transformation of the end-effector and adjust for gripper orientation
                % Tr = self.robot1.model.fkine(path1(j, :)).T * troty(-pi/2);

                tr = self.robot1.model.fkine(path1(j,:));

                rotation = trotx(-pi/2);

                offset = transl(0,-0.1,0);

                trEdgeGrasp = tr.T * rotation * offset;

                self.transformedVertices = [self.plates.plateVertices{plateIndex}, ones(size(self.plates.plateVertices{plateIndex},1),1)] * trEdgeGrasp';

                set(self.plates.plates_h{plateIndex}, 'Vertices', self.transformedVertices(:,1:3));


                % Animate the gripper with no delay (gripper movement in sync with the robot)
                % self.gripper.robot.delay = 0;
                % self.gripper.animateGripper(Tr);  % Move the gripper along with the robot's end-effector
                axis equal  % Keep equal axis proportions for proper visualization
                drawnow();  % Update the figure window in real-time
                pause(0.01);  % Add a small pause to control the animation speed
                % 
                % % Delete the previously displayed text handle if it exists
                % try delete(self.text_hSixteen); end  %#ok<TRYNC>
                % 
                % % Compute and display the final transformation matrix at the last step
                % Tr1 = self.robot1.model.fkine(path1(size(self.stepsSixteen, 1), :)).T;  % Get the transformation matrix at the final step
                % try delete(Tr1); end  %#ok<TRYNC>  % Try to delete Tr1 if needed
                % 
                % % Create a message displaying the final pose information
                % message = sprintf([num2str(round(Tr1(1, :), 2, 'significant')), '\n' ...
                %     num2str(round(Tr1(2, :), 2, 'significant')), '\n' ...
                %     num2str(round(Tr1(3, :), 2, 'significant'))]);
                % 
                % % Display the final pose as text on the plot
                % self.text_hSixteen = text(0, 0, 3, message, 'FontSize', 10, 'Color', [.6 .2 .6]);  % Text display in purple color

                drawnow();  % Update the figure window to display the text
            end

            % Update the starting joint configuration to the final configuration for the next motion
            self.qStartSixteen = self.qEndSixteen + deg2rad([0,0,0,0,0,360]);
        end






        function UR3eRotatedDown(self, plateIndex)
            % Move to the first waypoint above the plate pickup point.
            disp("STATUS: MOVING TO HOME POSITION")
            fprintf('\n');

            % Define the desired transformation matrix (pose) above the pickup point

            t1 = transl(-0.8, 0.7800, 1.4)*rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0,0,0,0,0,0];
            self.qEndSixteen = self.robot1.model.ikine(t1) + deg2rad([0,0,0,0,0,0]);

            % self.qEndSixteen = self.qStartSixteen + deg2rad([-30,-15,0,0,0,0]);

            % % Compute actual joint state after inverse kinematics (ikcon)
            % self.tActualSixteen = self.robot1.model.fkine(self.qEndSixteen).T;  % Get the transformation matrix of the current joint configuration
            % self.desiredPoseSixteen = t1(1:3, 4);  % Extract the desired end-effector position (x, y, z)
            % self.actualPoseSixteen = self.tActualSixteen(1:3, 4);  % Extract the actual end-effector position (x, y, z) from forward kinematics
            % 
            % % Compute the position error between the desired and actual end-effector positions
            % self.positionErrorSixteen = norm(self.desiredPoseSixteen - self.actualPoseSixteen);  % Euclidean distance between desired and actual positions
            % 
            % % Check if the error is within the acceptable tolerance of 5mm
            % if self.positionErrorSixteen <= self.toleranceSixteen
            %     disp('Pose satisfied within +- 5mm.');  % Display success message if within tolerance
            % else
            %     disp('Pose does not meet the +-5mm tolerance.');  % Display warning if error exceeds tolerance
            % end
            % 
            % % Display the desired and actual poses for debugging/verification
            % disp('Desired pose:');
            % disp(self.desiredPoseSixteen);
            % 
            % disp('Actual pose:');
            % disp(self.actualPoseSixteen);
            % 
            % % Display the computed position error in meters
            % disp('Position error:');
            % disp(self.positionErrorSixteen);

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

                if self.app.eStopFlag == 1
                    
                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep); % Animate and hold the robot at the current position
                    
                else
                    % Animate the robot movement for the j-th step
                    self.robot1.model.animate(path1(j, :));
                end

                
                
                % Get the current transformation of the end-effector and adjust for gripper orientation
                % Tr = self.robot1.model.fkine(path1(j, :)).T * troty(-pi/2);

                tr = self.robot1.model.fkine(path1(j,:));

                rotation = trotx(pi/2);

                offset = transl(0,0.1,0);

                trEdgeGrasp = tr.T * rotation * offset;

                self.transformedVertices = [self.plates.plateVertices{plateIndex}, ones(size(self.plates.plateVertices{plateIndex},1),1)] * trEdgeGrasp';

                set(self.plates.plates_h{plateIndex}, 'Vertices', self.transformedVertices(:,1:3));


                % Animate the gripper with no delay (gripper movement in sync with the robot)
                % self.gripper.robot.delay = 0;
                % self.gripper.animateGripper(Tr);  % Move the gripper along with the robot's end-effector
                axis equal  % Keep equal axis proportions for proper visualization
                drawnow();  % Update the figure window in real-time
                pause(0.01);  % Add a small pause to control the animation speed

                % % Delete the previously displayed text handle if it exists
                % try delete(self.text_hSixteen); end  %#ok<TRYNC>
                % 
                % % Compute and display the final transformation matrix at the last step
                % Tr1 = self.robot1.model.fkine(path1(size(self.stepsSixteen, 1), :)).T;  % Get the transformation matrix at the final step
                % try delete(Tr1); end  %#ok<TRYNC>  % Try to delete Tr1 if needed
                % 
                % % Create a message displaying the final pose information
                % message = sprintf([num2str(round(Tr1(1, :), 2, 'significant')), '\n' ...
                %     num2str(round(Tr1(2, :), 2, 'significant')), '\n' ...
                %     num2str(round(Tr1(3, :), 2, 'significant'))]);
                % 
                % % Display the final pose as text on the plot
                % self.text_hSixteen = text(0, 0, 3, message, 'FontSize', 10, 'Color', [.6 .2 .6]);  % Text display in purple color

                drawnow();  % Update the figure window to display the text
            end

            % Update the starting joint configuration to the final configuration for the next motion
            self.qStartSixteen = self.qEndSixteen + deg2rad([0,0,0,0,0,-180]);
        end



        function AboveSensorPoint(self, plateIndex)
            % Move to the first waypoint above the plate pickup point.
            disp("STATUS: MOVING TO HOME POSITION")
            fprintf('\n');

            % Define the desired transformation matrix (pose) above the pickup point

            t1 = transl(-0.683, 0.7, 1.668)*rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0,0,0,0,0,0];
            self.qEndSixteen = self.robot1.model.ikine(t1);

            % Compute actual joint state after inverse kinematics (ikcon)
            self.tActualSixteen = self.robot1.model.fkine(self.qEndSixteen).T;  % Get the transformation matrix of the current joint configuration
            self.desiredPoseSixteen = t1(1:3, 4);  % Extract the desired end-effector position (x, y, z)
            self.actualPoseSixteen = self.tActualSixteen(1:3, 4);  % Extract the actual end-effector position (x, y, z) from forward kinematics

            % Compute the position error between the desired and actual end-effector positions
            self.positionErrorSixteen = norm(self.desiredPoseSixteen - self.actualPoseSixteen);  % Euclidean distance between desired and actual positions

            % Check if the error is within the acceptable tolerance of 5mm
            if self.positionErrorSixteen <= self.toleranceSixteen
                disp('Pose satisfied within +- 5mm.');  % Display success message if within tolerance
            else
                disp('Pose does not meet the +-5mm tolerance.');  % Display warning if error exceeds tolerance
            end

            % Display the desired and actual poses for debugging/verification
            disp('Desired pose:');
            disp(self.desiredPoseSixteen);

            disp('Actual pose:');
            disp(self.actualPoseSixteen);

            % Display the computed position error in meters
            disp('Position error:');
            disp(self.positionErrorSixteen);

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

                if self.app.eStopFlag == 1
                    
                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position
                    
                else
                    % Animate the robot movement for the j-th step
                    self.robot1.model.animate(path1(j, :));
                end

               

                % Get the current transformation of the end-effector and adjust for gripper orientation
                Tr = self.robot1.model.fkine(path1(j, :)).T * troty(-pi/2);

                tr = self.robot1.model.fkine(path1(j,:));

                rotation = trotx(pi/2);

                offset = transl(0,0.1,0);

                trEdgeGrasp = tr.T * rotation * offset;

                self.transformedVertices = [self.plates.plateVertices{plateIndex}, ones(size(self.plates.plateVertices{plateIndex},1),1)] * trEdgeGrasp';

                set(self.plates.plates_h{plateIndex}, 'Vertices', self.transformedVertices(:,1:3));


                % Animate the gripper with no delay (gripper movement in sync with the robot)
                self.gripper.robot.delay = 0;
                self.gripper.animateGripper(Tr);  % Move the gripper along with the robot's end-effector
                axis equal  % Keep equal axis proportions for proper visualization
                drawnow();  % Update the figure window in real-time
                pause(0.01);  % Add a small pause to control the animation speed

                % Delete the previously displayed text handle if it exists
                try delete(self.text_hSixteen); end  %#ok<TRYNC>

                % Compute and display the final transformation matrix at the last step
                Tr1 = self.robot1.model.fkine(path1(size(self.stepsSixteen, 1), :)).T;  % Get the transformation matrix at the final step
                try delete(Tr1); end  %#ok<TRYNC>  % Try to delete Tr1 if needed

                % Create a message displaying the final pose information
                message = sprintf([num2str(round(Tr1(1, :), 2, 'significant')), '\n' ...
                    num2str(round(Tr1(2, :), 2, 'significant')), '\n' ...
                    num2str(round(Tr1(3, :), 2, 'significant'))]);

                % Display the final pose as text on the plot
                self.text_hSixteen = text(0, 0, 3, message, 'FontSize', 10, 'Color', [.6 .2 .6]);  % Text display in purple color

                drawnow();  % Update the figure window to display the text
            end

            % Update the starting joint configuration to the final configuration for the next motion
            self.qStartSixteen = self.qEndSixteen;
        end


        function HeadingToDrop(self, plateIndex)
            % Move to the first waypoint above the plate pickup point.
            disp("STATUS: MOVING TO HOME POSITION")
            fprintf('\n');

            % Define the desired transformation matrix (pose) above the pickup point

            t1 = transl(-0.3, 0.35, 1.6)*rpy2tr(90, 0, 180, 'deg'), 'q0', self.qStartSixteen, 'mask', [0,0,0,0,0,0];
            self.qEndSixteen = self.robot1.model.ikine(t1);

            % Compute actual joint state after inverse kinematics (ikcon)
            self.tActualSixteen = self.robot1.model.fkine(self.qEndSixteen).T;  % Get the transformation matrix of the current joint configuration
            self.desiredPoseSixteen = t1(1:3, 4);  % Extract the desired end-effector position (x, y, z)
            self.actualPoseSixteen = self.tActualSixteen(1:3, 4);  % Extract the actual end-effector position (x, y, z) from forward kinematics

            % Compute the position error between the desired and actual end-effector positions
            self.positionErrorSixteen = norm(self.desiredPoseSixteen - self.actualPoseSixteen);  % Euclidean distance between desired and actual positions

            % Check if the error is within the acceptable tolerance of 5mm
            if self.positionErrorSixteen <= self.toleranceSixteen
                disp('Pose satisfied within +- 5mm.');  % Display success message if within tolerance
            else
                disp('Pose does not meet the +-5mm tolerance.');  % Display warning if error exceeds tolerance
            end

            % Display the desired and actual poses for debugging/verification
            disp('Desired pose:');
            disp(self.desiredPoseSixteen);

            disp('Actual pose:');
            disp(self.actualPoseSixteen);

            % Display the computed position error in meters
            disp('Position error:');
            disp(self.positionErrorSixteen);

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

                if self.app.eStopFlag == 1
                    
                     currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position
                    
                else
                    % Animate the robot movement for the j-th step
                    self.robot1.model.animate(path1(j, :));
                end

                

                % Get the current transformation of the end-effector and adjust for gripper orientation
                Tr = self.robot1.model.fkine(path1(j, :)).T * troty(-pi/2);

                tr = self.robot1.model.fkine(path1(j,:));

                rotation = trotx(pi/2);

                offset = transl(0,0.1,0);

                trEdgeGrasp = tr.T * rotation * offset;

                self.transformedVertices = [self.plates.plateVertices{plateIndex}, ones(size(self.plates.plateVertices{plateIndex},1),1)] * trEdgeGrasp';

                set(self.plates.plates_h{plateIndex}, 'Vertices', self.transformedVertices(:,1:3));


                % Animate the gripper with no delay (gripper movement in sync with the robot)
                self.gripper.robot.delay = 0;
                self.gripper.animateGripper(Tr);  % Move the gripper along with the robot's end-effector
                axis equal  % Keep equal axis proportions for proper visualization
                drawnow();  % Update the figure window in real-time
                pause(0.01);  % Add a small pause to control the animation speed

                % Delete the previously displayed text handle if it exists
                try delete(self.text_hSixteen); end  %#ok<TRYNC>

                % Compute and display the final transformation matrix at the last step
                Tr1 = self.robot1.model.fkine(path1(size(self.stepsSixteen, 1), :)).T;  % Get the transformation matrix at the final step
                try delete(Tr1); end  %#ok<TRYNC>  % Try to delete Tr1 if needed

                % Create a message displaying the final pose information
                message = sprintf([num2str(round(Tr1(1, :), 2, 'significant')), '\n' ...
                    num2str(round(Tr1(2, :), 2, 'significant')), '\n' ...
                    num2str(round(Tr1(3, :), 2, 'significant'))]);

                % Display the final pose as text on the plot
                self.text_hSixteen = text(0, 0, 3, message, 'FontSize', 10, 'Color', [.6 .2 .6]);  % Text display in purple color

                drawnow();  % Update the figure window to display the text
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

            % Compute the actual joint state using forward kinematics after inverse kinematics
            self.tActualSixteen = self.robot1.model.fkine(self.qEndSixteen).T;  % Get the actual transformation matrix
            self.desiredPoseSixteen = t2(1:3, 4);  % Extract the desired position (x, y, z) from transformation matrix
            self.actualPoseSixteen = self.tActualSixteen(1:3, 4);  % Extract the actual position from the forward kinematics result

            % % Generate a smooth trajectory between current and desired joint configurations
            % path1 = nan(self.stepsSixteen, 6);
            % for k = 1:6
            %     path1(:, k) = linspace(self.qStartSixteen(k), self.qEndSixteen(k), self.stepsSixteen);
            % end
            

            % Compute the position error between the desired and actual positions
            self.positionErrorSixteen = norm(self.desiredPoseSixteen - self.actualPoseSixteen);  % Calculate Euclidean distance

            % Check if the error is within the tolerance (5mm)
            if self.positionErrorSixteen <= self.toleranceSixteen
                disp('Pose satisfied within +- 5mm.');  % Success message
            else
                disp('Pose does not meet the +-5mm tolerance.');  % Error message if tolerance is not met
            end

            % Display the desired and actual positions for debugging
            disp('Desired pose:');
            disp(self.desiredPoseSixteen);

            disp('Actual pose:');
            disp(self.actualPoseSixteen);

            disp('Position error:');
            disp(self.positionErrorSixteen);

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

                if self.app.eStopFlag == 1
                    
                     currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position
                    
                else
                    % Animate the robot movement for the j-th step
                    self.robot1.model.animate(path2(j, :));
                end

               
                % Tr = self.robot1.model.fkine(path2(j,:)).T * troty(-pi/2);  % Get the transformation matrix and rotate the gripper
                % self.gripper.robot.delay = 0;  % No delay for gripper animation
                % self.gripper.animateGripper(Tr);  % Animate the gripper to follow the end-effector
                axis equal;  % Keep axis scaling equal for proper visualization
                drawnow();  % Update the figure window for real-time animation
                pause(0.01);  % Pause for a short duration to control animation speed

                % % Try to delete any previous text handle if it exists
                % try delete(self.text_hSixteen); end %#ok<TRYNC>
                % 
                % % Compute and display the final transformation at the last step
                % Tr2 = self.robot1.model.fkine(path2(size(self.stepsSixteen,1),:)).T;  % Get the final transformation matrix
                % try delete(Tr2); end %#ok<TRYNC>  % Try to delete Tr2 if it exists
                % 
                % % Create a message showing the final pose information
                % message = sprintf([num2str(round(Tr2(1,:),2,'significant')),'\n' ...
                %     num2str(round(Tr2(2,:),2,'significant')),'\n' ...
                %     num2str(round(Tr2(3,:),2,'significant'))]);
                % 
                % % Display the final pose as text in the plot
                % self.text_hSixteen = text(0, 0, 3, message, 'FontSize', 10, 'Color', [.6 .2 .6]);  % Purple text
                drawnow();  % Update the figure window to show the text
            end

            % Update the starting configuration for the next move
            self.qStartSixteen = self.qEndSixteen;

        end




        function AboveDropPoint(self, plateIndex)
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

               if self.app.eStopFlag == 1
                    
                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position
                    
               else
                    % Animate the robot movement for the j-th step
                    self.robot1.model.animate(path3(j, :));
               end

               tr = self.robot1.model.fkine(path3(j,:));

                rotation = trotx(pi/2);

                offset = transl(0,0.1,0);

                trEdgeGrasp = tr.T * rotation * offset;

                self.transformedVertices = [self.plates.plateVertices{plateIndex}, ones(size(self.plates.plateVertices{plateIndex},1),1)] * trEdgeGrasp';

                set(self.plates.plates_h{plateIndex}, 'Vertices', self.transformedVertices(:,1:3));

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

            % Compute the actual joint state using forward kinematics after inverse kinematics
            self.tActualSixteen = self.robot1.model.fkine(self.qEndSixteen).T;  % Get the actual transformation matrix
            self.desiredPoseSixteen = t4(1:3, 4);  % Extract the desired position (x, y, z)
            self.actualPoseSixteen = self.tActualSixteen(1:3, 4);  % Extract the actual position

            % Compute the position error between the desired and actual positions
            self.positionErrorSixteen = norm(self.desiredPoseSixteen - self.actualPoseSixteen);  % Euclidean distance

            % Check if the position error is within the tolerance (5mm)
            if self.positionErrorSixteen <= self.toleranceSixteen
                disp('Pose satisfied within +- 5mm.');
            else
                disp('Pose does not meet the +-5mm tolerance.');
            end

            % Display the desired and actual positions for debugging purposes
            disp('Desired pose:');
            disp(self.desiredPoseSixteen);

            disp('Actual pose:');
            disp(self.actualPoseSixteen);

            disp('Position error:');
            disp(self.positionErrorSixteen);

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

                if self.app.eStopFlag == 1

                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position

                else
                    % Animate the robot movement for the j-th step
                    self.robot1.model.animate(path4(j, :));
                end

                self.robot1.model.animate(path4(j, :));  % Animate the robot joint states
                Tr = self.robot1.model.fkine(path4(j,:)).T * troty(-pi/2);  % Get the transformation matrix for the gripper
                self.gripper.robot.delay = 0;  % Set the delay for gripper animation
                self.gripper.animateGripper(Tr);  % Animate the gripper following the end-effector
                axis equal;  % Keep axis scaling uniform
                drawnow();  % Update the plot

                % Add a small pause to control the animation speed
                pause(0.01);

                % Try to delete the previous text handle to avoid overlap
                try delete(self.text_hSixteen); end %#ok<TRYNC>

                % Compute the final transformation at the last step of the trajectory
                Tr4 = self.robot1.model.fkine(path4(size(self.stepsSixteen,1),:)).T;
                try delete(Tr4); end %#ok<TRYNC>  % Try deleting any residual transformations

                % Create and display the final pose information as a message
                message = sprintf([num2str(round(Tr4(1,:),2,'significant')),'\n' ...
                    num2str(round(Tr4(2,:),2,'significant')),'\n' ...
                    num2str(round(Tr4(3,:),2,'significant'))]);
                self.text_hSixteen = text(0, 0, 3, message, 'FontSize', 10, 'Color', [.6 .2 .6]);  % Display message with purple text
                drawnow();  % Update the plot
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

            % Compute the actual joint state using forward kinematics
            self.tActualSixteen = self.robot1.model.fkine(self.qEndSixteen).T;  % Get the actual transformation matrix
            self.desiredPoseSixteen = t5(1:3, 4);  % Extract the desired position (x, y, z)
            self.actualPoseSixteen = self.tActualSixteen(1:3, 4);  % Extract the actual position

            % Compute the position error between the desired and actual positions
            self.positionErrorSixteen = norm(self.desiredPoseSixteen - self.actualPoseSixteen);  % Euclidean distance

            % Check if the position error is within the tolerance (5mm)
            if self.positionErrorSixteen <= self.toleranceSixteen
                disp('Pose satisfied within +- 5mm.');
            else
                disp('Pose does not meet the +-5mm tolerance.');
            end

            % Display the desired and actual positions for debugging purposes
            disp('Desired pose:');
            disp(self.desiredPoseSixteen);

            disp('Actual pose:');
            disp(self.actualPoseSixteen);

            disp('Position error:');
            disp(self.positionErrorSixteen);

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

                if self.app.eStopFlag == 1
                    
                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position
                    
                else
                    % Animate the robot movement for the j-th step
                    self.robot1.model.animate(path5(j, :));
                end

                
                Tr = self.robot1.model.fkine(path5(j,:)).T * troty(-pi/2);  % Get the transformation matrix for the gripper
                self.gripper.robot.delay = 0;  % Set the delay for the gripper
                self.gripper.animateGripper(Tr);  % Animate the gripper following the end-effector
                axis equal;  % Keep axis scaling uniform
                drawnow();  % Update the plot

                % Add a small pause to control the animation speed
                pause(0.01);

                % Try to delete the previous text handle to avoid overlap
                try delete(self.text_hSixteen); end %#ok<TRYNC>

                % Compute the final transformation at the last step
                Tr5 = self.robot1.model.fkine(path5(size(self.stepsSixteen,1),:)).T;
                try delete(Tr5); end %#ok<TRYNC>  % Try deleting any residual transformations

                % Create and display the final pose information as a message
                message = sprintf([num2str(round(Tr5(1,:),2,'significant')),'\n' ...
                    num2str(round(Tr5(2,:),2,'significant')),'\n' ...
                    num2str(round(Tr5(3,:),2,'significant'))]);
                self.text_hSixteen = text(0, 0, 3, message, 'FontSize', 10, 'Color', [.6 .2 .6]);  % Display message with purple text
                drawnow();  % Update the plot
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

            % Compute actual joint state after inverse kinematics (ikcon)
            self.tActualThree = self.robot2.model.fkine(self.qEndThree).T;  % Get the transformation matrix of the current joint configuration
            self.desiredPoseThree = t1(1:3, 4);  % Extract the desired end-effector position (x, y, z)
            self.actualPoseThree = self.tActualThree(1:3, 4);  % Extract the actual end-effector position (x, y, z) from forward kinematics

            % Compute the position error between the desired and actual end-effector positions
            self.positionErrorThree = norm(self.desiredPoseThree - self.actualPoseThree);  % Euclidean distance between desired and actual positions

            % Check if the error is within the acceptable tolerance of 5mm
            if self.positionErrorThree <= self.toleranceThree
                disp('Pose satisfied within +- 5mm.');  % Display success message if within tolerance
            else
                disp('Pose does not meet the +-5mm tolerance.');  % Display warning if error exceeds tolerance
            end

            % Display the desired and actual poses for debugging/verification
            disp('Desired pose:');
            disp(self.desiredPoseThree);

            disp('Actual pose:');
            disp(self.actualPoseThree);

            % Display the computed position error in meters
            disp('Position error:');
            disp(self.positionErrorThree);

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

                if self.app.eStopFlag == 1
                    
                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position
                    
                else
                    % Animate the robot movement for the j-th step
                    self.robot2.model.animate(path1(j, :));
                end

                % Animate the robot movement for the j-th step
                self.robot2.model.animate(path1(j, :));

                % Get the current transformation of the end-effector and adjust for gripper orientation
                % Tr = self.robot2.model.fkine(path1(j, :)).T * troty(-pi/2);

                % Animate the gripper with no delay (gripper movement in sync with the robot)
                % self.gripper.robot.delay = 0;
                % self.gripper.animateGripper(Tr);  % Move the gripper along with the robot's end-effector
                axis equal  % Keep equal axis proportions for proper visualization
                drawnow();  % Update the figure window in real-time
                pause(0.01);  % Add a small pause to control the animation speed

                % % Delete the previously displayed text handle if it exists
                % try delete(self.text_hThree); end  %#ok<TRYNC>
                % 
                % % Compute and display the final transformation matrix at the last step
                % Tr1 = self.robot2.model.fkine(path1(size(self.stepsThree, 1), :)).T;  % Get the transformation matrix at the final step
                % try delete(Tr1); end  %#ok<TRYNC>  % Try to delete Tr1 if needed
                % 
                % % Create a message displaying the final pose information
                % message = sprintf([num2str(round(Tr1(1, :), 2, 'significant')), '\n' ...
                %     num2str(round(Tr1(2, :), 2, 'significant')), '\n' ...
                %     num2str(round(Tr1(3, :), 2, 'significant'))]);
                % 
                % % Display the final pose as text on the plot
                % self.text_hThree = text(0, 0, 3, message, 'FontSize', 10, 'Color', [.6 .2 .6]);  % Text display in purple color

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

            % Compute the actual joint state using forward kinematics after inverse kinematics
            self.tActualThree = self.robot2.model.fkine(self.qEndThree).T;  % Get the actual transformation matrix
            self.desiredPoseThree = t3(1:3, 4);  % Extract the desired position (x, y, z) from transformation matrix
            self.actualPoseThree = self.tActualThree(1:3, 4);  % Extract the actual position from the forward kinematics result

            % Compute the position error between the desired and actual positions
            self.positionErrorThree = norm(self.desiredPoseThree - self.actualPoseThree);  % Calculate Euclidean distance

            % Check if the error is within the tolerance (5mm)
            if self.positionErrorThree <= self.toleranceThree
                disp('Pose satisfied within +- 5mm.');  % Success message
            else
                disp('Pose does not meet the +-5mm tolerance.');  % Error message if tolerance is not met
            end

            % Display the desired and actual positions for debugging
            disp('Desired pose:');
            disp(self.desiredPoseThree);

            disp('Actual pose:');
            disp(self.actualPoseThree);

            disp('Position error:');
            disp(self.positionErrorThree);

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

                if self.app.eStopFlag == 1
                    
                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position
                    
                else
                    % Animate the robot movement for the j-th step
                    self.robot2.model.animate(path3(j, :));
                end

                % Tr = self.robot2.model.fkine(path3(j,:)).T * troty(-pi/2);  % Get the transformation matrix and rotate the gripper
                % self.gripper.robot.delay = 0;  % No delay for gripper animation
                % self.gripper.animateGripper(Tr);  % Animate the gripper to follow the end-effector
                axis equal;  % Keep axis scaling equal for proper visualization
                drawnow();  % Update the figure window for real-time animation
                pause(0.01);  % Pause for a short duration to control animation speed

                % % Try to delete any previous text handle if it exists
                % try delete(self.text_hThree); end %#ok<TRYNC>
                % 
                % % Compute and display the final transformation at the last step
                % Tr3 = self.robot2.model.fkine(path3(size(self.stepsThree,1),:)).T;  % Get the final transformation matrix
                % try delete(Tr3); end %#ok<TRYNC>  % Try to delete Tr3 if it exists
                % 
                % % Create a message showing the final pose information
                % message = sprintf([num2str(round(Tr3(1,:),2,'significant')),'\n' ...
                %     num2str(round(Tr3(2,:),2,'significant')),'\n' ...
                %     num2str(round(Tr3(3,:),2,'significant'))]);
                % 
                % % Display the final pose as text in the plot
                % self.text_hThree = text(0, 0, 3, message, 'FontSize', 10, 'Color', [.6 .2 .6]);  % Purple text
                drawnow();  % Update the figure window to show the text
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

            % Compute the actual joint state using forward kinematics after inverse kinematics
            self.tActualThree = self.robot2.model.fkine(self.qEndThree).T;  % Get the actual transformation matrix
            self.desiredPoseThree = t3(1:3, 4);  % Extract the desired position (x, y, z) from transformation matrix
            self.actualPoseThree = self.tActualThree(1:3, 4);  % Extract the actual position from the forward kinematics result

            % Compute the position error between the desired and actual positions
            self.positionErrorThree = norm(self.desiredPoseThree - self.actualPoseThree);  % Calculate Euclidean distance

            % Check if the error is within the tolerance (5mm)
            if self.positionErrorThree <= self.toleranceThree
                disp('Pose satisfied within +- 5mm.');  % Success message
            else
                disp('Pose does not meet the +-5mm tolerance.');  % Error message if tolerance is not met
            end

            % Display the desired and actual positions for debugging
            disp('Desired pose:');
            disp(self.desiredPoseThree);

            disp('Actual pose:');
            disp(self.actualPoseThree);

            disp('Position error:');
            disp(self.positionErrorThree);

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

                if self.app.eStopFlag == 1
                    
                  currentstep = self.robot1.model.getpos();  % Get current joint positions
                  self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position
                    
                else
                    % Animate the robot movement for the j-th step
                    self.robot2.model.animate(path3(j, :));
                end

                self.robot2.model.animate(path3(j, :));  % Animate the robot at each step
                % Tr = self.robot2.model.fkine(path3(j,:)).T * troty(-pi/2);  % Get the transformation matrix and rotate the gripper
                % self.gripper.robot.delay = 0;  % No delay for gripper animation
                % self.gripper.animateGripper(Tr);  % Animate the gripper to follow the end-effector
                axis equal;  % Keep axis scaling equal for proper visualization
                drawnow();  % Update the figure window for real-time animation
                pause(0.01);  % Pause for a short duration to control animation speed
                % 
                % % Try to delete any previous text handle if it exists
                % try delete(self.text_hThree); end %#ok<TRYNC>
                % 
                % % Compute and display the final transformation at the last step
                % Tr3 = self.robot2.model.fkine(path3(size(self.stepsThree,1),:)).T;  % Get the final transformation matrix
                % try delete(Tr3); end %#ok<TRYNC>  % Try to delete Tr3 if it exists
                % 
                % % Create a message showing the final pose information
                % message = sprintf([num2str(round(Tr3(1,:),2,'significant')),'\n' ...
                %     num2str(round(Tr3(2,:),2,'significant')),'\n' ...
                %     num2str(round(Tr3(3,:),2,'significant'))]);
                % 
                % % Display the final pose as text in the plot
                % self.text_hThree = text(0, 0, 3, message, 'FontSize', 10, 'Color', [.6 .2 .6]);  % Purple text
                drawnow();  % Update the figure window to show the text
            end

            % Update the starting configuration for the next move
            self.qStartThree = self.qEndThree;

        end

        function POC3(self)
            % Move to the dropoff zone above the target
            disp("STATUS: MOVING TO DROP OFF ZONE");  % Display the current action
            fprintf('\n');  % New line for cleaner output

            % Generate the transformation to move above the dropoff zone
            t3 = transl([-0.7 0.7800 1.45]) * rpy2tr(180,0,0,'deg');
            self.qEndThree = self.robot2.model.ikcon(t3);  % Calculate inverse kinematics to find joint configuration

            % Compute the actual joint state using forward kinematics after inverse kinematics
            self.tActualThree = self.robot2.model.fkine(self.qEndThree).T;  % Get the actual transformation matrix
            self.desiredPoseThree = t3(1:3, 4);  % Extract the desired position (x, y, z) from transformation matrix
            self.actualPoseThree = self.tActualThree(1:3, 4);  % Extract the actual position from the forward kinematics result

            % Compute the position error between the desired and actual positions
            self.positionErrorThree = norm(self.desiredPoseThree - self.actualPoseThree);  % Calculate Euclidean distance

            % Check if the error is within the tolerance (5mm)
            if self.positionErrorThree <= self.toleranceThree
                disp('Pose satisfied within +- 5mm.');  % Success message
            else
                disp('Pose does not meet the +-5mm tolerance.');  % Error message if tolerance is not met
            end

            % Display the desired and actual positions for debugging
            disp('Desired pose:');
            disp(self.desiredPoseThree);

            disp('Actual pose:');
            disp(self.actualPoseThree);

            disp('Position error:');
            disp(self.positionErrorThree);

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

                if self.app.eStopFlag == 1

                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position

                else
                    % Animate the robot movement for the j-th step
                    self.robot2.model.animate(path3(j, :));
                end

                
                % Tr = self.robot2.model.fkine(path3(j,:)).T * troty(-pi/2);  % Get the transformation matrix and rotate the gripper
                % self.gripper.robot.delay = 0;  % No delay for gripper animation
                % self.gripper.animateGripper(Tr);  % Animate the gripper to follow the end-effector
                axis equal;  % Keep axis scaling equal for proper visualization
                drawnow();  % Update the figure window for real-time animation
                pause(0.01);  % Pause for a short duration to control animation speed

                % Try to delete any previous text handle if it exists
                % try delete(self.text_hThree); end %#ok<TRYNC>
                % 
                % % Compute and display the final transformation at the last step
                % Tr3 = self.robot2.model.fkine(path3(size(self.stepsThree,1),:)).T;  % Get the final transformation matrix
                % try delete(Tr3); end %#ok<TRYNC>  % Try to delete Tr3 if it exists
                % 
                % % Create a message showing the final pose information
                % message = sprintf([num2str(round(Tr3(1,:),2,'significant')),'\n' ...
                %     num2str(round(Tr3(2,:),2,'significant')),'\n' ...
                %     num2str(round(Tr3(3,:),2,'significant'))]);
                % 
                % % Display the final pose as text in the plot
                % self.text_hThree = text(0, 0, 3, message, 'FontSize', 10, 'Color', [.6 .2 .6]);  % Purple text
                drawnow();  % Update the figure window to show the text
            end

            % Update the starting configuration for the next move
            self.qStartThree = self.qEndThree;

        end




        function ReturnHome(self)
            % Return the robot to the home position after finishing the task
            disp("STATUS: RETURNING HOME");  % Display the status message
            fprintf('\n');  % New line for clean output formatting

            % Calculate the transformation matrix for the home position
            t5 = transl([-0.75, 0.5, 0.5]) * rpy2tr(180,0,0,'deg');
            self.qEndThree = self.robot2.model.ikcon(t5);  % Compute the joint angles using inverse kinematics

            % Compute the actual joint state using forward kinematics
            self.tActualThree = self.robot2.model.fkine(self.qEndThree).T;  % Get the actual transformation matrix
            self.desiredPoseThree = t5(1:3, 4);  % Extract the desired position (x, y, z)
            self.actualPoseThree = self.tActualThree(1:3, 4);  % Extract the actual position

            % Compute the position error between the desired and actual positions
            self.positionErrorThree = norm(self.desiredPoseThree - self.actualPoseThree);  % Euclidean distance

            % Check if the position error is within the tolerance (5mm)
            if self.positionErrorThree <= self.toleranceThree
                disp('Pose satisfied within +- 5mm.');
            else
                disp('Pose does not meet the +-5mm tolerance.');
            end

            % Display the desired and actual positions for debugging purposes
            disp('Desired pose:');
            disp(self.desiredPoseThree);

            disp('Actual pose:');
            disp(self.actualPoseThree);

            disp('Position error:');
            disp(self.positionErrorThree);

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

                if self.app.eStopFlag == 1
                    
                    currentstep = self.robot1.model.getpos();  % Get current joint positions
                    self.robot1.model.plot(currentstep);  % Animate and hold the robot at the current position
                    
                else
                    % Animate the robot movement for the j-th step
                    self.robot2.model.animate(path5(j, :));
                end

                Tr = self.robot2.model.fkine(path5(j,:)).T * troty(-pi/2);  % Get the transformation matrix for the gripper
                self.gripper.robot.delay = 0;  % Set the delay for the gripper
                self.gripper.animateGripper(Tr);  % Animate the gripper following the end-effector
                axis equal;  % Keep axis scaling uniform
                drawnow();  % Update the plot

                % Add a small pause to control the animation speed
                pause(0.01);

                % Try to delete the previous text handle to avoid overlap
                try delete(self.text_hThree); end %#ok<TRYNC>

                % Compute the final transformation at the last step
                Tr5 = self.robot2.model.fkine(path5(size(self.stepsThree,1),:)).T;
                try delete(Tr5); end %#ok<TRYNC>  % Try deleting any residual transformations

                % Create and display the final pose information as a message
                message = sprintf([num2str(round(Tr5(1,:),2,'significant')),'\n' ...
                    num2str(round(Tr5(2,:),2,'significant')),'\n' ...
                    num2str(round(Tr5(3,:),2,'significant'))]);
                self.text_hThree = text(0, 0, 3, message, 'FontSize', 10, 'Color', [.6 .2 .6]);  % Display message with purple text
                drawnow();  % Update the plot
            end

            % Update the starting joint configuration for the next move
            self.qStartThree = self.qEndThree;
        end


    end
end


