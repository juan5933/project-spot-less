classdef RealRobot < handle
    properties
        client                % ROS action client for UR3e trajectory control
        jointStateSubscriber  % Subscriber to UR3e joint states topic
        jointNames            % Names of the robot joints
        qCurrent              % Current joint state
        qInitial              % Initial joint state
        durationSeconds = 10; % Movement duration in seconds
        bufferSeconds = 1;    % Buffer time to account for network delays
        openService
        closeService
    end

    methods
        function self = RealRobot()
            % Constructor: Initialize ROS connection, subscribers, and client.
            rosinit('192.168.27.1');  % Connect to ROS network
            self.jointStateSubscriber = rossubscriber('/ur/joint_states','sensor_msgs/JointState');
            self.client = rosactionclient('/ur/scaled_pos_joint_traj_controller/follow_joint_trajectory');
            self.jointNames = {'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};
            self.qInitial = [-pi -pi/2 0 pi 0 0];  % Define initial joint state
           
            self.openService = rossvcclient("/onrobot/open", "std_srvs/Trigger");
            self.closeService = rossvcclient("/onrobot/close", "std_srvs/Trigger");
        end

        function updateCurrentState(self)
            % Updates the current joint state from the ROS joint state topic
            currentJointState_321456 = (self.jointStateSubscriber.LatestMessage.Position)';
            self.qCurrent = [currentJointState_321456(3:-1:1), currentJointState_321456(4:6)];
        end

        function moveToState(self, qTarget)
            % Moves the robot to the desired joint state qTarget using ROS trajectory action
            startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            startJointSend.Positions = self.qCurrent;
            startJointSend.TimeFromStart = rosduration(0);
            
            endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
            endJointSend.Positions = qTarget;
            endJointSend.TimeFromStart = rosduration(self.durationSeconds);
            
            goal = rosmessage('control_msgs/FollowJointTrajectoryGoal');
            goal.Trajectory.JointNames = self.jointNames;
            goal.Trajectory.Points = [startJointSend; endJointSend];
            goal.Trajectory.Header.Stamp = self.jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(self.bufferSeconds);
            
            % Send the trajectory to the robot
            sendGoal(self.client, goal);
        end

        function moveWithAdjustment(self, jointAdjustments)
            % Moves the robot by adjusting the current joint state using the specified adjustments.
            self.updateCurrentState();  % Get the latest joint state
            qTarget = self.qCurrent + jointAdjustments;  % Apply joint adjustments
            self.moveToState(qTarget);  % Move to the new joint state
        end

        function controlGripperOpen(self)
            self.openService.call();
        end

        function controlGripperClose(self)
            self.closeService.call();
        end

        % Step-by-step Methods
        function moveToInitialPosition(self)
            disp('Moving to initial position...');
            self.moveToState(self.qInitial);
        end

        function moveToPickUpPosition(self)
            disp('Moving to pick-up position...');
            movement_adjustment = [pi/2, 0, pi/2, 0, -pi/2, 0];
            self.moveWithAdjustment(movement_adjustment);
        end

        function moveToAbovePickUp(self)
            disp('Moving above pick-up point...');
            self.moveWithAdjustment([pi/2, 0, pi/2, 0, -pi/2, 0]);
        end

        function moveToUnderTap(self)
            disp('Moving under tap...');
            self.moveWithAdjustment([pi/2, 0, pi/2, 0, -pi/2, 0]);
        end

        function rinsePositionDown(self)
            disp('Rinsing plate...');
            self.moveWithAdjustment([pi/2, 0, pi/2, 0, -pi/2, 0]);
        end

        function moveToCleaningPosition(self)
            disp('Moving to cleaning position...');
            self.moveWithAdjustment([pi/2, 0, pi/2, 0, -pi/2, 0]);
        end

        function rotateUpForCleaning(self)
            disp('Rotating up for cleaning...');
            self.moveWithAdjustment([pi/2, 0, pi/2, 0, -pi/2, 0]);
        end

        function finalRinseUp(self)
            disp('Final rinse...');
            self.moveWithAdjustment([pi/2, 0, pi/2, 0, -pi/2, 0]);
        end

        function finalRinseDown(self)
            disp('Final rinse down...');
            self.moveWithAdjustment([pi/2, 0, pi/2, 0, -pi/2, 0]);
        end
        
        function scanningPlatePos(self)
            disp('Scanning plate...');
            self.moveWithAdjustment([pi/2, 0, pi/2, 0, -pi/2, 0]);
        end

        % function dirtyPlate(self)
        %     if binaryValue = 1 
        %            self.moveToUnderTap();
        %            self.rinsePositionDown(self);
        % 
        %     else  
        %            self.placePlateDown();
        % end 


        function placePlateDown(self)
            disp('Placing Plate down...');
            self.moveWithAdjustment([pi/2, 0, pi/2, 0, -pi/2, 0]);
        end

        function returnHome(self)
            disp('Returning To Homing Position...');
            self.moveWithAdjustment([pi/2, 0, pi/2, 0, -pi/2, 0]);
        end

        % Execute all steps in sequence
        function executeTask(self)
            self.moveToInitialPosition();
            self.moveToPickUpPosition();
            self.openGripper();
            self.moveToAbovePickUp();
            self.moveToUnderTap();
            self.rinsePositionDown();
            self.moveToCleaningPosition();
            self.rotateUpForCleaning();
            self.finalRinseUp();
            self.finalRinseDown();
            self.scanningPlatePos();
          % self.dirtyPlate();
            self.placePlateDown();
            self.returnHome();
            disp('Task sequence completed.');
        end

        function shutdown(self)
            rosshutdown();  % Cleanly shuts down the ROS connection
        end
    end
end

