%% A1 Lab Demo
% Initialisation phase: 
% Connect to Rasberry Pi via ROS and subscribe to joint topics
rosinit('192.168.27.1'); 
jointStateSubscriber = rossubscriber('/ur/joint_states','sensor_msgs/JointState');
pause(2);

%% Initial Calibration
% USE THIS AT ORIGIN TO SET AS CURRENT STATE 
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];
 
%% IF FAIL TO CONNECT - CHECK LATEST MESSAGE 
jointStateSubscriber.LatestMessage % Run in command terminal

%% Joint command names - matches up to joint number in array
jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};





%% STEP 1 - Get the arm into 'picking up ball' state 
% Reinstate the current position
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; 
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];   %(3:-1)



% Set up the action: start to end = goal trajectory
[client, goal] = rosactionclient('/ur/scaled_pos_joint_traj_controller/follow_joint_trajectory');
goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now','system');
goal.GoalTimeTolerance = rosduration(0.05);
bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
durationSeconds = 10; % This is how many seconds the movement will take (WARNING: Allow enough time for safety)
% Create ros message                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                    




startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     
%
endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

movement_adjustment = [0, -pi/2, 0, -pi/2, 0, 0];                         % joint rotations
nextJointState_123456 = currentJointState_123456 + movement_adjustment;     % offset not needed but added here
endJointSend.Positions = nextJointState_123456;                             %assigning positions to endJointSend rostopic
endJointSend.TimeFromStart = rosduration(durationSeconds);                  %assigning timefromstart to endJointSend rostopic ie 15seconds

goal.Trajectory.Points = [startJointSend; endJointSend];                    %setting trajectory goals ie start and end joint configs.

%% Run compiled trajectory (Step 1)
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);    %sending trajectory goals to the robot
sendGoal(client,goal);

%% Activating claw - connecting to the corresponding ros clients
openService = rossvcclient("/onrobot/open", "std_srvs/Trigger");            % Create ROS service clients for open/close services
closeService = rossvcclient("/onrobot/close", "std_srvs/Trigger");

pause(1);                                                                  %pause for safety

%% Close claw command
closeService.call();        %close claw






%% Step 2 - Rotate to new position
% Save the new moved position as current state 
% WARNING: If not done, the UR3 bot will snap to origin (last saved point)
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)';
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);

movement_adjustment = [-pi/5, 0, 0, 0, 0, pi];           %second position q values ie rotation values
nextJointState_123456 = currentJointState_123456 + movement_adjustment;
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];

%% Run compiled trajectory (Step 2)
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);

%% Open claw command
% Pause for safety
pause(1);
openService.call();










%% RESET FUNCTIONS
% If not posting properly, use ... then re-initialise
%rosshutdown;