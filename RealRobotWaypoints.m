%% RESET FUNCTIONS
% If not posting properly, use ... then re-initialise
rosshutdown;
%% To get the current joint state from the real robot
% Initialisation phase: 
% Connect to Rasberry Pi via ROS and subscribe to joint topics
rosinit('192.168.27.1'); 

%%
jointStateSubscriber = rossubscriber('/ur/joint_states','sensor_msgs/JointState');

%% Initial Calibration
% USE THIS AT ORIGIN TO SET AS CURRENT STATE 
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; % Note the default order of the joints is 3,2,1,4,5,6
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

%% IF FAIL TO CONNECT - CHECK LATEST MESSAGE 
jointStateSubscriber.LatestMessage % Run in command terminal

%% Joint command names - matches up to joint number in array
jointNames = {'shoulder_pan_joint','shoulder_lift_joint', 'elbow_joint', 'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'};


%% Put Into Initial Position

currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)';
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

[client, goal] = rosactionclient('/ur/scaled_pos_joint_traj_controller/follow_joint_trajectory');
goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now','system');
goal.GoalTimeTolerance = rosduration(0.05);
bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
durationSeconds = 4; % This is how many seconds the movement will take (WARNING: Allow enough time for safety)
% Create ros message
startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);


nextJointState_123456 = [-pi -pi/2 0 -pi/2 0 0];
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];


%%  step 0 this doesn't move
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);

%% Activating claw - connecting to the corresponding ros clients
openService = rossvcclient("/onrobot/open", "std_srvs/Trigger");
closeService = rossvcclient("/onrobot/close", "std_srvs/Trigger");

%% 4) CLose or open gripper
% Pause for safety
%pause(10);
openService.call();

%pause(10);
%Close claw command
%closeService.call();


%% 4) CLose or open gripper
% Pause for safety
%pause(10);
closeService.call();

%pause(10);
%Close claw command
%closeService.call();


%% 1) Get the arm into 'in front picking up plate' state 
% Reinstate the current position
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; 
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

% Create ros message
startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

movement_adjustment = [-150.2*pi/180, -161.9*pi/180, -50*pi/180, -76*pi/180, 3.86*pi/180, 15*pi/180];
nextJointState_123456 = movement_adjustment;
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];

%% Run compiled trajectory (Step 1)  this doesnt move, moves now
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);  

    



%% 4) CLose or open gripper
% Pause for safety
%pause(10);
openService.call();

%pause(10);
%Close claw command
%closeService.call();

%%  2) - Moving into plate
% Save the new moved position as current state 
% WARNING: If not done, the UR3 bot will snap to origin (last saved point)
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)';
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

movement_adjustment = [-153.96*pi/180, -166.34*pi/180, -34.35*pi/180, -113.82*pi/180, 1.69*pi/180, 42.87*pi/180]; 
nextJointState_123456 = movement_adjustment;
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];

%% Run compiled trajectory (Step 2)     this moves down left
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);  

%% 4) CLose or open gripper
% Pause for safety
%pause(10);
closeService.call();

%pause(10);
%Close claw command
%closeService.call();

%% 5) Move Above Pick Up Point 
% Reinstate the current position
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; 
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

% Create ros message
startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     
%

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

movement_adjustment = [-154.44*pi/180, -158.42*pi/180, -33.96*pi/180, -116.97*pi/180, 2.16*pi/180, 39.89*pi/180];
nextJointState_123456 = movement_adjustment;
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];

%% Run compiled trajectory (Step 2)
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);

%% 5) Move To Tap
% Reinstate the current position
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; 
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

% Create ros message
startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     
%

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

movement_adjustment = [-161.33*pi/180, -125.81*pi/180, -87.86*pi/180, -144.8*pi/180, 18.32*pi/180, 88.17*pi/180];
nextJointState_123456 = movement_adjustment;
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];

%% Run compiled trajectory (Step 2)
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);

%% 6) Rotate under tap
% Reinstate the current position
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; 
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

% Create ros message
startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     
%

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

movement_adjustment = [-161.33*pi/180, -125.81*pi/180, -87.86*pi/180, -144.8*pi/180, 18.32*pi/180, 264.90*pi/180];
nextJointState_123456 = movement_adjustment;
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];


%% Run compiled trajectory (Step 2)
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);

%% 7) Move to UR3e
% Reinstate the current position
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; 
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

% Create ros message
startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     
%

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

movement_adjustment = [-190.11*pi/180, -123.31*pi/180, -91.98*pi/180, -140.14*pi/180, 28.51*pi/180, 266.14*pi/180];
nextJointState_123456 = movement_adjustment;
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];


%% Run compiled trajectory (Step 2)
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);

%% 8) Rotate under UR3e
% Reinstate the current position
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; 
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

% Create ros message
startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     
%

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

movement_adjustment = [-190.11*pi/180, -123.31*pi/180, -91.98*pi/180, -140.14*pi/180, 28.51*pi/180, 86.23*pi/180];
nextJointState_123456 = movement_adjustment;
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];


%% Run compiled trajectory (Step 2)
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);

%% 5) Move To Tap
% Reinstate the current position
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; 
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

% Create ros message
startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     
%

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

movement_adjustment = [-161.33*pi/180, -125.81*pi/180, -87.86*pi/180, -144.8*pi/180, 18.32*pi/180, 88.17*pi/180];
nextJointState_123456 = movement_adjustment;
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];

%% Run compiled trajectory (Step 2)
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);

%% 6) Rotate under tap
% Reinstate the current position
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; 
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

% Create ros message
startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     
%

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

movement_adjustment = [-161.33*pi/180, -125.81*pi/180, -87.86*pi/180, -144.8*pi/180, 18.32*pi/180, 264.90*pi/180];
nextJointState_123456 = movement_adjustment;
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];


%% Run compiled trajectory (Step 2)
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);

%% 7) Move to UR3e
% Reinstate the current position
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; 
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

% Create ros message
startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     
%

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

movement_adjustment = [-190.11*pi/180, -123.31*pi/180, -91.98*pi/180, -140.14*pi/180, 28.51*pi/180, 266.14*pi/180];
nextJointState_123456 = movement_adjustment;
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];


%% Run compiled trajectory (Step 2)
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);

%% 8) Rotate under UR3e
% Reinstate the current position
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; 
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

% Create ros message
startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     
%

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

movement_adjustment = [-190.11*pi/180, -123.31*pi/180, -91.98*pi/180, -140.14*pi/180, 28.51*pi/180, 86.23*pi/180];
nextJointState_123456 = movement_adjustment;
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];


%% Run compiled trajectory (Step 2)
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);

%% 8) Move above dropoff
% Reinstate the current position
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; 
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

% Create ros message
startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     
%

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

movement_adjustment = [-312*pi/180, -139.95*pi/180, -69.07*pi/180, -149.55*pi/180, -51.77*pi/180, 91.58*pi/180];
nextJointState_123456 = movement_adjustment;
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];


%% Run compiled trajectory (Step 2)
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);

%% 8) Move down to drop
% Reinstate the current position
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; 
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

% Create ros message
startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     
%

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

movement_adjustment = [-311.27*pi/180, -146.26*pi/180, -67.5*pi/180, -147.47*pi/180, -50.24*pi/180, 89.83*pi/180];
nextJointState_123456 = movement_adjustment;
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];


%% Run compiled trajectory (Step 2)
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);

%% 4) CLose or open gripper
% Pause for safety
%pause(10);
openService.call();

%pause(10);
%Close claw command
%closeService.call();

%% 8) Move out of drop
% Reinstate the current position
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; 
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

% Create ros message
startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     
%

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

movement_adjustment = [-310.92*pi/180, -138.48*pi/180, -88.28*pi/180, -132.3*pi/180, -55.70*pi/180, 89.74*pi/180];
nextJointState_123456 = movement_adjustment;
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];


%% Run compiled trajectory (Step 2)
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);

%% 4) CLose or open gripper
% Pause for safety
%pause(10);
closeService.call();

%pause(10);
%Close claw command
%closeService.call();

%% Put Into Initial Position

currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)';
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

[client, goal] = rosactionclient('/ur/scaled_pos_joint_traj_controller/follow_joint_trajectory');
goal.Trajectory.JointNames = jointNames;
goal.Trajectory.Header.Seq = 1;
goal.Trajectory.Header.Stamp = rostime('Now','system');
goal.GoalTimeTolerance = rosduration(0.05);
bufferSeconds = 1; % This allows for the time taken to send the message. If the network is fast, this could be reduced.
durationSeconds = 4; % This is how many seconds the movement will take (WARNING: Allow enough time for safety)
% Create ros message
startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);


nextJointState_123456 = [-pi -pi/2 0 -pi/2 0 0];
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];


%%  FINAL STEP
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);













%% 9) rotate up to clean
% Reinstate the current position
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; 
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

% Create ros message
startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     
%

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

movement_adjustment = [pi/2, 0, pi/2, 0, -pi/2, 0];
nextJointState_123456 = movement_adjustment;
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];


%% Run compiled trajectory (Step 2)
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);

%% 9) Rotated up go to rinse
% Reinstate the current position
currentJointState_321456 = (jointStateSubscriber.LatestMessage.Position)'; 
currentJointState_123456 = [currentJointState_321456(3:-1:1),currentJointState_321456(4:6)];

% Create ros message
startJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');
startJointSend.Positions = currentJointState_123456;
startJointSend.TimeFromStart = rosduration(0);     
%

endJointSend = rosmessage('trajectory_msgs/JointTrajectoryPoint');

movement_adjustment = [pi/2, 0, pi/2, 0, -pi/2, 0];
nextJointState_123456 = movement_adjustment;
endJointSend.Positions = nextJointState_123456;
endJointSend.TimeFromStart = rosduration(durationSeconds);

goal.Trajectory.Points = [startJointSend; endJointSend];


%% Run compiled trajectory (Step 2)
goal.Trajectory.Header.Stamp = jointStateSubscriber.LatestMessage.Header.Stamp + rosduration(bufferSeconds);
sendGoal(client,goal);


%% RESET FUNCTIONS
% If not posting properly, use ... then re-initialise
rosshutdown;
%%
% 
% [-27*pi/180, -103*pi/180, -138*pi/180, -118*pi/180, -81 90*pi/180]
% 
% 
% [-27*pi/180, -119*pi/180, -114*pi/180, -126*pi/180, -81*pi/180, 90*pi/180]
% 
% 
% 
% [-88*pi/180, -79*pi/180, 121*pi/180, -158*pi/180, -90*pi/180, 90*pi/180]
% 
% 
% [-88*pi/180, -79*pi/180, 121*pi/180, -158*pi/180, -90*pi/180, 270*pi/180]
% 
% 
% [-122*pi/180, -99*pi/180, -99*pi/180, -165*pi/180, -92*pi/180, 270*pi/180]
% 
% 
% [-122*pi/180, -99*pi/180, -99*pi/180, -165*pi/180, -92*pi/180, 90*pi/180]