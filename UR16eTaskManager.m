%% Create an instance of the RealRobot class
robot = RealRobot();

% Run individual methods step by step
%%
robot.moveToInitialPosition();  % Move to initial position
%%
robot.moveToPickUpPosition();   % Move to pick-up position
%%
robot.openGripperClose();            % Open the gripper
%%
robot.moveToAbovePickUp();      % Move to position above the pick-up point
%%
robot.moveToUnderTap();         % Move the robot under the tap for rinsing
%%
robot.rinsePositionDown();      % Perform a rinse motion
%%
robot.moveToCleaningPosition(); % Move to the virtual cleaning position
%%
robot.rotateUpForCleaning();    % Rotate up for cleaning
%%
robot.finalRinseUp();           % Perform a final rinse (up position)
%%
robot.finalRinseDown();         % Perform a final rinse (down position)
%%
robot.scanningPlatePos();
%%
% robot.dirtyPlate();
%%
robot.placePlateDown();
%%
robot.openGripperOpen();
%%
robot.returnHome();

% Task sequence completed
disp('Task sequence completed.');
%%
% Cleanly shut down the ROS connection after the task is complete
robot.shutdown();
