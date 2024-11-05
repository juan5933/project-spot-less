classdef SpotlessMain < handle
    % Main class to handle the execution of the robot's tasks, environment setup,
    % and point cloud simulation. It contains methods to run the simulation and 
    % generate the workspace point cloud.

    methods (Static)
        function Run()
            % Run the main simulation for the robot task.
            % This method sets up the environment, bricks, gripper, and robot,
            % then executes the robot's trajectory while profiling the execution.
            
            % Clear the current figure and reset profiling data
            clf;  % Clear the current figure window
            
            % Set up the environment and bricks
            EnvironmentSetup();  % Create and display the environment


            plates = Plates();  % Initialize the Plates object
            plates.PlacePlates();  % Place the plates in the environment
            gripper = Gripper();
            app = Guiapp1();

            % Set up the robot

            robotSixteen = UR16e();  % Initialize the robot (Linear UR3e model)
            robotThree = UR3e();

            % Set up and run the robot trajectory, passing the robot, bricks, and gripper
            % trajectory = UR16eTrajectory(robotSixteen, plates, gripper);
            trajectory = SpotlessTrajectory(robotThree, robotSixteen, plates, app);  % Initialize RobotTrajectory
            % trajectory = SpotlessTrajectoryRMRC(robotThree, robotSixteen, plates, gripper, app);  % Initialize RobotTrajectory

            trajectory.Run();  % Execute the robot's movement trajectory
            
        end


    end
end
