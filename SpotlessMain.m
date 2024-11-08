classdef SpotlessMain < handle
    % Main class to manage the execution of robot tasks, environment setup,
    % and point cloud simulation for the workspace.

    methods (Static)
        function Run()
            % Run the main simulation sequence for the robot task.
            % This method initializes the environment, places plates, sets up the
            % robot and gripper, and executes the robot's trajectory while profiling.

            clf;  % Clear the current figure to reset the display

            % Initialize and set up the environment with objects
            EnvironmentSetup();  % Create and configure the 3D environment

            % Initialize plates and place them at their starting positions
            plates = Plates();  % Create an instance of Plates
            plates.PlacePlates();  % Place the plates in the environment

            % Initialize the gripper and application interface
            gripper = Gripper();
            app = Guiapp1();  % Initialize GUI application

            % Create and configure the robots
            robotSixteen = UR16e();  % Initialize UR16e robot model
            robotThree = UR3e();  % Initialize UR3e robot model

            % Initialize and run the robot trajectory for the task
            trajectory = SpotlessTrajectory(robotThree, robotSixteen, plates, app);  % Create trajectory instance
            trajectory.Run();  % Execute the trajectory to move the robots
        end
    end
end
