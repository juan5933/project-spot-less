classdef Gripper < handle
    properties
        p1         % Left side of the gripper (SerialLink)
        p2         % Right side of the gripper (SerialLink)
        qclose1    % Joint configuration for closed state (p1)
        qclose2    % Joint configuration for closed state (p2)
        qopen1     % Joint configuration for open state (p1)
        qopen2     % Joint configuration for open state (p2)
        steps      % Number of steps for animation
        linkLength % Length of the gripper's links
        robot      % Placeholder for the robot model (not used here)
    end

    methods
        function self = Gripper()
            % Constructor for the Gripper class
            % Initializes both sides of the gripper with link lengths and configurations

            self.linkLength = 0.05;  % Set link length for gripper arms

            % Create the left side of the gripper (p1) as a 2-link robot
            self.p1 = SerialLink([ ...
                Revolute('d', 0, 'a', self.linkLength, 'alpha', 0, 'standard'), ...
                Revolute('d', 0, 'a', self.linkLength, 'alpha', 0, 'standard')], ...
                'name', 'GripperLeft');  % SerialLink model for the left side

            % Create the right side of the gripper (p2) as a 2-link robot
            self.p2 = SerialLink([ ...
                Revolute('d', 0, 'a', self.linkLength, 'alpha', 0, 'standard'), ...
                Revolute('d', 0, 'a', self.linkLength, 'alpha', 0, 'standard')], ...
                'name', 'GripperRight');  % SerialLink model for the right side

            % Define joint configurations for open and closed states
            self.qclose1 = [45 -45] * pi/180;  % Closed position for p1
            self.qclose2 = [-45 45] * pi/180;  % Closed position for p2
            self.qopen1 = [0 0];               % Open position for p1
            self.qopen2 = [0 0];               % Open position for p2

            % Define the number of steps for smooth animation
            self.steps = 50;  % Animation steps for opening/closing

            % Optionally, offset the second side of the gripper by a small distance (not used)
            % self.p2.base = transl(0, 0.05, 0);  % Optional: Offset p2 5cm to the right
        end

        function animateGripper(self, Tr)
            % Method to animate the gripper following the robot's end-effector
            % Tr is the transformation matrix of the robot's end-effector

            % Set the base of p1 and p2 to follow the end-effector's transformation
            self.p1.base = Tr * transl(0, -0.02, 0);  % Position the left side of the gripper
            self.p2.base = Tr * transl(0, 0.02, 0);   % Offset the right side by 2cm

            % Animate both sides of the gripper at the current configuration (closed state)
            self.p1.plot(self.qclose1, 'noarrow');  % Plot the left side without arrows
            self.p2.plot(self.qclose2, 'noarrow');  % Plot the right side without arrows
        end

        function openGripper(self)
            % Method to animate the gripper opening

            % Create a time scaling vector for smooth motion using LSPB (Linear Segment with Parabolic Blend)
            s = lspb(0, 1, self.steps);  % Interpolation steps between 0 and 1

            for i = 1:self.steps
                % Interpolate joint angles from closed to open configuration
                q1 = (1 - s(i)) * self.qclose1 + s(i) * self.qopen1;  % Left side interpolation
                q2 = (1 - s(i)) * self.qclose2 + s(i) * self.qopen2;  % Right side interpolation

                % Animate both sides of the gripper opening
                self.p1.animate(q1);  % Animate the left side (p1)
                self.p2.animate(q2);  % Animate the right side (p2)
                drawnow;              % Update the figure
                pause(0.01);          % Small pause to control the animation speed
            end
        end

        function closeGripper(self)
            % Method to animate the gripper closing

            % Create a time scaling vector for smooth motion using LSPB
            s = lspb(0, 1, self.steps);  % Interpolation steps between 0 and 1

            for i = 1:self.steps
                % Interpolate joint angles from open to closed configuration
                q1 = (1 - s(i)) * self.qopen1 + s(i) * self.qclose1;  % Left side interpolation
                q2 = (1 - s(i)) * self.qopen2 + s(i) * self.qclose2;  % Right side interpolation

                % Animate both sides of the gripper closing
                self.p1.animate(q1);  % Animate the left side (p1)
                self.p2.animate(q2);  % Animate the right side (p2)
                drawnow;              % Update the figure
                pause(0.01);          % Small pause to control the animation speed
            end
        end
    end
end
