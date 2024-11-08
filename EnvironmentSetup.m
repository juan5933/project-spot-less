classdef EnvironmentSetup
    % Class to set up a 3D environment and add objects for visualization

    methods
        function self = EnvironmentSetup()
            % Constructor: Initialize and configure the environment
            self.SetupWorkspace();  % Setup the 3D workspace settings
            self.AddJPG();          % Add a background texture (e.g., concrete)
            self.ObjectLayout();    % Arrange predefined PLY objects in the scene
        end

        function SetupWorkspace(self)
            % Configure the 3D workspace view and axis limits
            axis([-6 6 -3 3 -0.5 2]);  % Set the boundaries for the 3D plot
            axis manual;              % Fix the axis limits to avoid auto-scaling
            view(3);                  % Set the view to 3D perspective
            axis equal;               % Ensure uniform scaling for all axes
            hold on;                  % Retain current plot when adding objects
        end

        function AddJPG(self)
            % Render a texture as the environment background
            surf([-2.4, -2.4; 3, 3], [-2.7, 4; -2.7, 4], [0, 0; 0, 0], ...
                'CData', imread('concrete.jpg'), 'FaceColor', 'texturemap');  % Apply texture
            hold on;  % Maintain current plot
        end
    
        function ObjectLayout(self)
            % Add multiple objects (e.g., buttons, models) to the environment
            self.AddPly('emergencyStopButton.ply', [2, 2, -0.45], trotx(-pi/2), 0.5);
            self.AddPly('emergencyStopButton.ply', [2, -2, 3.9], trotx(pi/2), 0.5);
            self.AddPly('personMaleCasual.ply', [-1, -3, 0], trotz(pi), 1.2);
            self.AddPly('fireExtinguisher.ply', [2.1, -1, 0], trotz(pi), 1);
            self.AddPly('kitchenenvironment.ply', [0, 0, 0], trotz(pi) * 1.45, 200);
            self.AddPly('PlateStand.ply', [1.451, -0.31, 1.31], trotz(pi), 1);
            self.AddPly('PlateStand.ply', [-0.15, -0.35, 1.31], trotz(pi), 1);
            self.AddPly('slipMat.ply', [0.2, 0.22, 0], trotz(-pi), 4);
            self.AddPly('slipMat.ply', [-0.08, 0.22, 0], trotz(-pi), 4);
        end
        
        function AddPly(self, fileName, position, rotationFactor, scaleFactor)
            % Load and position a PLY object with rotation and scaling
            ply_h = PlaceObject(fileName, position);  % Insert the PLY object at the specified position
            verts = [get(ply_h, 'Vertices'), ones(size(get(ply_h, 'Vertices'), 1), 1)] * rotationFactor;  % Apply rotation
            verts(:, 1:3) = verts(:, 1:3) * scaleFactor;  % Scale the vertices
            set(ply_h, 'Vertices', verts(:, 1:3));  % Update the object vertices after transformation
            hold on;  % Keep the plot active for further additions
        end
    end
end
