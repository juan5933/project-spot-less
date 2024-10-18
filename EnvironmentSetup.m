classdef EnvironmentSetup

    methods
        function self = EnvironmentSetup()
            % Constructor: setup the environment and add objects
            self.SetupWorkspace();
            self.AddJPG();
            self.ObjectLayout();  % Call ObjectLayout to add all PLY objects
        end

        function SetupWorkspace(self)
            % Setup the workspace with defined view and axis settings
            clf;
            axis([-6 6 -3 3 -0.5 2]);  % Set axis limits for the workspace
            axis manual;               % Prevents automatic axis resizing
            view(3);                   % Set a 3D view
            axis equal;                % Ensure equal scaling on all axes
            hold on;                   % Retain the current plot when adding new objects
        end

        function AddJPG(self)
            % Add concrete texture to the environment
            surf([-3,-3;3,3], [-2.7,4;-2.7,4], [0,0;0,0], ...
                'CData', imread('concrete.jpg'), 'FaceColor', 'texturemap');
            hold on;
        end
    
        function ObjectLayout(self)
            % Layout all the objects (fences, tables, etc.) in the environment
            self.AddPly('emergencyStopButton.ply', [2, 2, -0.45], trotx(-pi/2), 0.5);
            self.AddPly('emergencyStopButton.ply', [2.5, 2, -0.45], trotx(-pi/2), 0.5);
            self.AddPly('personMaleCasual.ply', [-1,-3,0], trotz(pi), 1.2);
            self.AddPly('fireExtinguisher.ply', [2.1, -1, 0], trotz(pi), 1);
            self.AddPly('kitchenenvironment.ply', [0,0,0], trotz(pi)*1.45, 200);
            self.AddPly('PlateStand.ply', [1.55,-0.6, 1.31], trotz(pi), 1);
            self.AddPly('PlateStand.ply', [-0.35, -0.6, 1.31], trotz(pi), 1);
        end
        
        function AddPly(self, fileName, position, rotationFactor, scaleFactor)
            % Add PLY object to the environment
            ply_h = PlaceObject(fileName, position);  % Place the object in the scene
            verts = [get(ply_h, 'Vertices'), ones(size(get(ply_h, 'Vertices'), 1), 1)] * rotationFactor;  % Rotate the object
            verts(:, 1:3) = verts(:, 1:3) * scaleFactor;  % Scale the object
            set(ply_h, 'Vertices', verts(:, 1:3));  % Update the vertices of the object
            hold on;
        end
    end
end
