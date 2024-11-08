classdef Plates < handle
    % Class to manage the placement and attributes of plates in the environment

    properties
        platesStart  % Coordinates for initial positions of the plates
        platesEnd    % Coordinates for destination positions of the plates
        plates_h     % Handles to graphical plate objects in the environment
        platesGrab   % Coordinates for plate grabbing positions
        plateVertices  % Original vertices of the plates
        transformedVertices  % Vertices after transformation for positioning
    end

    methods
        function self = Plates()
            % Constructor: Initialize the starting and ending positions of the plates
            % and the positions used for grabbing the plates.

            % Define plate grabbing positions in 3D space
            self.platesGrab = [-1.45, 0.25, 1.4; 
                               -1.451, 0.2, 1.375;
                               -1.451, 0.2, 1.35];

            % Define the initial positions for each plate in 3D space
            self.platesStart = [-1.451, 0.31, 1.42; 
                                -1.451, 0.31, 1.381;
                                -1.451, 0.31, 1.356];

            % Define the end positions for each plate in 3D space
            self.platesEnd = [0.05, 0.35, 1.355;
                              0.05, 0.35, 1.375;
                              0.05, 0.35, 1.4];

            % Preallocate space for storing handles to the plate objects
            self.plates_h = cell(1, 3);  % Array for graphical object handles
            self.plateVertices = cell(1, 3);  % Array for storing original plate vertices
        end

        function PlacePlates(self)
            % Place each plate in the environment at its starting position

            for i = 1:size(self.platesStart, 1)
                % Load and place the plate object in the environment
                self.plates_h{i} = PlaceObject('yellowplatever2.ply');

                % Retrieve the original vertices of the plate
                self.plateVertices{i} = get(self.plates_h{i}, 'Vertices');

                % Apply translation to position the plate at its starting coordinates
                self.transformedVertices = [self.plateVertices{i}, ones(size(self.plateVertices{i}, 1), 1)] * transl(self.platesStart(i, :))';

                % Update the plate's position with the transformed vertices
                set(self.plates_h{i}, 'Vertices', self.transformedVertices(:, 1:3));

                % Retain the current scene while adding more plates
                hold on;
            end
        end
    end
end
