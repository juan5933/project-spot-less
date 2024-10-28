classdef Plates < handle
    % Class to manage the placement and attributes of bricks in the environment

    properties
        platesStart  % Coordinates for the starting positions of the bricks
        platesEnd    % Coordinates for the destination (end) positions of the bricks
        plates_h     % Handles to the graphical brick objects in the environment
        platesGrab   % Grabbing positions of the plate
        plateVertices
        transformedVertices
    end

    methods
        function self = Plates()
            % Constructor: Initialize the brick starting and ending positions
            % brickStart: 3D coordinates for where the bricks are initially placed
            % brickEnd: 3D coordinates for where the bricks will be moved to\

            self.platesGrab = [-1.45, 0.25,  1.4; 
                                -1.451, 0.2, 1.375;
                                -1.451, 0.2, 1.35;
                                ];  % Plate Grabbing positions


            self.platesStart = [-1.451, 0.35  1.4; 
                                -1.451, 0.35, 1.375;
                                -1.451, 0.35, 1.35;
                                ];  % Plate positions

            self.platesEnd = [  0.05, 0.35, 1.355;
                                0.05, 0.35, 1.375;
                                0.05, 0.35, 1.4;
                                     ];  % Corresponding end positions for each brick

            % Preallocate the cell array for storing the handles to the bricks in the environment
            self.plates_h = cell(1, 3);  % Array to store brick handles (9 bricks)
            self.plateVertices = cell(1,3);

        end

        function PlacePlates(self)
            % Method to place the bricks at their start positions in the environment

            for i = 1:size(self.platesStart, 1)
                % Place each brick using the 'PlaceObject' function at its start position
                self.plates_h{i} = PlaceObject('yellowplatever2.ply');

                self.plateVertices{i} = get(self.plates_h{i}, 'Vertices');

                self.transformedVertices = [self.plateVertices{i}, ones(size(self.plateVertices{i},1),1)] * transl(self.platesStart(i,:))';
                
                
                % Update the brick's position using the transformed vertices
                set(self.plates_h{i}, 'Vertices', self.transformedVertices(:,1:3));

                hold on;  % Retain the previous objects in the scene while adding new ones
            end
        end


    end
end