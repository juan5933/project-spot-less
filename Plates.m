classdef Plates < handle
    % Class to manage the placement and attributes of bricks in the environment
    
    properties
        platesStart  % Coordinates for the starting positions of the bricks
        platesEnd    % Coordinates for the destination (end) positions of the bricks
        plates_h     % Handles to the graphical brick objects in the environment
    end
    
    methods
        function self = Plates()
            % Constructor: Initialize the brick starting and ending positions
            % brickStart: 3D coordinates for where the bricks are initially placed
            % brickEnd: 3D coordinates for where the bricks will be moved to
            
            self.platesStart = [-1.55, 0.6  1.35; 
                                -1.55, 0.6, 1.325;  
                                -1.55, 0.6, 1.3;                
                                ];  % Plate positions
            
            self.platesEnd = [  0, 1, 1.25;
                                0, 1, 1.30;
                               -0, 1, 1.32;  
                                     ];  % Corresponding end positions for each brick
            
            % Preallocate the cell array for storing the handles to the bricks in the environment
            self.plates_h = cell(1, 9);  % Array to store brick handles (9 bricks)
        end
        
        function PlacePlates(self)
            % Method to place the bricks at their start positions in the environment

            for i = 1:size(self.platesStart, 1)
                % Place each brick using the 'PlaceObject' function at its start position
                self.plates_h{i} = PlaceObject('yellowPlate.ply', self.platesStart(i,:));
                hold on;  % Retain the previous objects in the scene while adding new ones
            end
        end

    
    end
end
