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
            
            self.platesStart = [-1, 0.5, 0;
                               -0.9, 0.5, 0;
                               -0.8, 0.5, 0;
                               -0.7, 0.5, 0;
                               -0.6, 0.5, 0;
                               -0.5, 0.5, 0;
                               -0.4, 0.5, 0;
                               -0.3, 0.5, 0;
                               -0.2, 0.5, 0];  % Starting positions for each brick (9 in total)
            
            self.platesEnd = [-1.25, 0.25, 0; 
                             -1.25, 0, 0; 
                             -1.25, 0.25, 0.1; 
                             -1.25, 0, 0.1;
                             -1.25, 0.25, 0.2; 
                             -1.25, 0, 0.2;
                             -1.25, 0.25, 0.3;
                             -1.25, 0, 0.3; 
                             -1.25, 0.25, 0.4];  % Corresponding end positions for each brick
            
            % Preallocate the cell array for storing the handles to the bricks in the environment
            self.plates_h = cell(1, 9);  % Array to store brick handles (9 bricks)
        end
        
        function PlacePlates(self)
            % Method to place the bricks at their start positions in the environment
            % Iterates through all bricks and places them in the scene
            
            for i = 1:size(self.platesStart, 1)
                % Place each brick using the 'PlaceObject' function at its start position
                self.plates_h{i} = PlaceObject('yellowPlate.ply', self.platesStart(i,:));
                hold on;  % Retain the previous objects in the scene while adding new ones
            end
        end
    end
end
