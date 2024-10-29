
classdef UR16e < RobotBaseClass
    %% UR16e Universal Robot 3kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarentee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'UR16e';
    end
    
    methods
%% Constructor
        function self = UR16e(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(-0.55,0.35,1.3)* rpy2tr(0, 0, 180, 'deg');  
                end             
            else % All passed in 
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end
          
            self.CreateModel();

            % % Create ellipsoid for collision detection
            % 
            % % Define the center points and radii for each link
            % % Example values, customize as needed for each link
            % centerPoints = [
            %     0, 0, 0;  % Link 1
            %     0, 0, 0;  % Link 2
            %     0.24, 0, 0.16;  % Link 3
            %     0.2, 0, 0.03;  % Link 4
            %     0, 0, 0;  % Link 5
            %     0, 0, 0;  % Link 6
            %     0, 0, 0   % Link 7
            %     ];
            % 
            % radiiValues = [
            %     0.12, 0.12, 0.12;  % Link 1
            %     0.13, 0.10, 0.13; % Link 2
            %     0.38, 0.13, 0.13;  % Link 3
            %     0.3, 0.12, 0.12; % Link 4
            % 
            %     0.1, 0.1, 0.1;  % Link 5
            %     0.08, 0.08, 0.08;   % Link 6
            %     0.08, 0.08, 0.05   % Link 7
            %     ];
            % 
            % % Generate an ellipsoid for each link with specified parameters
            % for i = 1:7
            %     centerPoint = centerPoints(i, :);
            %     radii = radiiValues(i, :);
            % 
            %     % Generate ellipsoid with custom parameters for this link
            %     [X, Y, Z] = ellipsoid(centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3));
            % 
            %     % Store points and faces for the model
            %     self.model.points{i} = [X(:), Y(:), Z(:)];
            % 
            %     % Suppress warnings for Delaunay triangulation and create faces
            %     warning off
            %     self.model.faces{i} = delaunay(self.model.points{i});
            %     warning on
            % end
            % 
            % self.model.plot3d([0,0,0,0,0,0]);




            axis([-0.1 0.1 -0.1 0.1 -0.1 0.1]); % Adjust based on your ellipsoid dimensions
            camlight
            hold on


			self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
			warning('The DH parameters are correct. But as of July 2023 the ply files for this UR3e model are definitely incorrect, since we are using the UR3 ply files renamed as UR3e. Once replaced remove this warning.')  
            self.PlotAndColourRobot();

            drawnow
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0.1807,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(2) = Link('d',0,'a',	-0.4784,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            link(3) = Link('d',0,'a',	-0.36,'alpha',0,'qlim', deg2rad([-360 360]), 'offset', 0);
            link(4) = Link('d',	0.17415,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            link(5) = Link('d',	0.11985,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            link(6) = Link('d',	0.11655,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
             
            self.model = SerialLink(link,'name',self.name);
        end      
    end
end
