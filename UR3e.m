classdef UR3e < RobotBaseClass
    %% UR3e Universal Robot 3kg payload robot model
    %
    % WARNING: This model has been created by UTS students in the subject
    % 41013. No guarantee is made about the accuracy or correctness of the
    % of the DH parameters of the accompanying ply files. Do not assume
    % that this matches the real robot!

    properties(Access = public)   
        plyFileNameStem = 'UR3e';
        qEndThree;
        environment; % Property to store the environment instance
    end
    
    methods
%% Constructor
        function self = UR3e(baseTr,useTool,toolFilename)
            if nargin < 3
                if nargin == 2
                    error('If you set useTool you must pass in the toolFilename as well');
                elseif nargin == 0 % Nothing passed
                    baseTr = transl(-0.5,1.1,1.3); 
                end             
            else % All passed in 
                self.useTool = useTool;
                toolTrData = load([toolFilename,'.mat']);
                self.toolTr = toolTrData.tool;
                self.toolFilename = [toolFilename,'.ply'];
            end
          
            self.CreateModel();
            
            % Initialize the environment
            % self.environment = EnvironmentSetup();
            % environmentPoints = self.environment.GetEnvironmentVertices(); % Get vertices of environment objects
            
            % Define ellipsoids for each link for collision detection
            % centerPoints = [
            %     0, 0, 0;  % Link 1
            %     0, 0, 0;  % Link 2
            %     0.12, 0, 0.12;  % Link 3
            %     0.12, 0, 0;  % Link 4
            %     0, 0, 0;  % Link 5
            %     0, 0, 0;  % Link 6
            %     0, 0, 0   % Link 7
            %     ];
            % 
            % radiiValues = [
            %     0.08, 0.08, 0.15;  % Link 1
            %     0.07, 0.07, 0.08;  % Link 2
            %     0.19, 0.09, 0.09;  % Link 3
            %     0.18, 0.08, 0.08;  % Link 4
            %     0.05, 0.05, 0.05;  % Link 5
            %     0.06, 0.06, 0.07;  % Link 6
            %     0.05, 0.05, 0.04   % Link 7
            %     ];
            % 
            % % Generate ellipsoids for collision detection
            % for i = 1:7
            %     centerPoint = centerPoints(i, :);
            %     radii = radiiValues(i, :);
            %     [X, Y, Z] = ellipsoid(centerPoint(1), centerPoint(2), centerPoint(3), radii(1), radii(2), radii(3));
            %     self.model.points{i} = [X(:), Y(:), Z(:)];
            % 
            %     warning off
            %     self.model.faces{i} = delaunay(self.model.points{i});
            %     warning on
            % end
            % 
            % self.model.plot3d([0,0,0,0,0,0]);


            axis([-0.1 0.1 -0.1 0.1 -0.1 0.1]); % Adjust based on your ellipsoid dimensions
            camlight
            hold on

            % % Define transformations
            % q = zeros(1, self.model.n);
            % tr = zeros(4,4,self.model.n+1);
            % tr(:,:,1) = self.model.base;
            % L = self.model.links;
            % for i = 1 : self.model.n
            %     tr(:,:,i+1) = tr(:,:,i) * trotz(q(i)) * transl(0,0,L(i).d) * transl(L(i).a,0,0) * trotx(L(i).alpha);
            % end
            % 
            % % Collision detection with environment vertices
            % for i = 1: size(tr,3)
            %     pointsAndOnes = [inv(tr(:,:,i)) * [environmentPoints, ones(size(environmentPoints, 1), 1)]']';
            %     updatedPoints = pointsAndOnes(:,1:3);
            % 
            %     % Calculate algebraic distance
            %     algebraicDist = ((updatedPoints(:,1) - centerPoint(1)) / radii(1)).^2 ...
            %                   + ((updatedPoints(:,2) - centerPoint(2)) / radii(2)).^2 ...
            %                   + ((updatedPoints(:,3) - centerPoint(3)) / radii(3)).^2;
            % 
            %     pointsInside = find(algebraicDist < 1);
            %     disp(['Collision Check: ', num2str(size(pointsInside,1)),' points inside the ',num2str(i),'th ellipsoid']);
            % end
            
			self.model.base = self.model.base.T * baseTr;
            self.model.tool = self.toolTr;
			warning('The DH parameters are correct. But as of July 2023 the ply files for this UR3e model are definitely incorrect, since we are using the UR3 ply files renamed as UR3e. Once replaced remove this warning.');  
            self.PlotAndColourRobot();
            
            drawnow
        end

%% CreateModel
        function CreateModel(self)
            link(1) = Link('d',0.15185,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]), 'offset',0);
            link(2) = Link('d',0,'a',-0.24355,'alpha',0,'qlim', deg2rad([-360 360]), 'offset',0);
            link(3) = Link('d',0,'a',-0.2132,'alpha',0,'qlim',deg2rad([-360 360]), 'offset', 0);
            link(4) = Link('d',0.13105,'a',0,'alpha',pi/2,'qlim',deg2rad([-360 360]),'offset', 0);
            link(5) = Link('d',0.08535,'a',0,'alpha',-pi/2,'qlim',deg2rad([-360,360]), 'offset',0);
            link(6) = Link('d',0.0921,'a',0,'alpha',0,'qlim',deg2rad([-360,360]), 'offset', 0);
             
            self.model = SerialLink(link,'name',self.name);
        end
        
    end
end
