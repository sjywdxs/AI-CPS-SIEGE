classdef ParkingLotEnvironment < handle
% MAP = ParkingLotEnvironment() creates a parking lot containing randomly chosen free
% spots.
%
% MAP = ParkingLotEnvironment(FREEIDXS) creates a parking lot with the free spots 
% specified in the array FREEIDXS.
% 
% MAP is a struct containing the following fields:
%   1. XLimits - map X limits
%   2. YLimits - map Y limits
%   3. SpotLocations = [x y] of occupied spaces
%   4. OccupiedSpots = binary array of size 1x64, 1 occupied, 0 free
%   5. TotalSpots - total number of parking spots
%   6. AxesHandle - axes handle of the figure
%
% Total number of parking spots in this parking lot is 46 (non-tunable).

% Copyright 2021 The MathWorks, Inc.

    properties
        VehiclePose         = [20 16 0]
        TrainingZoneXLimits = [-1.2 12.425]
        TrainingZoneYLimits = [-41.34 -29]
    end
    properties (SetAccess = private)
        XLimits             = [-9, 50]       % Smaller 3D Environment
        YLimits             = [-60,-12]      % Smaller 3D Environment
        
        SpotDimensions      = [2.725 5.56]    % Measured from 3D environ.
        ObstacleDimensions  = [3.864 1.653]   % Length/Width of hatchback adjusted to SpotDimensions
        
        NonVehicleObs = [-7.925 -22.125 2.15    13.25   0;
                         12.475 -22.04  38.65   2.22    0;
                         33.925 -22.125 4.25    13.25   0;
                         -6.55  -42.225 4.9     12.85   0;
                         14.35  -42.235 36.9    1.79    0;
                         34.515 -42.225 3.43    12.85   0;
                         13.4   -58.1   44.8    3.8     0]                    

        % Hatchback dimensions
        centerToFront = 1.104; % meters
        centerToRear  = 1.343; % meters
        frontOverhang = 0.828; % meters
        rearOverhang  = 0.589; % meters
        vehicleWidth  = 1.653; % meters
        vehicleHeight = 1.513; % meters
        vehicleLength = 3.864;
        VehicleDimensions = [3.864 1.653];
        PassageWidth        = 16    % Width of channel car can drive
        MaxSpots            = 46
        SpotLocations       = zeros(46,2)
        SensorLocations     = zeros(46,2)
        OccupiedSpots       = ones(1,46)
        ObstacleMatrix
    end
    properties (Hidden)
        Figure
        VehicleBodyHandle
        VehicleAxlesHandle
        VehicleWheelsHandle
        VehiclePoly
        LidarLinesHandle
        LidarCrossHandle
        CameraFOVHandle
        TrainZoneHandle
        VehicleStatusHandle
        ControllerStatusHandle
        TimeDisplayHandle
        Route
    end
    
    methods
        function this = ParkingLotEnvironment(freeidxs,args)
            arguments
                freeidxs (1,:) double {localValidateIdx} = randperm(46,10) 
                args.Position  double   = [435 200 640 480] %[435 200 950 740]
                args.Name      char     = 'Parking Lot'
                args.Route     double   = [40 -60 90]
            end
            
            this.Route = args.Route;
            this.OccupiedSpots(freeidxs) = 0;
            buildFigure(this,args.Position,args.Name);
            initializeMap(this);
            this.Figure.Visible = 'off';
            this.ObstacleMatrix = getObstacles(this, find(this.OccupiedSpots==1));
        end

        function show(this)
            % display the map in a figure.
            if isempty(this.Figure) || ~isvalid(this.Figure)
                buildFigure(this,[435 200 640 480],'Parking Lot');
                initializeMap(this);
            end
            this.Figure.Visible = 'on';
        end
        
        function buildFigure(this, position, name)
            % Build the figure
            this.Figure = figure('Position',position, ...
                'Name',name, ...
                'NumberTitle','off', ...
                'MenuBar','none');
            ax = gca(this.Figure);
            legend(ax,'off');
            hold(ax, 'on');
            ax.XLimMode = 'manual';
            ax.YLimMode = 'manual';
            ax.XLim = this.XLimits;
            ax.YLim = this.YLimits;
        end

        function highlight(this,idx)
            % highlight(map,idx) highlights a spot in the map specified in idx.
            arguments
                this
                idx (1,1) double
            end
            ax = gca(this.Figure);
            posx = this.SpotLocations(idx,1) - 0.5*this.ObstacleDimensions(2) - 0.5;
            posy = this.SpotLocations(idx,2) - 0.5*this.ObstacleDimensions(1) - 0.5;
            len = this.ObstacleDimensions(2) + 1;
            wid = this.ObstacleDimensions(1) + 1;
            
            hlight = findobj(ax,'Tag','hlight');
            if isempty(hlight)
                rectangle(ax,'Position',[posx posy len wid],'EdgeColor','g','LineWidth',2,'Tag','hlight')
            else
                hlight.Position = [posx posy len wid];
            end
            drawnow;
        end
        
        function setFree(this,idxs)
            % setFree(map,idxs) frees the spots with index IDXS.
            localValidateIdx(idxs)
            this.OccupiedSpots(idxs) = 0;
            this.ObstacleMatrix = getObstacles(this, find(this.OccupiedSpots==1));
            initializeMap(this);
        end

        function setOccupied(this,idxs)
            % setOccupied(map,idxs) occupies the spots with index IDXS.
            localValidateIdx(idxs)
            this.OccupiedSpots(idxs) = 1;
            this.ObstacleMatrix = getObstacles(this, find(this.OccupiedSpots==1));
            initializeMap(this);
        end

        function obs = getObstacles(this,spotidx)
            % obs = getObstacleList(map) returns the list of obstacles in the
            % parking lot in [center.x center.y xlen ylen theta] format.
            %
            % obs = getObstacleList(map,spotidx) returns the obstacle at the
            % spot index spotidx in [center.x center.y xlen ylen theta] format.
            arguments
                this
                spotidx (1,:) double = find(this.OccupiedSpots)
            end
            obs = zeros(numel(spotidx)+11,5);
            for ct = 1:numel(spotidx)
                obs(ct,1) = this.SpotLocations(spotidx(ct),1);
                obs(ct,2) = this.SpotLocations(spotidx(ct),2);
                obs(ct,3) = this.ObstacleDimensions(2);
                obs(ct,4) = this.ObstacleDimensions(1);
                obs(ct,5) = 0;
            end
            
            % add obstacles at map boundary
            obs(numel(spotidx)+1:numel(spotidx)+4,:) = ...
                [this.XLimits(1)+0.5*diff(this.XLimits)  this.YLimits(1)+0.25 diff(this.XLimits) 0.5 0; ...
                 this.XLimits(2)-0.25  this.YLimits(1)+0.5*diff(this.YLimits) 0.5 diff(this.YLimits) 0; ...
                 this.XLimits(1)+0.5*diff(this.XLimits)  this.YLimits(2)-0.25 diff(this.XLimits) 0.5 0; ...
                 this.XLimits(1)+0.25  this.YLimits(1)+0.5*diff(this.YLimits) 0.5 diff(this.YLimits) 0];
                 
            % add non-vehicle obstacles
            obs(numel(spotidx)+5:end,:) = this.NonVehicleObs;
        end

        function pose = createTargetPose(this,idx,direction)
            % pose = createTargetPose(map,idx,centerToRear,theta) returns a target
            % pose for the vehicle based on the specified parking spot index
            % idx and length from center of car to rear axel, centerToRear.
            arguments
                this
                idx (1,1) double {mustBePositive,mustBeInteger}
                direction (1,1) string 
            end
            
            loc = this.SpotLocations(idx,:);
            isrow1 = idx <= 14;
            isrow2 = idx >= 15 && idx <= 28;
            isrow3 = idx >= 29 && idx <= 37;
            isrow4 = idx >= 38 && idx <= 46;
            
            if isrow1 || isrow3     % Parked car faces south
                pose(1) = loc(1);
                pose(2) = loc(2) + this.centerToRear;
                pose(3) = -pi/2;
            elseif isrow2 || isrow4 % Parked car faces north
                pose(1) = loc(1);
                pose(2) = loc(2) - this.centerToRear;
                pose(3) = pi/2;
            end

            if direction == "north"
                pose(3) = pi/2;     % Parked car faces north
            elseif direction == "south"
                pose(3) = -pi/2;    % Parked car faces south
            end
        end

        function ax = getAxesHandle(this)
            % getAxesHandle(map) returns the axes handle to the figure.
            ax = gca(this.Figure);
        end

        function delete(this)
            % delete(map) deletes the figure.
            if ~isempty(this.Figure) || ~isvalid(this.Figure)
                delete(this.Figure);
            end
        end
    
        function initializeMap(this)
            % get handle and clear axis
            ax = gca(this.Figure);
            if ~isempty(this.Figure)
                cla(ax)
            end
            %this.Figure.Visible = 'on';
            axis(ax,'equal')
            axis(ax,'tight')
            
            % Number of spots in Rows 1-4
            numSpotsRow1 = 14;
            numSpotsRow2 = 14;
            numSpotsRow3 = 9;
            numSpotsRow4 = 9;

            xStartRow1 = -6.65;
            yStartRow1 = -20.93;            
            
            xStartRow2 = -6.65;
            yStartRow2 = -23.145;
            
            xStartRow3 = -6.65 + this.SpotDimensions(1);
            yStartRow3 = -41.34;
            
            xStartRow4 = -6.65 + this.SpotDimensions(1);
            yStartRow4 = -43.14;
          
            spotIdx = 1;
            txtoffset = 0.6*this.SpotDimensions(2);

            % Plot parking lot environment image
            img = imread('ParkingLotEnvironmentSubsection.jpg');
            image(ax,'CData',img,'XData',[-9, 50],'YData',[-12,-60])
            
            % Plot outer boundary
            rectangle(ax,'Position',[this.XLimits(1) this.YLimits(1) diff(this.XLimits) diff(this.YLimits)],'LineWidth',2)
            
            % Plot non-vehicle obstacle boundaries
            for i = 1:size(this.NonVehicleObs,1)
                rectangle(ax,'Position'...
                    ,[this.NonVehicleObs(i,1)-0.5*this.NonVehicleObs(i,3)...
                    this.NonVehicleObs(i,2)-0.5*this.NonVehicleObs(i,4)...
                    this.NonVehicleObs(i,3)... 
                    this.NonVehicleObs(i,4)],'EdgeColor',[0 0.45 0.25],'LineWidth',2.25)
            end
            
            % Plot reference path
            plot(ax,this.Route(:,1),this.Route(:,2),'-.m','LineWidth',1.5)

            %% Plot Rows 1-4
            % Plot Row 1
            for i = 1:numSpotsRow1+1
                % Plot parking lines
                x = (xStartRow1 + (i-1)*this.SpotDimensions(1))*[1 1];
                y = yStartRow1 + [0 this.SpotDimensions(2)];
                line(ax,x,y,'LineWidth',2)
                
                if i<=numSpotsRow1  % to avoid plotting an extra vehicle at the end
                    % Plot spot number
                    text(ax,x(1)+0.5*this.SpotDimensions(1)-0.5,y(1)+0.5*diff(y)+txtoffset,sprintf('%02u',spotIdx),'Color','w','FontSize',5)
                    
                    this.SpotLocations(spotIdx,:) = [x(1)+0.5*this.SpotDimensions(1) y(1)+0.5*diff(y)];
                    obsx = this.SpotLocations(spotIdx,1) - 0.5*this.ObstacleDimensions(2);
                    obsy = this.SpotLocations(spotIdx,2) - 0.5*this.ObstacleDimensions(1);
                    sensorRectX = obsx + 0.5*this.ObstacleDimensions(2) - 0.5;
                    sensorRectY = obsy + this.ObstacleDimensions(1) + 2;
                    this.SensorLocations(spotIdx,:) = [sensorRectX+0.5 sensorRectY+0.5];
                    
                    if this.OccupiedSpots(spotIdx)
                        % Plot occupied spots
                        rectangle(ax,'Position',[obsx obsy this.ObstacleDimensions(2) this.ObstacleDimensions(1)],'FaceColor','k')
                        % Plot sensor rectangles in red                        
                        rectangle(ax,'Position',[sensorRectX,sensorRectY,1,1],'FaceColor','r')
                        this.OccupiedSpots(spotIdx) = 1;
                    else
                        % Plot sensor rectangles in green                        
                        rectangle(ax,'Position',[sensorRectX,sensorRectY,1,1],'FaceColor','g')
                    end            
                    
                    spotIdx = spotIdx + 1;
                end
            end
            
            % Plot Row 2
            for i = numSpotsRow2+1:-1:1
                % parking lines
                x = (xStartRow2 + (i-1)*this.SpotDimensions(1))*[1 1];
                y = yStartRow2 + [-this.SpotDimensions(2) 0];
                line(ax,x,y,'LineWidth',2)
                
                if i<=numSpotsRow2
                    % Plot spot number
                    text(ax,x(1)+0.5*this.SpotDimensions(1)-0.5,y(1)+0.5*diff(y)-txtoffset,sprintf('%02u',spotIdx),'Color','w','FontSize',5)
                    
                    this.SpotLocations(spotIdx,:) = [x(1)+0.5*this.SpotDimensions(1) y(1)+0.5*diff(y)];
                    obsx = this.SpotLocations(spotIdx,1) - 0.5*this.ObstacleDimensions(2);
                    obsy = this.SpotLocations(spotIdx,2) - 0.5*this.ObstacleDimensions(1);
                    sensorRectX = obsx + 0.5*this.ObstacleDimensions(2) - 0.5;
                    sensorRectY = obsy - 3;
                    this.SensorLocations(spotIdx,:) = [sensorRectX+0.5 sensorRectY+0.5];
                    
                    if this.OccupiedSpots(spotIdx)
                        % Plot occupied spots
                        rectangle(ax,'Position',[obsx obsy this.ObstacleDimensions(2) this.ObstacleDimensions(1)],'FaceColor','k')
                        % Plot sensor rectangles in red                        
                        rectangle(ax,'Position',[sensorRectX,sensorRectY,1,1],'FaceColor','r')
                        this.OccupiedSpots(spotIdx) = 1;
                    else
                        % Plot sensor rectangles in green                        
                        rectangle(ax,'Position',[sensorRectX,sensorRectY,1,1],'FaceColor','g')
                    end 
                    
                    spotIdx = spotIdx + 1;
                end
            end
            
            %Plot Row 3
            for i = 1:numSpotsRow3+1
                % Plot parking lines
                x = (xStartRow3 + (i-1)*this.SpotDimensions(1))*[1 1];
                y = yStartRow3 + [0 this.SpotDimensions(2)];
                line(ax,x,y,'LineWidth',2)
                
                if i<=numSpotsRow3  % to avoid plotting an extra vehicle at the end
                    % Plot spot number
                    text(ax,x(1)+0.5*this.SpotDimensions(1)-0.5,y(1)+0.5*diff(y)+txtoffset,sprintf('%02u',spotIdx),'Color','w','FontSize',5)
                    
                    this.SpotLocations(spotIdx,:) = [x(1)+0.5*this.SpotDimensions(1) y(1)+0.5*diff(y)];
                    obsx = this.SpotLocations(spotIdx,1) - 0.5*this.ObstacleDimensions(2);
                    obsy = this.SpotLocations(spotIdx,2) - 0.5*this.ObstacleDimensions(1);
                    sensorRectX = obsx + 0.5*this.ObstacleDimensions(2) - 0.5;
                    sensorRectY = obsy + this.ObstacleDimensions(1) + 2;
                    this.SensorLocations(spotIdx,:) = [sensorRectX+0.5 sensorRectY+0.5];
                    
                    if this.OccupiedSpots(spotIdx)
                        % Plot occupied spots
                        rectangle(ax,'Position',[obsx obsy this.ObstacleDimensions(2) this.ObstacleDimensions(1)],'FaceColor','k')
                        % Plot sensor rectangles in red                        
                        rectangle(ax,'Position',[sensorRectX,sensorRectY,1,1],'FaceColor','r')
                        this.OccupiedSpots(spotIdx) = 1;
                    else
                        % Plot sensor rectangles in green                        
                        rectangle(ax,'Position',[sensorRectX,sensorRectY,1,1],'FaceColor','g')
                    end            
                    
                    spotIdx = spotIdx + 1;
                end
            end
            
            %Plot Row 4
            for i = numSpotsRow4+1:-1:1
                % parking lines
                x = (xStartRow4 + (i-1)*this.SpotDimensions(1))*[1 1];
                y = yStartRow4 + [-this.SpotDimensions(2) 0];
                line(ax,x,y,'LineWidth',2)
                
                if i<=numSpotsRow4
                    % Plot spot number
                    text(ax,x(1)+0.5*this.SpotDimensions(1)-0.5,y(1)+0.5*diff(y)-txtoffset,sprintf('%02u',spotIdx),'Color','w','FontSize',5)
                    
                    this.SpotLocations(spotIdx,:) = [x(1)+0.5*this.SpotDimensions(1) y(1)+0.5*diff(y)];
                    obsx = this.SpotLocations(spotIdx,1) - 0.5*this.ObstacleDimensions(2);
                    obsy = this.SpotLocations(spotIdx,2) - 0.5*this.ObstacleDimensions(1);
                    sensorRectX = obsx + 0.5*this.ObstacleDimensions(2) - 0.5;
                    sensorRectY = obsy - 3;
                    this.SensorLocations(spotIdx,:) = [sensorRectX+0.5 sensorRectY+0.5];
                    
                    if this.OccupiedSpots(spotIdx)
                        % Plot occupied spots
                        rectangle(ax,'Position',[obsx obsy this.ObstacleDimensions(2) this.ObstacleDimensions(1)],'FaceColor','k')
                        % Plot sensor rectangles in red                        
                        rectangle(ax,'Position',[sensorRectX,sensorRectY,1,1],'FaceColor','r')
                        this.OccupiedSpots(spotIdx) = 1;
                    else
                        % Plot sensor rectangles in green                        
                        rectangle(ax,'Position',[sensorRectX,sensorRectY,1,1],'FaceColor','g')
                    end 
                    
                    spotIdx = spotIdx + 1;
                end
            end
        end
        
        function plotVehicle(this,pose,steer)
            ax =  gca(this.Figure);
            ax.XLim = this.XLimits;
            ax.YLim = this.YLimits;
            % extract data and translate pose to center of vehicle
            th = pose(3);
            xc = pose(1) + this.centerToRear*cos(th);
            yc = pose(2) + this.centerToRear*sin(th);
            
            % corners of the vehicle rectangle in the order LU LD RD RU
            cornersx = xc + 0.5*this.VehicleDimensions(1)*[-1 -1 1 1];
            cornersy = yc + 0.5*this.VehicleDimensions(2)*[1 -1 -1 1];
            vbody = rotate( polyshape(cornersx,cornersy), rad2deg(th), [xc yc] );
            
            % prepare axles
            axwid = this.VehicleDimensions(1)/50; % axle width
            faxlex = xc + this.centerToFront*[1 1 1 1]+ 0.5*axwid*[-1 -1 1 1];
            raxlex = xc + this.centerToRear*[-1 -1 -1 -1]+ 0.5*axwid*[-1 -1 1 1];
            faxley = yc + 0.5*this.VehicleDimensions(2)*[1 -1 -1 1];
            raxley = faxley;
            axles(1) = rotate( polyshape(faxlex,faxley), rad2deg(th), [xc yc] );
            axles(2) = rotate( polyshape(raxlex,raxley), rad2deg(th), [xc yc] );
            
            % prepare wheels
            whlen = this.VehicleDimensions(1)/8;  % wheel rectangle length
            whwid = this.VehicleDimensions(1)/30; % wheel rectangle width
            whx = 0.5*whlen*[-1 -1 1 1]; % wheel rectangle corners x
            why = 0.5*whwid*[1 -1 -1 1]; % wheel rectangle corners y
            wheels0(1) = rotate( polyshape(xc-this.centerToRear+whx, yc+0.5*this.VehicleDimensions(2)+why), rad2deg(th), [xc yc] );
            wheels0(2) = rotate( polyshape(xc-this.centerToRear+whx, yc-0.5*this.VehicleDimensions(2)+why), rad2deg(th), [xc yc] );
            wheels0(3) = rotate( polyshape(xc+this.centerToRear+whx, yc-0.5*this.VehicleDimensions(2)+why), rad2deg(th), [xc yc] );
            wheels0(4) = rotate( polyshape(xc+this.centerToRear+whx, yc+0.5*this.VehicleDimensions(2)+why), rad2deg(th), [xc yc] );
            wheels(1) = wheels0(1);
            wheels(2) = wheels0(2);
            wheels(3) = rotate( wheels0(3), rad2deg(steer), [axles(1).Vertices(3,1), axles(1).Vertices(3,2)+0.5*axwid] );
            wheels(4) = rotate( wheels0(4), rad2deg(steer), [axles(1).Vertices(2,1), axles(1).Vertices(2,2)+0.5*axwid] );
            
            if isempty(this.VehicleBodyHandle) || any(~isvalid(this.VehicleBodyHandle))
                this.VehicleBodyHandle = plot(ax,vbody,'FaceColor','r','FaceAlpha',0.25);
                this.VehicleAxlesHandle = plot(ax,axles,'FaceColor','k','FaceAlpha',1);
                this.VehicleWheelsHandle = plot(ax,wheels,'FaceColor','k','FaceAlpha',1);
                this.VehiclePoly = images.roi.Polyline('Parent', ax, 'Position'...
                    , [pose(1) pose(2)], 'InteractionsAllowed', 'none','Color','g');
            else
                this.VehicleBodyHandle.Shape = vbody;
                for i = 1:2
                    this.VehicleAxlesHandle(i).Shape = axles(i);
                end
                for i = 1:4
                    this.VehicleWheelsHandle(i).Shape = wheels(i);
                end
            end
        end
        
        function plotVehiclePath(this,pose)
            this.VehiclePoly.Position(end+1,:) = [pose(1) pose(2)];
        end
        
        function plotCameraFOV(this,pose,depth,viewAngle)
            ax = gca(this.Figure);
            ax.XLim = this.XLimits;
            ax.YLim = this.YLimits;
            th = pose(3);
            xc = pose(1) + this.centerToRear*cos(th);
            yc = pose(2) + this.centerToRear*sin(th);
            theta = mod(pose(3), 2*pi);  % wrap theta
            
            phi_min = theta - 0.5*viewAngle;
            phi_max = theta + 0.5*viewAngle;
            phi = linspace(phi_min,phi_max,100); % phi is in radians
            x = xc + depth * cos(phi);
            y = yc + depth * sin(phi);
            x = [x xc x(1)];
            y = [y yc y(1)];
            if isempty(this.CameraFOVHandle) || ~isvalid(this.CameraFOVHandle)
                this.CameraFOVHandle = fill(ax, x, y, 'g', 'FaceAlpha', 0.25, 'Tag', 'arc');
            else
                this.CameraFOVHandle.XData = x;
                this.CameraFOVHandle.YData = y;
            end
        end
        
        function plotLidar(this,pose,lidar,maxLidarDist)
            ax = gca(this.Figure);
            ax.XLim = this.XLimits;
            ax.YLim = this.YLimits;
            numSensors = numel(lidar);
            th = pose(3);
            xc = pose(1) + this.centerToRear*cos(th);
            yc = pose(2) + this.centerToRear*sin(th);
            phi    = 0:2*pi/numSensors:2*pi*(1-1/numSensors);
            
            lidar = reshape(lidar,numSensors,1);
            lidarx = xc + lidar' .* cos(th+phi);
            lidary = yc + lidar' .* sin(th+phi);
            nlines = numSensors/2;
            xplot = zeros(2,nlines);
            yplot = zeros(2,nlines);
            for i = 1:nlines
                xplot(:,i) = [lidarx(i); lidarx(nlines+i)];
                yplot(:,i) = [lidary(i); lidary(nlines+i)];
            end
            
            if any(isempty(this.LidarLinesHandle)) || any(~isvalid(this.LidarLinesHandle))
                this.LidarLinesHandle = plot(ax, xplot, yplot, 'g', 'Tag', 'lidar');
            else
                for i = 1:nlines
                    this.LidarLinesHandle(i).XData = xplot(:,i);
                    this.LidarLinesHandle(i).YData = yplot(:,i);
                end
            end
            
            crosshandles = findobj(ax,'Tag','lidarcross');
            if any(~isempty(crosshandles)) || any(~isvalid(crosshandles))
                delete(crosshandles)
            end
            for i = 1:numSensors
                if lidar(i) < maxLidarDist
                    this.LidarCrossHandle = plot(ax,lidarx(i),lidary(i),'rx','Tag','lidarcross');
                end
            end
        end
        
        function plotTrainingZone(this)
            if isempty(this.TrainZoneHandle) || ~isvalid(this.TrainZoneHandle)
                ax = gca(this.Figure);
                x = [this.TrainingZoneXLimits(1) this.TrainingZoneXLimits(2) this.TrainingZoneXLimits(2) this.TrainingZoneXLimits(1)];
                y = [this.TrainingZoneYLimits(1) this.TrainingZoneYLimits(1) this.TrainingZoneYLimits(2) this.TrainingZoneYLimits(2)];
                this.TrainZoneHandle = fill(ax,x,y,'r','FaceAlpha',0.1);
            end
        end
        
        function plotVehicleStatus(this,status)
            ax = gca(this.Figure);
            txt = "Vehicle Status : " + status;
            if isempty(this.VehicleStatusHandle) || ~isvalid(this.VehicleStatusHandle)
                this.VehicleStatusHandle = text(ax, 2, 57, txt);
            else
                this.VehicleStatusHandle.String = txt;
            end
        end
        
        function plotControllerStatus(this,status)
            ax = gca(this.Figure);
            txt = "Controller Mode : " + status;
            if isempty(this.ControllerStatusHandle) || ~isvalid(this.ControllerStatusHandle)
                this.ControllerStatusHandle = text(ax, 2, 55, txt);
            else
                this.ControllerStatusHandle.String = txt;
            end
        end
        
        function plotTime(this,currentTime)
            ax = gca(this.Figure);
            txt = "Elapsed Time : " + currentTime + "s";
            if isempty(this.TimeDisplayHandle) || ~isvalid(this.TimeDisplayHandle)
                this.TimeDisplayHandle = text(ax, 2, 53, txt);
            else
                this.TimeDisplayHandle.String = txt;
            end
        end
        
        function clearVehicle(this)
            delete(this.VehicleBodyHandle)
            delete(this.VehicleAxlesHandle)
            delete(this.VehicleWheelsHandle)
        end
        
        function clearLidar(this)
            ax = gca(this.Figure);
            if ~isempty(ax) || ~isvalid(ax)
                crosshandles = findobj(ax,'Tag','lidarcross');
                delete(crosshandles)
            end
            delete(this.LidarLinesHandle)
        end
        
        function clearCameraFOV(this)
            delete(this.CameraFOVHandle)
        end
        
        function clearTrainZone(this)
            delete(this.TrainZoneHandle);
        end
        
    end
end
%% Local functions
function localValidateIdx(idx)
    if any(idx<0) || any(idx>64) || any(~isnumeric(idx)) || any((idx-floor(idx))~=0)
        error('Index values must be integers between 0 and 64.');
    end
end
    