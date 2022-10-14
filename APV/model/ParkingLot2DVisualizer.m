classdef ParkingLot2DVisualizer < matlab.System
    % ParkingLot2DVisualizer visualizes ParkingLotEnvironment

    % Copyright 2021 The MathWorks, Inc.

    % Public, non-tunable properties
    properties(Nontunable)
        % ParkingLot object name
        MapObject char
        % Depth of view of camera (m)
        CameraDepth      (1,1) double
        % Total view angle (rad)
        CameraViewAngle  (1,1) double
        % Max lidar distance (m)
        MaxLidarDistance (1,1) double
        % Training flag (0 or 1)
        TrainingFlag  (1,1) double
    end

    methods
        % Constructor
        function obj = ParkingLot2DVisualizer(varargin)
            setProperties(obj,nargin,varargin{:})
        end
    end

    methods(Access = protected)
        function stepImpl(obj,pose,steer,lidar,park,currentTime)
            % Implement algorithm.

            map = evalin('base',obj.MapObject);
            show(map)

            % Plot vehicle
            plotVehicle(map,pose,steer);

            if ~obj.TrainingFlag
                plotVehiclePath(map,pose)
            end

            % Plot/clear training zone
            if obj.TrainingFlag
                plotTrainingZone(map);
            else
                clearTrainZone(map);
            end

            % Plot/clear lidar and camera FOV
            if obj.TrainingFlag || park
                clearCameraFOV(map);
                plotLidar(map,pose,lidar,obj.MaxLidarDistance);
                if obj.TrainingFlag
                    plotVehicleStatus(map,'TRAIN');
                else
                    plotVehicleStatus(map,'PARK');
                end
                plotControllerStatus(map,'RL');
            else
                clearLidar(map);
                plotCameraFOV(map,pose,obj.CameraDepth,obj.CameraViewAngle);
                plotVehicleStatus(map,'SEARCH');
                plotControllerStatus(map,'MPC');
            end

            if round(currentTime,1) > floor(currentTime)
                plotTime(map,round(currentTime,1));
            end

            drawnow();
        end

        function resetImpl(obj)
            % Initialize / reset discrete-state properties
            %map = evalin('base',obj.MapObject);
            %initializeMap(map);
        end

        function ds = getDiscreteStateImpl(obj)
            ds = struct([]);
        end

        function flag = isInputSizeMutableImpl(obj,index)
            flag = false;
        end

        function out = getOutputSizeImpl(obj)
            out = [0 0];
        end

        function icon = getIconImpl(obj)
            icon = mfilename("class");
        end
    end

    methods(Static, Access = protected)
        function header = getHeaderImpl
            header = matlab.system.display.Header(mfilename("class"),...
                'Title','Parking Lot Simulator',...
                'Text', 'This system object simulates a Parking Lot with obstacles.');
        end

        function group = getPropertyGroupsImpl
            group = matlab.system.display.Section(mfilename("class"));
        end

        function simMode = getSimulateUsingImpl
            simMode = "Interpreted execution";
        end
    end
end
