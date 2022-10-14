%% Parameters used by Auto Parking Valet 3D using MPC and RL example

% Copyright 2021 The MathWorks, Inc.
%%
% Ego vehicle dimension
centerToRear  = 1.343;      % distance from center to rear axle (m)

% Lidar parameters
maxLidarDist = 6;           % maximum distance that can be measured by lidar (m)
numSensors   = 12;          % number of lidar sensors
obsMat = map.ObstacleMatrix;
lidarTol    = 0.2;          % minimum distance measured by lidar (m)

% Error tolerances with target pose
xyerrTol    = 0.75;         % position error tolerance w.r.t. target pose (m)
terrTol     = deg2rad(10);  % orientation error tolerance w.r.t. target pose (m)

% Camera parameters
cameraDepth = 10;               % camera depth (m)
cameraViewAngle = deg2rad(120); % camera field of view (rad)

% Parameters for training
speedMax = 2;               % maximum speed of ego vehicle (m/s)
steerMax = pi/4;            % maximum steering angle (rad)
trainXBounds  = map.TrainingZoneXLimits;
trainYBounds  = map.TrainingZoneYLimits;
trainTBounds  = [-2*pi 2*pi];

% Parameters for simulation
xBounds  = map.XLimits;
yBounds  = map.YLimits;
tBounds  = [-Inf Inf];