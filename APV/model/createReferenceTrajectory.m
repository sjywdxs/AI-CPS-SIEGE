function Xref = createReferenceTrajectory(Ts,Tf,freeSpotIndex)
% Get x,y, and theta reference path
% 3 Sections: North, Left Turn, East

% Copyright 2021 The MathWorks, Inc.

if freeSpotIndex <= 14
    % North Section (91 points, 0.2 dist.)
    northY = (-55:0.2:-18)';
    northX = 40*ones(length(northY),1);
    northT = 90*ones(length(northY),1);
    northSection = [northX northY northT];

    % Turn Section (50->48 points, 0.1443 dist.)
    th = linspace( 0, pi/2, 50);
    R = 4.5;
    turnX = (R*cos(th)+35.5)';
    turnY = (R*sin(th)-18)';
    turnT = (rad2deg(th)+90)';
    turnSection = [turnX(2:end-1) turnY(2:end-1) turnT(2:end-1)]; % Remove overlapping points
    
    % West Section (193 points, 0.2 dist.)
    westX = (35.5:-0.2:-3)';
    westY = -13*ones(length(westX),1);
    westT = 180*ones(length(westX),1);
    westSection = [westX westY westT];

elseif freeSpotIndex <=37
    % North Section (91 points, 0.2 dist.)
    northY = (-55:0.2:-37)';
    northX = 40*ones(length(northY),1);
    northT = 90*ones(length(northY),1);
    northSection = [northX northY northT];

    % Turn Section (50->48 points, 0.1443 dist.)
    th = linspace( 0, pi/2, 50);
    R = 4.5;
    turnX = (R*cos(th)+35.5)';
    turnY = (R*sin(th)-37)';
    turnT = (rad2deg(th)+90)';
    turnSection = [turnX(2:end-1) turnY(2:end-1) turnT(2:end-1)]; % Remove overlapping points
    
    % West Section (193 points, 0.2 dist.)
    westX = (35.5:-0.2:-3)';
    westY = -32.5*ones(length(westX),1);
    westT = 180*ones(length(westX),1);
    westSection = [westX westY westT];
    
else
    % North Section (91 points, 0.2 dist.)
    northY = (-55:0.2:-54)';
    northX = 40*ones(length(northY),1);
    northT = 90*ones(length(northY),1);
    northSection = [northX northY northT];

    % Turn Section (50->48 points, 0.1443 dist.)
    th = linspace( 0, pi/2, 50);
    R = 4.5;
    turnX = (R*cos(th)+35.5)';
    turnY = (R*sin(th)-54)';
    turnT = (rad2deg(th)+90)';
    turnSection = [turnX(2:end-1) turnY(2:end-1) turnT(2:end-1)]; % Remove overlapping points
    
    % West Section (193 points, 0.2 dist.)
    westX = (35.5:-0.2:-3)';
    westY = -52*ones(length(westX),1);
    westT = 180*ones(length(westX),1);
    westSection = [westX westY westT];
end

% Combine 3 sections into single path
refPath = [northSection; turnSection; westSection];

Tsteps = Tf/Ts; %Add round or ceil
xRef = [refPath(:,1), refPath(:,2), deg2rad(refPath(:,3))];
p = size(xRef,1);
Xref = [xRef(1:p,:);repmat(xRef(end,:),Tsteps-p,1)];
end