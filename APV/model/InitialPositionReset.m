function [pose, speed] = InitialPositionReset()
% Reset function for auto parking valet example

% Copyright 2021 The MathWorks, Inc.

    choice = rand;
    if choice <= 0.35
        x = 0;
        y = -32.5;
        t = deg2rad(-45 + 2*45*rand);
    elseif choice <= 0.70       % centered around 5.6
        x = 11.2;
        y = -32.5;
        t = deg2rad(-225 + 2*45*rand);
    else
        zone = rand;
        if zone <= 0.4
            x = 1 + (4.5-0)*rand;               % average 3.25
            t = deg2rad(-45 + 2*45*rand);
        elseif zone <= 0.8
            x = 5.725 + (10.225-5.725)*rand;    % average 7.975
            t = deg2rad(-225 + 2*45*rand);
        else
            x = 4.4875 + (6.7375-4.4875)*rand;  % average 5.6125
            t = deg2rad(-135 + 2*45*rand);        
        end
        y = -34 + (33-30)*rand;                 % average -32.5
    end
    
    pose = [x,y,t];
    speed = 4 * rand;
%     in = setVariable(in,'egoInitialPose',pose);
%     in = setVariable(in,'egoInitialSpeed',speed);
end