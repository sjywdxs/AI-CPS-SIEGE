function newpose = transformPose(pose,targetposeidx)

% Copyright 2021 The MathWorks, Inc.

x = pose(1);
y = pose(2);
t = pose(3);

if targetposeidx > 14 && targetposeidx <= 28
    x = 41 - pose(1);
    y = -64.485 - pose(2);
    t = pose(3) - pi;
elseif targetposeidx > 0 && targetposeidx <= 14
    x = pose(1);
    y = 20.41 + pose(2);
    t = pose(3);
elseif targetposeidx > 37 && targetposeidx <= 46
    x = 41 - pose(1);
    y = -84.48 - pose(2);
    t = pose(3) - pi;
end

newpose = [x y t];


