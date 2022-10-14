function animatedPath(out)
% Animate the ball path on the plate

% Copyright 2021, The MathWorks Inc.

persistent ax

doViz = evalin('base','doViz');
if doViz
    if isempty(ax) || ~isvalid(ax)
        f = figure("Name","Ball Balance Animation", ...
            "NumberTitle","off", ...
            "MenuBar","none", ...
            "Position",[500,500,300,300]);
        ax = gca(f);
        hold(ax,'on')
        title(ax,'Ball position on plate')
        xlabel(ax,'X (m)');
        ylabel(ax,'Y (m)');
    end
    cla(ax);
    ball = evalin('base','ball');
    plate = evalin('base','plate');
    rectangle(ax,"Position",plate.width*[-0.5,-0.5,1,1],"FaceColor","c");
    ballx = out.logsout{2}.Values.Data(:,1);
    bally = out.logsout{2}.Values.Data(:,2);
    plot(ballx,bally,"b.")
    hball = rectangle(ax, ...
        "Position", [ballx(1),bally(1),0,0] + 2*ball.radius*[-0.5,-0.5,1,1], ...
        "Curvature", [1,1], ...
        "FaceColor","r");
    axis(ax,'equal');
    ax.XLim = 1.05 * plate.width * [-1,1];
    ax.YLim = 1.05 * plate.width * [-1,1];
    for ct = 1:numel(ballx)
        hball.Position = [ballx(ct),bally(ct),0,0] + 2*ball.radius*[-0.5,-0.5,1,1];
        drawnow limitrate;
    end
end
