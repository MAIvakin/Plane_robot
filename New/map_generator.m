global ax f
f = figure('Visible','on','Name','Map','NumberTitle','off');
ax = axes('Units', 'normalized', 'Position', [0.05 0.2 0.9 0.7]);
ax.XLim = [-10 10];
ax.YLim = [-10 10];
hold on
axis equal
grid on
    
% Set start point
uicontrol('Style', 'pushbutton', 'String', 'Set start point',...
        'Units', 'normalized','Position', [0.05 0.05 0.2 0.1],...
        'Callback', @set_start_point);
   
% Set traj point
uicontrol('Style', 'pushbutton', 'String', 'Set target point',...
        'Units', 'normalized','Position', [0.35 0.05 0.2 0.1],...
        'Callback', @set_trajectory_point);
% Exit
uicontrol('Style', 'pushbutton', 'String', 'Finished',...
        'Units', 'normalized','Position', [0.65 0.05 0.2 0.1],...
        'Callback', @finished);

function set_start_point(source, event)
    global ax
    dot = ginput(1);
    xstart = dot(1);
    ystart = dot(2);
    start_coord = [xstart, ystart];
    plot(ax,xstart,ystart,'*')
    assignin('base','start_coord',start_coord);
end

function set_trajectory_point(source, event)
    global ax
    i = 1;
    for i = 1:100
    dot = ginput(1);
    xtarget = dot(1);
    ytarget = dot(2);
    %target_coord = [xtarget, ytarget];
    trajectory_points(i,:) = [xtarget, ytarget];
    i = i+1;
    plot(ax,xtarget, ytarget,'o')
    assignin('base','trajectory_points',trajectory_points);
    end 
end
function finished(source, event)
    global f
    close(f)
end