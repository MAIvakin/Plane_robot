function RDK_Sim(obstacles, showlaser, trajectory)
	%Comment
    RDK = RDK_class;
    point_id = 1;
    
    initime = cputime;

    dt = 0.2;
    rays = 180;
    model_steps = 20000;
    Ray_length = 17;
    model = Triangle_graph_model(RDK);
    info_base = [22.814 22.814 10.684; %А320
                 11.619 11.619 5.74; %SSJ-100
                 26.628 26.628 10.4; %IL-96
                 24.37 24.37 8.8; %IL-62
                 26.18 26.18 11]; %Boeing 747
    syms A320 SSJ100 IL96 IL62 Boeing747
    model_base = [A320 SSJ100 IL96 IL62 Boeing747];
    if nargin > 2
        traj = trajectory;
        RDK.targetX = traj(point_id,1);
        RDK.targetY = traj(point_id,2);
    else
        traj = [0,0;6,3];
    end
    
    f = figure('Visible','on','Name','RDK sim','NumberTitle','off');
    ax = axes('Units', 'normalized', 'Position', [0.05 0.05 0.4 0.4]);
    ax2 = axes('Units', 'normalized', 'Position', [0.05 0.55 0.4 0.4]);
    ax3 = axes('Units', 'normalized', 'Position', [0.55 0.05 0.4 0.4]);
    ax4 = axes('Units', 'normalized', 'Position', [0.55 0.55 0.4 0.4]);
    
    axes(ax2)
    obstacle_plots(1) = plot(ax2,0,0,'r');
    hold on
    RDK_measured_plot = plot(ax2,model(:,1), model(:,2),'b','linewidth',3);
    ax2.XLim = [-20 20];
    ax2.YLim = [-15 15];
    axis equal
    grid on
    
    axes(ax3)
    [cx, cy] = circle(0, 0, Ray_length);
    plot(cx, cy, 'Color',[0.6 0.6 0.6],'linewidth',3);
    hold on
    tri.x = 0;
    tri.y = 0;
    tri.theta = 0;
    tri.model = Triangle_graph_model(tri);
    plot(cx, cy, 'Color',[0.6 0.6 0.6]);
    plot(tri.model(:,1),tri.model(:,2),'b','linewidth',3);
    measure_plot = scatter(ax3,0,0,'.','r');
    ax3.XLim = [-Ray_length*1.1 Ray_length*1.1];
    ax3.YLim = [-Ray_length*1.1 Ray_length*1.1];
    axis equal
    grid on
    
    axes(ax4)
    ax4.XLim = [-20 20];
    ax4.YLim = [-15 15];
    hold on
    RDK_collision_plot = plot(ax4,model(:,1), model(:,2),'b','linewidth',3);
    Init_obstacles(obstacles, ax4);
    target_line = plot(ax4,0,0,'g','linewidth',1);
    collision_plot = scatter(ax4,0,0,'*','g');
    axis equal
    grid on
    
    
    axes(ax)
    ax.XLim = [-20 20];
    ax.YLim = [-15 15];
    Xtraj = [RDK.x];
    Ytraj = [RDK.y];
    RDK_plot = plot(ax,model(:,1), model(:,2),'b','linewidth',3);
    hold on
    zones_plots(1) = plot(ax,0,0,'Color',[1 0.6 0.0],'linewidth',2);
    RDK_trajectory_plot = plot(ax, Xtraj, Ytraj,'m','linewidth',2);
    plot(ax, trajectory(:,1),trajectory(:,2), '--g','linewidth',2);
    axis equal
    grid on
    if nargin > 1 && showlaser
        hold on;
        Laser_plot = plot(ax,0, 0,'g','linewidth',1,'MarkerSize',10);
    end

    Init_obstacles(obstacles, ax);
    distance = zeros(model_steps,rays);
    Vx = zeros(1,model_steps);
    Vy = zeros(1,model_steps);
    dz = [];
    
    wheel_massive = zeros(1,2,3);
    wheel_active = 1;
    a = 0;
    j = 1;
    
    for i = 1:model_steps
        %measurement
        [laser_lines, measured_distance] = Measure_laser(RDK, obstacles, rays, Ray_length);
        distance(i,:) = measured_distance;
        if nargin > 1 && showlaser
            laser_model = Laser_Lines_Draw(laser_lines);
        end
        measure_model = Measure_draw(laser_lines, Ray_length);
        measured_obstacles = Find_obstacles(measure_model);
        measured_obstacles = Join_obstacles(measured_obstacles);
        measured_obstacles = Convert_obstacle(RDK, measured_obstacles);
        measured_obstacles = Sort_obstacles(RDK, measured_obstacles);
        danger_zones = Find_danger_zones(RDK, measured_obstacles);
        dz = Join_zones(dz,danger_zones);
        dz = Sort_zone(dz);
        
        [move_collisions, collision_obstacles] = Collision_detection(RDK,danger_zones);
        if ~isempty(move_collisions)
            new_point = work_around(RDK, move_collisions, collision_obstacles);
            if ~RDK.Avoiding_obstacle
                RDK.Avoiding_obstacle = true;
            end
            RDK.workAroundX = new_point(1);
            RDK.workAroundY = new_point(2);
        else
            RDK.Avoiding_obstacle = false;
        end
        
        
        %check target
        if point_reached(RDK)
            point_id = point_id+1;
            if (point_id>length(traj))
                break
            end
            RDK.targetX = traj(point_id,1);
            RDK.targetY = traj(point_id,2);
        end
        
        %control
        [V,w] = Regulator (RDK);

        %use of control
        RDK=RDK.move(V, w, dt);
        Xtraj = [Xtraj RDK.x];
        Ytraj = [Ytraj RDK.y];
        
        %draw
        model = Triangle_graph_model(RDK); %комменты были ниже
        set(RDK_plot,'xdata',model(:,1),'ydata', model(:,2));
        set(RDK_trajectory_plot,'xdata',Xtraj,'ydata', Ytraj);
        if nargin > 1 && showlaser
            set(Laser_plot,'xdata',laser_model(:,1),'ydata', laser_model(:,2));
        end
        ax.XLim = [-20 20];
        ax.YLim = [-15 15];
        
        obstacle_plots = plot_measured_obstacles(measured_obstacles, ax2, obstacle_plots);
        set(zones_plots(1),'xdata',dz(:,1),'ydata', dz(:,2))
%         zones_plots = plot_danger_zones(danger_zones, ax, zones_plots);
        set(RDK_measured_plot,'xdata',model(:,1),'ydata', model(:,2));
        ax2.XLim = [-20 20];
        ax2.YLim = [-15 15];
        set(measure_plot,'xdata',measure_model(:,1),'ydata', measure_model(:,2));
        
        set(RDK_collision_plot,'xdata',model(:,1),'ydata', model(:,2));
        if ~RDK.Avoiding_obstacle
            set(target_line,'xdata',[RDK.x, RDK.targetX],'ydata', [RDK.y, RDK.targetY]);
        else
            set(target_line,'xdata',[RDK.x, RDK.workAroundX],'ydata', [RDK.y, RDK.workAroundY]);
        end
        if (move_collisions)
            set(collision_plot,'xdata',move_collisions(:,1),'ydata', move_collisions(:,2));
        end
        drawnow
            %... и выше
        %RDK speed projections on X and Y axes.
        %The first line of "projections" matrix is X-projections
        %The second line is Y-projections
        Vx(i) = V*cos(RDK.theta);
        Vy(i) = V*sin(RDK.theta);
        Vproj = [Vx;Vy];
        
        %опасно
        work_massive = [];
        for q = 1:length(measured_obstacles)
            work_massive = [work_massive;measured_obstacles(q).points];
        end
        for qq = 1:length(work_massive)
            work_massive(qq,3) = Length_between(laser_lines(1,:,1),work_massive(qq,1:2));
            if work_massive(qq,3) <= 3
                a = wheel_active;
                wheel_massive(end+1,:,wheel_active) = work_massive(qq,1:2);
            end
        end
        if isempty(work_massive(find(work_massive(:,3)<=3))) == 1
            wheel_active = a + 1;
        end
    end
    assignin('base','wheel_massive',wheel_massive)
    mid1 = []; mid2 = []; mid3 = [];
        for j = 1:length(wheel_massive)
            if wheel_massive(j,1,1) ~= 0 && wheel_massive(j,2,1) ~= 0
                mid1 = [mid1; wheel_massive(j,:,1)];
            end
            if wheel_massive(j,1,2) ~= 0 && wheel_massive(j,2,2) ~= 0
                mid2 = [mid2; wheel_massive(j,:,2)];
            end
            if wheel_massive(j,1,3) ~= 0 && wheel_massive(j,2,3) ~= 0
                mid3 = [mid3; wheel_massive(j,:,3)];
            end
        end
    mid1 = [mean(mid1(:,1)) mean(mid1(:,2))];
    mid2 = [mean(mid2(:,1)) mean(mid2(:,2))];
    mid3 = [mean(mid3(:,1)) mean(mid3(:,2))];
    %Здесь нужен алгоритм выбора среди трёх точек двух основных (основные стойки шасси)
    midmid = [];
    assignin('base','mid1',mid1); assignin('base','mid2',mid2); assignin('base','mid3',mid3)
    length_massive = [Length_between(mid1, mid2); Length_between(mid1,mid3); Length_between(mid2,mid3)];
    if Length_between(mid1, mid2)>  Length_between(mid1,mid3)
        D = (mid1+ mid3)/2;
        X_c = (mid2(1,1)-D(1,1));
        Y_c = (mid2(1,2)-D(1,2));
    else
        D = (mid1+ mid2)/2;
        X_c =(mid3(1,1)-D(1,1));
        Y_c = (mid3(1,2)-D(1,2));
    end
    psi_c = atand((X_c)/(Y_c));
    check_kvadrant(X_c,Y_c,psi_c)
    assignin('base','psi_c',psi_c)
    sorted_length = sort(length_massive,1,'descend');
    for u = 1:length(info_base)
        dif_massive(u,1:3) = info_base(u,:) - sorted_length';
        dif_massive(u,4) = sum(abs(dif_massive(u,1:3)));
    end
    result = find(dif_massive(:,4)==min(dif_massive(:,4)));
    model_base(result)
    dz = [dz; danger_zones(1).points];
    dz = Sort_zone(dz);
    set(zones_plots(1),'xdata',dz(:,1),'ydata', dz(:,2))
    assignin('base','projections',Vproj)
    assignin('base','distance',distance)
    load handel
    fintime = cputime;
    fprintf('CPUTIME: %g\n', fintime - initime);
%     w_c_1(1,1) = mean(wheel_coord(2:(end-1),1,1));
%     w_c_1(1,2) = mean(wheel_coord(2:(end-1),2,1));
%     w_c_2(1,1) = mean(wheel_coord(:,1,2));
%     w_c_2(1,2) = mean(wheel_coord(:,2,2));
%     w_c_3(1,1) = mean(wheel_coord(:,1,3));
%     w_c_3(1,2) = mean(wheel_coord(:,2,3));
%     assignin('base','w_c_1',w_c_1)
%     assignin('base','w_c_2',w_c_2)
%     assignin('base','w_c_3',w_c_3)
%     sound(y,Fs)
end

function plots = plot_measured_obstacles(measured_obstacles, axes, plots)
    n_obstacles = length(measured_obstacles);
    n_plots = length(plots);
    for i = 1:n_plots
        set(plots(i),'xdata',[0,0],'ydata',[0,0]);
    end
    if (n_plots<n_obstacles)
        for i = n_plots:n_obstacles
            plots(i) = plot(axes,0,0,'r');
        end
    end
    if n_obstacles>=1
        for i=1:n_obstacles
            x1 = measured_obstacles(i).points(1,1);
            y1 = measured_obstacles(i).points(1,2);
            set(plots(i),'xdata',[measured_obstacles(i).points(:,1);x1],'ydata',[measured_obstacles(i).points(:,2);y1]);
        end
    end
end

function plots = plot_danger_zones(measured_obstacles, axes, plots)
    n_obstacles = length(measured_obstacles);
    n_plots = length(plots);
    for i = 1:n_plots
        set(plots(i),'xdata',[0,0],'ydata',[0,0]);
    end
    if (n_plots<n_obstacles)
        for i = n_plots:n_obstacles
            plots(i) = plot(axes,0,0,'m');
        end
    end
    if n_obstacles>=1
        for i=1:n_obstacles
            x1 = measured_obstacles(i).points(1,1);
            y1 = measured_obstacles(i).points(1,2);
            set(plots(i),'xdata',[measured_obstacles(i).points(:,1);x1],'ydata',[measured_obstacles(i).points(:,2);y1]);
        end
    end
end

function result = point_reached(RDK)
    dist = sqrt((RDK.x-RDK.targetX)^2 +(RDK.y-RDK.targetY)^2);

    if dist < 0.05
        result = true;
    else
        result = false;
    end
end

function [xunit, yunit] = circle(x,y,r)
    th = 0:pi/50:2*pi;
    xunit = r * cos(th) + x;
    yunit = r * sin(th) + y;
end
    




