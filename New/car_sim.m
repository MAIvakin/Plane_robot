clear X Y cx xy course_diff course_graph graph_t wheel_angle_graph wheel_spd_graph
Car = car_class;
Car.x = start_coord(1);
Car.y = start_coord(2);
dt = 0.1;
graph_counter = 1;
t = 0;
k = 2;
X(1) = start_coord(1);
Y(1) = start_coord(2);
cx(1) = 90;
cy(1) = 0;
max_angle = 40;
wheel_rotate_spd = 0;
%Для построения графиков
%wheel_angle_graph
%wheel_spd_graph
%course_graph
Car = change_w(Car, 2); %Это должна быть угловая скорость вращения колеса
for i = 1:length(trajectory_points)
while sqrt((abs(trajectory_points(i,1)-Car.x))^2 + (abs(trajectory_points(i,2)-Car.y))^2)>0.5
    if t >= 200
        break
    end
    Car = Car.course(dt);
    %Car = Car.speed_projections;
    Car = Car.coordinates(dt);  
    X(k) = Car.x;
    Y(k) = Car.y;
    cx(k) = Car.course_angle;
    cy(k) = rad2deg(atan2((trajectory_points(i,2)-Car.y),(trajectory_points(i,1)-Car.x)));
    k = k+1;
%     if Car.course_angle ~= rad2deg(atan2((trajectory_points(i,2)-Car.y),(trajectory_points(i,1)-Car.x)))
%         if Car.course_angle > rad2deg(atan2((trajectory_points(i,2)-Car.y),(trajectory_points(i,1)-Car.x))) && Car.wheel_angle > -40
%             Car = change_wheel_ang(Car, -5);
%         end
%         if Car.course_angle < rad2deg(atan2((trajectory_points(i,2)-Car.y),(trajectory_points(i,1)-Car.x))) && Car.wheel_angle < 40
%             Car = change_wheel_ang(Car, 5);
%         end
%     end
    %Car = change_wheel_ang(Car, rad2deg(atan2((trajectory_points(i,2)-Car.y),(trajectory_points(i,1)-Car.x))) - Car.course_angle);
    wheel_angle_graph(graph_counter) = Car.wheel_angle;
    wheel_spd_graph(graph_counter) = Car.w;
    course_graph(graph_counter) = Car.course_angle;
    graph_t(graph_counter) = t;
    graph_counter = graph_counter + 1;
    t = t+dt;
end
end
plot(X,Y)
grid on
figure;
plot(graph_t,wheel_angle_graph)
grid on
figure;
plot(graph_t,wheel_spd_graph)
grid on
figure;
plot(graph_t,course_graph)
grid on
axis equal

%     if Car.course_angle ~= rad2deg(atan2((trajectory_points(i,2)-Car.y),(trajectory_points(i,1)-Car.x)))
%         if Car.course_angle > rad2deg(atan2((trajectory_points(i,2)-Car.y),(trajectory_points(i,1)-Car.x)))
%             Car = change_wheel_ang(Car, -10);
%         else
%             Car = change_wheel_ang(Car, 10);
%         end
%     end