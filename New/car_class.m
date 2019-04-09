classdef car_class
    properties
        x = 0;
        y = 0;
        dx = 0;
        dy = 0;
        w = 0; %Это угловая скорость вращения колеса (в рад/с)
        %wheel_speed = 0;
        wheel_angle = 0
        course_angle = 90;
        wheel_radius = 0.2;
        base_length = 1 ;
        
    end
    methods
        function obj = change_w(obj, delta_w)
            obj.w = obj.w + delta_w;
        end
        function obj = change_wheel_ang(obj, delta_angle) %ДУ
            obj.wheel_angle = obj.wheel_angle + delta_angle;
        end
        function obj = speed_projections(obj)
            obj.dx = obj.v * cosd(obj.course_angle);
            obj.dy = obj.v * sind(obj.course_angle);
        end
        function obj = coordinates(obj, dt)
            obj.x = obj.x + 2*obj.wheel_radius*dt*obj.w*cosd(obj.course_angle); %ДУС
            obj.y = obj.y + 2*obj.wheel_radius*dt*obj.w*sind(obj.course_angle);
        end
        function obj = course(obj,dt) %Компас
            obj.course_angle = rad2deg(deg2rad(obj.course_angle) + (2*obj.wheel_radius*obj.w * deg2rad(obj.wheel_angle) * dt)/obj.base_length);
        end
    end
end