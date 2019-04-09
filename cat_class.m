classdef cat_class
    properties
        legs = 4;
        x = 0;
        y = 0;
    end
    methods
        function obj = move (obj, steps_x, steps_y)
            obj.x = obj.x+steps_x;
            obj.y = obj.y+steps_y;
        end
        function meow(obj)
            'Meow'
        end
    end
end