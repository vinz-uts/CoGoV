classdef Obstacle < handle
    %OBSTACLE class
    %   Concrete class
    %   Class used to represent obstacles in a 2D simulated scenario.
    %   Obstacles are respresented as a collection of vertices v
    %   organized in the following way:
    %      v = | v_x1, v_x2, v_x3, ...., v_xn| = |v1, v2, ...., vn|.
    %          | v_y1, v_y2, v_y3, ...., v_yn|
    %
    %   For a square shaped obstacles a possible representation is
    %       (x1, y1)---------------(x4, y4)
    %           |                      |
    %           |                      |
    %       (x2, y2)---------------(x3, y3)    
    %   
    %   ob = Obstacle(v) create an obstacle object ob starting from the 
    %   vertices v. Note that the vertices v may be the output of a real
    %   life vision module like a LIDAR.
    %  
    %   
    %  Authors: Vincenzo D'Angelo, Ayman El Qemmah, Franco Angelo Torchiaro
    %  Credits: Alessandro Casavola, Francesco Tedesco
    %  Copyright 2021 - CoGoV.

    
    properties
        vertices % Vertices that represent the obstacle
    end
    
    methods
        
        function obj = Obstacle(v)
            %OBSTACLE - Constructor
            obj.vertices = v;            
        end
        
        function plot(obj, c)
            %PLOT(c)
            %   Plotting function for obstacles using the specified color
            %   c.
            %   
            h = ishold;
            hold on;
            shape = polyshape(obj.vertices(1, :), obj.vertices(2, :));
            plot(shape, 'FaceColor', 'red');
            if(h)
                hold on;
            else
                hold off;
            end
            
            
        end
        function move_obstacle(obj, deltax, deltay)
            %MOVE_OBSTACLE(deltax, deltay)
            %   apply a translation deltax, deltay to the obstacle.
            %   The new vertices will be computed as
            %      vx = vx + deltax
            %      vy = vy + deltay
            %    
            obj.vertices(1,:) = obj.vertices(1,:) + deltax;
            obj.vertices(2,:) = obj.vertices(2,:) + deltay;
        end
    end
end