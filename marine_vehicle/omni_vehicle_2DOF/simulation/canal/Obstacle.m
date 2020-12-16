classdef Obstacle < handle
    
    properties
        vertices
    end
    
    methods
        
        function obj = Obstacle(v1, v2, v3, v4)
            
            
            v = zeros(2, 4);
            v(:,1) = v1;
            v(:,2) = v2;
            v(:,3) = v3;
            v(:,4) = v4;
            
            obj.vertices = v;
            
        end
        
        function plot(obj, c)
            % Plotting function for obstacles
            
            % ob -- Object containing info about the obstacle
            % c  -- Color and type of plotting
            % fig -- Which figure on which we want to draw the obstacle
            
            
            h = ishold;
            
            
            hold on;
            
            % Interval of x and y
            %       (x1, y1)---------------(x4, y4)
            %           |                      |
            %           |                      |
            %       (x2, y2)---------------(x3, y3)
            % x goes from x1 to x4 and y goes from y2 to y1
            
            x = obj.vertices(1, 1):0.1:obj.vertices(1, 4);
            y = obj.vertices(2, 2):0.1:obj.vertices(2, 1);
            
            % plot
            plot(ones(length(y))*x(1), y, c);
            plot(x, ones(length(x))*y(1), c);
            plot(ones(length(y))*x(end), y, c);
            plot(x, ones(length(x))*y(end), c);
            
            if(h)
                hold on;
            else
                hold off;
            end
            
            
        end
        function move_obstacle(obj, deltax, deltay)
            % Function usefull to move object by deltax and deltay
            
            obj.vertices(1,:) = obj.vertices(1,:) + deltax;
            obj.vertices(2,:) = obj.vertices(2,:) + deltay;
            
        end
        
    end
end