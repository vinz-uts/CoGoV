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
        
        function [ob_seen] = seen_obstacles(obj, x_pos, R_vision, oblist)
            
            ob_seen = [];
            best_d = 1000;
            for i = 1:length(oblist)
                ob = oblist(i);
                vert = ob.vertices;
                d1 = norm(x_pos-vert(:,1));
                d2 = norm(x_pos-vert(:,2));
                d3 = norm(x_pos-vert(:,3));
                d4 = norm(x_pos-vert(:,4));
                
                if(min([d1 d2 d3 d4])==d1)
                    if(d1 < R_vision)
                        if(d1 < best_d)
                            best_d = d1;
                            ob_seen = ob;
                        end
                    end
                elseif(min([d1 d2 d3 d4])==d2)
                    if(d2 < R_vision)
                        if(d2 < best_d)
                            best_d = d2;
                            ob_seen = ob;
                        end
                    end
                    
                elseif(min([d1 d2 d3 d4])==d3)
                    if(d3 < R_vision)
                        if(d3 < best_d)
                            best_d = d3;
                            ob_seen = ob;
                        end
                    end
                else
                    if(d4 < R_vision)
                        if(d4 < best_d)
                            best_d = d4;
                            ob_seen = ob;
                        end
                    end
                end
                
            end
            
            
        end
    end
end