classdef LinePlanner < Planner
    % Planner class used to follow straight lines
    
    properties
        points              % A vector containing all points that have to be reached
        point_iterator      % Index of curret point to be reached
        radius              % Variation in radians used to get the next reference
        line                % Function of the strait line used for debug feature
        slope               % line slope
        intercept           % line y intercept (x intercept with inf slope)
        direction           % direction of movement (-1 from right to left, 1 from left to right)
    end
    
    methods
        function obj = LinePlanner(references, varargin)
            % Class constructor
            % references - reference points expressed as a single vector
            % Valid variable arguments:
            % radius    - Radius [m] of the circumference used for the calculation of intermediate references
            % tolerance - tolerance used to understand if the reference was
            %             reached (in [m])
            
            obj = obj@Planner(varargin{:}); % Superclass constructor
            
            obj.points = references;
            
            % Default values
            obj.point_iterator = 1;
            obj.radius = 0.5;
            obj.tol = 0.2;
            
            %%%%%%% Variable arguments management %%%%%%%
            validnames = {'tolerance', 'radius'};
            
            nargs = length(varargin);
            params = varargin(1:2:nargs);   values = varargin(2:2:nargs);
            
            for name = params
                pos = strcmp(name{:}, params);
                switch name{:}
                    case validnames{1}
                        obj.tol =  values{pos};
                    case validnames{2}
                        obj.radius =  values{pos};
                end
            end
            %%%%%%%%%%%%
        end
        
        function r = initialize_old_reference(obj, p) % inherited abstract method
            r = obj.points(obj.point_iterator:obj.point_iterator+1)';
            if(r(1) - p(1) > 0)
                obj.direction = 1;
            else
                obj.direction = -1;
            end
            
            if((r(1) - p(1)) == 0)
                obj.slope = inf;
                obj.intercept = p(1);
                if(r(2) - p(2) < 0)
                    obj.direction = 1;
                else
                    obj.direction = -1;
                end
            else
                obj.slope = (r(2) - p(2))/(r(1) - p(1));
                obj.intercept = -obj.slope*p(1) + p(2);
            end
            
            
            if(obj.slope == inf)
                obj.line = @(x) (obj.intercept);
            else
                obj.line = @(x) (x*obj.slope + obj.intercept);
            end
            r = obj.compute_next_reference(p);
        end
        
        function [r, theta] = compute_referecence_when_not_reached(obj, p) % inherited abstract method
            r = obj.r_old;
            theta = atan2(r(2) - p(2),r(1) - p(1));
        end
        
        function res = reference_reached(obj, p) % inherited abstract method
            res = norm(p(1) - obj.r_old(1)) < obj.tol || norm(p(2) - obj.r_old(2)) < obj.tol;
        end
        
        function [r, theta] = compute_standard_reference(obj, p) % inherited abstract method
            ref = obj.points(obj.point_iterator:obj.point_iterator+1)';
            if(norm(p - ref) > obj.tol) % If the vehicle has reached an intermediate reference
                r = obj.compute_next_reference(p);
                
                if(obj.slope == inf)
                    
                    if(obj.direction > 0)
                        if(r(2) < ref(2))
                            r = ref;
                        end
                        
                    else
                        if(r(2) > ref(2))
                            r = ref;
                        end
                    end
                    
                else
                    if(obj.direction < 0)
                        if(r(1) < ref(1))
                            r = ref;
                        end
                        
                    else
                        if(r(1) > ref(1))
                            r = ref;
                        end
                    end
                end
                
                % adjust the reference if it exceeds the final one
                
            else % If the vehicle has reached the real reference
                obj.point_iterator = obj.point_iterator + 2; % change reference
                
                if(obj.point_iterator >= length(obj.points))
                    obj.point_iterator = 1;
                end
                
                r = obj.initialize_old_reference(p); % calculate another line
                hold on;
                plot(-2:0.1:2.5, obj.line(-2:0.1:2.5))
                
            end
            theta = atan2(r(2) - p(2),r(1) - p(1));
            
        end
        
        function r = compute_next_reference(obj, p)
            r = obj.points(obj.point_iterator:obj.point_iterator+1)'; % get the not intermediate reference
            % computentersections of circles and lines in Cartesian plane
            [xout,yout] = linecirc(obj.slope, obj.intercept,p(1), p(2), obj.radius);
            if(isnan(xout(1)) || isnan(yout(1))) % if it can not be found compute a different line
                if(r(1) - p(1) > 0)
                    obj.direction = 1;
                else
                    obj.direction = -1;
                end
                if((r(1) - p(1)) == 0)
                    obj.slope = inf;
                    obj.intercept = p(1);
                    if(r(2) - p(2) < 0)
                        obj.direction = 1;
                    else
                        obj.direction = -1;
                    end
                    
                else
                    obj.slope = (r(2) - p(2))/(r(1) - p(1));
                    obj.intercept = -obj.slope*p(1) + p(2);
                end
                
                if(obj.slope == inf)
                    obj.line = @(x) (obj.intercept);
                else
                    obj.line = @(x) (x*obj.slope + obj.intercept);
                end
                
                [xout,yout] = linecirc(obj.slope, obj.intercept,p(1), p(2), obj.radius);
            end
            
            % Choose the right poit to use as an intermediate reference
            if(obj.slope == inf)
                if(obj.direction < 0)
                    if(yout(1) > yout(2))
                        r(1) = xout(1);
                        r(2) = yout(1);
                    else
                        r(1) = xout(2);
                        r(2) = yout(2);
                    end
                else
                    if(yout(1) < yout(2))
                        r(1) = xout(1);
                        r(2) = yout(1);
                    else
                        r(1) = xout(2);
                        r(2) = yout(2);
                    end
                end
            else
                if(obj.direction < 0)
                    if(xout(1) < xout(2))
                        r(1) = xout(1);
                        r(2) = yout(1);
                    else
                        r(1) = xout(2);
                        r(2) = yout(2);
                    end
                else
                    if(xout(1) > xout(2))
                        r(1) = xout(1);
                        r(2) = yout(1);
                    else
                        r(1) = xout(2);
                        r(2) = yout(2);
                    end
                end
            end
        end
    end
end

