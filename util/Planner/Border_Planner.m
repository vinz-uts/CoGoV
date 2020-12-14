classdef Border_Planner < Planner
    %% BORDER PLANNER 
    % Concrete class used to compute references in a pool. When hitting the
    % boundaries of the pool, the new vehicle's trajectory starts with the
    % same strike angle.
    % The angles are computed using straight lines described by the
    % following equation:
    %               y = mx + q
    % The new references are calculated as the intersection between 
    % the straight lines that define the boundaries of the pool and the 
    % straight line that characterizes the movement of the vehicle.
    % In this version Intermediate references are used until
    % the vehicle reaches the edges
    properties
        limits          % A vector containing all points that have to be reached
        radius              % Variation in radians used to get the next reference
        line                % Function of the strait line used for debug feature
        slope               % line slope
        intercept           % line y intercept (x intercept with inf slope)
        direction           % direction of movement (-1 from right to left, 1 from left to right)
        border_reference
    end
    
    methods
        function obj = Border_Planner(boundaries, starting_slope, varargin)
            % Class constructor
            % references - reference points expressed as a single vector
            % Valid variable arguments:
            % radius    - Radius [m] of the circumference used for the calculation of intermediate references
            % tolerance - tolerance used to understand if the reference was
            %             reached (in [m])
            
            obj = obj@Planner(varargin{:}); % Superclass constructor
            
            % Default values
            obj.radius = 0.5;
            obj.tol = 0.1;
            obj.slope = starting_slope;
            obj.direction = 1;
            obj.limits = struct('x_l', -boundaries(1), 'x_r', boundaries(1), 'y_t', boundaries(2), 'y_b', -boundaries(2));
            
            %%%%%%% Variable arguments management %%%%%%%
            validnames = {'tolerance', 'radius', 'direction'};
            
            nargs = length(varargin);
            params = varargin(1:2:nargs);   values = varargin(2:2:nargs);
            
            for name = params
                pos = strcmp(name{:}, params);
                switch name{:}
                     case validnames{1}
                         obj.tol =  values{pos};
                    case validnames{2}
                        obj.radius =  values{pos};
                    case validnames{3}
                        obj.direction =  values{pos};
                end
            end 
            %%%%%%%%%%%%
        end
        
         function r = initialize_old_reference(obj, p) % inherited abstract method
             obj.update_line(p, obj.slope, obj.direction);
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
            % compute m, q and check if it is necessary to change direction
            % if the vehicle has reached the left or right edge, the direction of movement must be changed
            if(norm(p(1) - obj.limits.x_l) < obj.tol  || norm(p(1) - obj.limits.x_r) < obj.tol &&...
                    norm(p - obj.border_reference) < obj.tol)
                obj.update_line(p, -obj.slope, -obj.direction);
                % if the vehicle has reached the upper or lower edge, it is not necessary to change the direction of movement
            elseif(norm(p(2) - obj.limits.y_t) < obj.tol || norm(p(2) - obj.limits.y_b) < obj.tol && ...
                    norm(p - obj.border_reference) < obj.tol)
                obj.update_line(p, -obj.slope, obj.direction);
            end
            r = compute_next_reference(obj, p);
            
            % adjust the reference if it exceeds the final one
            if(obj.direction < 0)
                if(r(1) < obj.border_reference(1))
                    r = obj.border_reference;
                end
            else
                if(r(1) > obj.border_reference(1))
                    r = obj.border_reference;
                end
            end
            
            theta = atan2(r(2)-p(2),r(1)-p(1));
           
        end
        
        function r = compute_next_reference(obj, p)
             r = obj.border_reference; % get the not intermediate reference
             % computentersections of circles and lines in Cartesian plane
             [xout,yout] = linecirc(obj.slope, obj.intercept,p(1), p(2), obj.radius); 
             if(isnan(xout(1)) || isnan(yout(1))) % if it can not be found compute a different line
                 if(r(1) - p(1) > 0)
                     obj.direction = 1;
                 else
                     obj.direction = -1;
                 end
                 obj.slope = (r(2) - p(2))/(r(1) - p(1));
                 obj.intercept = -obj.slope*p(1) + p(2);
                 
                 obj.line = @(x) (x*obj.slope + obj.intercept);
                 [xout,yout] = linecirc(obj.slope, obj.intercept,p(1), p(2), obj.radius);
             end
             
             % Choose the right poit to use as an intermediate reference
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
        
        function update_line(obj, p, slope, direction)
            obj.intercept = p(2) - slope*p(1); 
            obj.slope = slope;
            if(slope < inf)
                obj.line = @(x) (slope*x+obj.intercept);
            else
                obj.line = @(x) (obj.intercept);
            end
            obj.direction = direction;
            inv_line = @(y) ((y - obj.intercept) \ obj.slope);
            % For the calculation of the reference we check which corner the vehicle is heading to
            if((obj.direction > 0 && obj.slope < 0) || ((obj.direction < 0 && obj.slope > 0)))
                % lower right corner
                if((obj.direction > 0 && obj.slope < 0))
                    r = [obj.limits.x_r, obj.line(obj.limits.x_r)]';
                    if(not(obj.is_admissible(r)))
                        r = [inv_line(obj.limits.y_b), obj.limits.y_b]';
                    end
                else % lower left corner
                    r = [obj.limits.x_l, obj.line(obj.limits.x_l)]';
                    if(not(obj.is_admissible(r)))
                        r = [inv_line(obj.limits.y_b), obj.limits.y_b]';
                    end
                end
            else
                % upper left corner
                if((obj.direction < 0 && obj.slope < 0))
                    r = [obj.limits.x_l, obj.line(obj.limits.x_l)]';
                    if(not(obj.is_admissible(r)))
                        r = [inv_line(obj.r_old, obj.limits.y_t), obj.limits.y_t]';
                    end
                else % upper right corner
                    r = [obj.limits.x_r, obj.line(obj.limits.x_r)]';
                    if(not(obj.is_admissible(r)))
                        r = [inv_line(obj.r_old, obj.limits.y_t), obj.limits.y_t]';
                    end
                end
            end
            obj.border_reference = r;
        end
    end
    
    methods (Access = private)
        % Check if the reference is inside the pool
        % input:
        % r     - reference
        % output:
        % ris   - check result (true if r is inside the pool boundaries
        %         false otherwise
        function ris = is_admissible(obj, r)
            if(r(1) > obj.limits.x_r || r(1) < obj.limits.x_l)
                ris = false;
            elseif(r(2) > obj.limits.y_t || r(2) < obj.limits.y_b)
                ris = false;
            else
                ris = true;
            end
        end
    end
end

