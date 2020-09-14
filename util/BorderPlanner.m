classdef BorderPlanner < handle
    %% BORDER PLANNER 2
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
        p_old;      % previous vehicle coordinates
        step;       % distance along x-axis used to compute the next reference
        m;          % line slope
        limits;     % boundaries of the pool
        r_old;      % previous reference
        tol;        % tolerance used during the reference choice
        offsetline; % offset of the line (q parameter in line equation)
    end
    
    methods
        % Class constructor
        % input:
        % direction - direction of movement (1 right -1 left)
        % m         - starting line slope
        % limits    - boundaries of the pool
        % tol       - tolerance associated to the reference choice
        %             (optional)
        % output: 
        % obj       - BorderPlanner instance
        function obj = BorderPlanner(step, m, limits, tol)
            obj.step = step;
            obj.p_old = [];
            obj.r_old = [];
            obj.m = m;
            obj.offsetline = [];
            
            if(nargin <= 3)  
                obj.tol = 0.01;
            else
                obj.tol = tol;
            end
            
            obj.limits = struct('x_l', limits(1), 'x_r', limits(2), 'y_t', limits(3), 'y_b', limits(4));
        end
        
        % Compute a new reference
        % input:
        % sys   - veichel model
        % output:
        % r     - new reference
        function [r, theta] = compute_reference(obj, sys)
            
            % If the simulation has just started, the initial state is used
            if(isempty(sys.x))
                p = [sys.xi(1); sys.xi(2)];
            else % use the current state
                p = [sys.x(1, end); sys.x(2, end)];
            end
            
            % The reference is not updated if the current one has not been reached
            if(not(isempty(obj.r_old)) && norm(p - obj.r_old) > obj.tol)
                r = obj.r_old;
                theta = atan2(r(2)-p(2),r(1)-p(1));
                return;
            end
            
            % Initialize the previous reference if necessary
            if(isempty(obj.r_old))
                
                obj.r_old = p;
            end
            
            % compute m, q and check if it is necessary to change direction
            if(length(sys.x) > 60 && not(isempty(obj.p_old)))
                if(norm(p(1) - obj.limits.x_l) < obj.tol || norm(p(1) - obj.limits.x_r) < obj.tol)
                    m_new = - obj.m;    
                    obj.step = - obj.step;
                    obj.m = m_new;
                    obj.offsetline = p(2) - obj.m*p(1); 
                % if the vehicle has reached the upper or lower edge, it is not necessary to change the direction of movement
                elseif(norm(p(2) - obj.limits.y_t) < obj.tol || norm(p(2) - obj.limits.y_b) < obj.tol)
                    m_new = - obj.m; 
                    obj.m = m_new;
                    obj.offsetline = p(2) - obj.m*p(1); 
                end
            end
            
            % if necessary initialize the offline set parameter
            if((isempty(obj.offsetline)))
                obj.offsetline = p(2) - obj.m*p(1); 
            end
            
            obj.p_old = p;          
             
            % compute the line equation using the family of straight lines
            % centered on the robot location and with the slope previously
            % computed
            rect = @(C, x) (C(2) + obj.m*(x - C(1)));
            
            % compute the new reference using the line previously computed
            r = [p(1) + obj.step, rect(obj.r_old, p(1) + obj.step)]';
            
            
            % Modify the reference so that it is admissible 
            % (it must not be outside the pool)
            if(r(1) > obj.limits.x_r)
                r(1) = obj.limits.x_r ;
                r(2) = rect(obj.r_old,r(1));
            elseif(r(1) < obj.limits.x_l)
                r(1) = obj.limits.x_l ;
                r(2) = rect(obj.r_old,r(1));
            elseif(r(2) < obj.limits.y_b)
                r(2) = obj.limits.y_b ;
            elseif(r(2) > obj.limits.y_t)
                r(2) = obj.limits.y_t ;
            end
            
            % save the reference
            obj.r_old = r;
            theta = atan2(r(2)-p(2),r(1)-p(1));
            
            
        end
    end
    

end


