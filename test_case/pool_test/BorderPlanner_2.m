classdef BorderPlanner_2
    %setenv('GRB_LICENSE_FILE', 'C:\Program\gurobi.lic');
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

    
    properties
        p_old;      % previous vehicle coordinates
        direction;  % direction of movement (1 right -1 left)
        m;          % line slope
        q;          % line offset
        limits;     % boundaries of the pool
        r_old;      % previous reference
        tol;        % tolerance of the reference choice
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
        function obj = BorderPlanner(direction, m, limits, tol)
            
            obj.direction = direction;
            obj.p_old = [];
            obj.r_old = [];
            obj.m = m;
            obj.q = [];
            
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
        function [r, obj] = compute_reference(obj, sys)
            
            % If the simulation has just started, the initial state is used
            if(isempty(sys.x))
                p = [sys.xi(1); sys.xi(2)];
            else % use the current state
                p = [sys.x(1, end); sys.x(2, end)];
            end
            
            % The reference is not updated if it has not been reached
            if(not(isempty(obj.r_old)) && norm(p - obj.r_old) > obj.tol)
                r = obj.r_old;
                return;
            end
            
            % Initialize the previous reference if necessary
            if(isempty(obj.r_old))
                obj.r_old = p;
            end
            
            % compute m, q and check if it is necessary to change direction
            if(length(sys.x) > 60 && not(isempty(obj.p_old)))
                % if the vehicle has reached the left or right edge, the direction of movement must be changed
                if(norm(p(1) - obj.limits.x_l) < obj.tol || norm(p(1) - obj.limits.x_r) < obj.tol)
                    m_new = - obj.m;    
                    obj.direction = - obj.direction;
                    obj.m = m_new;
                    obj.q = p(2) - obj.m*p(1);
                    
                % if the vehicle has reached the upper or lower edge, it is not necessary to change the direction of movement
                elseif(norm(p(2) - obj.limits.y_t) < obj.tol || norm(p(2) - obj.limits.y_b) < obj.tol)
                    m_new = - obj.m; 
                    obj.m = m_new;
                    obj.q = p(2) - obj.m*p(1); 
                end
            end
            
%             if((isempty(obj.q)))
%                 obj.q = p(2) - obj.m*p(1); 
%             end

            obj.p_old = p;
            
            % compute the line equation using the family of straight lines
            % concept
            rect = @(C, x) (C(2) + obj.m*(x - C(1)));
            inv_rect = @(C, y) ((y - C(2))/obj.m + C(1));
            
            % For the calculation of the reference we check which corner the vehicle is heading to
            if((obj.direction > 0 && obj.m < 0) || ((obj.direction < 0 && obj.m > 0)))
                % lower right corner
                if((obj.direction > 0 && obj.m < 0))
                    r = [obj.limits.x_r, rect(obj.r_old, obj.limits.x_r)]';
                    if(not(obj.is_admissible(r)))
                        r = [inv_rect(obj.r_old, obj.limits.y_b), obj.limits.y_b]';
                    end
                else % lower left corner
                    r = [obj.limits.x_l, rect(obj.r_old, obj.limits.x_l)]';
                    if(not(obj.is_admissible(r)))
                        r = [inv_rect(obj.r_old, obj.limits.y_b), obj.limits.y_b]';
                    end
                end
            else
                % upper left corner
                if((obj.direction < 0 && obj.m < 0))
                    r = [obj.limits.x_l, rect(obj.r_old, obj.limits.x_l)]';
                    if(not(obj.is_admissible(r)))
                        r = [inv_rect(obj.r_old, obj.limits.y_t), obj.limits.y_t]';
                    end
                else % upper right corner
                    r = [obj.limits.x_r, rect(obj.r_old, obj.limits.x_r)]';
                    if(not(obj.is_admissible(r)))
                        r = [inv_rect(obj.r_old, obj.limits.y_t), obj.limits.y_t]';
                    end
                end
            end
            
            obj.r_old = r;
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


