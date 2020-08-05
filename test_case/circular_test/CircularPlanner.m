classdef CircularPlanner
    %setenv('GRB_LICENSE_FILE', 'C:\Program\gurobi.lic');
    %% BORDER PLANNER
    % Concrete class used to compute references in a pool. When hitting the
    % boundaries of the pool the new vehicle's trajectory starts with the
    % same strike angle.
    % The angles are computed using straight lines described by the
    % following equation:
    %               y = mx + q
    % The new references are calculated as the intersection between 
    % the straight lines that define the boundaries of the pool and the 
    % straight line that characterizes the movement of the vehicle.

    
    properties
        center;
        radius;
        nstep;
        clockwise;
        x;
        y;
        counter;
        reached;
    end
    
    methods
        % Class constructor
        % input:
        % center - center of movement (1 right -1 left)
        % m         - starting line slope
        % limits    - boundaries of the pool
        % tol       - tolerance associated to the reference choice
        %             (optional)
        % output: 
        % obj       - BorderPlanner instance
        function obj = CircularPlanner(center, radius, nstep, clockwise)
            
            %clockwise (1) or (-1)
            
            obj.center = center;
            obj.radius = radius;
            obj.nstep = nstep;
            obj.clockwise = clockwise; 
            obj.counter = 1;
            obj.x = obj.center(1) + obj.radius*cos(0:nstep:2*pi);
            obj.y = obj.center(2) + obj.radius*sin(0:nstep:2*pi);
            
            obj.reached = 0.01;
            
        end
        
        % Compute a new reference
        % input:
        % sys   - veichel model
        % output:
        % r     - new reference
        function [r, obj] = compute_reference(obj, sys)
            r = [obj.x(obj.counter), obj.y(obj.counter)]';
            
            % If the simulation has just started, the initial state is used
            if(isempty(sys.x))
                p = [sys.xi(1); sys.xi(2)];
            else % use the current state
                p = [sys.x(1, end); sys.x(2, end)];
            end
            
            if(norm(p-r) < obj.reached)
                obj.counter = obj.counter + obj.clockwise; 
                
                if(obj.counter > length(obj.x))
                    obj.counter = 1 ;
                end
                if (obj.counter == 0)
                    obj.counter = length(obj.x) ;
                end
                
                r = [obj.x(obj.counter), obj.y(obj.counter)]'; 

            end
               
            
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