classdef CircularPlanner
    %% CircularPlanner
    % Concrete class used to compute references in order to follow 
    % a circular trajectory

    
    properties
        center;     % center of the trajectory
        radius;     % radius of the trajectory
        nstep;      % distance in radians between each sample of the trajectory (sampling step)
        clockwise;  % if 1 the trajectory is traveled clockwise, otherwise counterclockwise
        x;          % collection of x-reference
        y;          % collection of y-reference
        counter;    % counter used to determine the current reference
        reached;    % minimum distance used to determin if the current reference has been reached or not
    end
    
    methods
        % Class constructor
        % input:
        % center    - center of the trajectory
        % radius    - radius of the trajectory
        % nspep     - sampling step
        % clockwise - if 1 the trajectory is traveled clockwise, otherwise counterclockwise
        %             (optional)
        % output: 
        % obj       - CircularPlanner instance
        function obj = CircularPlanner(center, radius, nstep, clockwise)            
            obj.center = center;
            obj.radius = radius;
            obj.nstep = nstep;
            obj.clockwise = clockwise; 
            obj.counter = 1;
            
            % generation of references by sampling a circular path 
            % (center and radius specified as input variables) at intervals of width nstep
            obj.x = obj.center(1) + obj.radius*cos(0:nstep:2*pi);
            obj.y = obj.center(2) + obj.radius*sin(0:nstep:2*pi);
            
            obj.reached = 0.01;
            
        end
        
        % Compute a new reference
        % input:
        % sys   - veichle model
        % output:
        % r     - new reference
        % obj   - modified CircularPlanner instance
        function [r, obj] = compute_reference(obj, sys)
            r = [obj.x(obj.counter), obj.y(obj.counter)]';
            
            % If the simulation has just started, the initial state is used
            if(isempty(sys.x))
                p = [sys.xi(1); sys.xi(2)];
            else % use the current state
                p = [sys.x(1, end); sys.x(2, end)];
            end
            
            % the refererence is updated if the old one was reached
            if(norm(p-r) < obj.reached)
                % counter modular update
                obj.counter = obj.counter + obj.clockwise; 
                if(obj.counter > length(obj.x))
                    obj.counter = 1 ;
                elseif(obj.counter == 0)
                    obj.counter = length(obj.x) ;
                end
                
                % reference extraction
                r = [obj.x(obj.counter), obj.y(obj.counter)]'; 

            end

        end
    end
end