classdef StateSpaceSystem < ModelSystem
    %STATESPACESYSTEM class
    %   Define a time-continuous dynamic system with a state-space model:
    %      dx = A*x + B*u
    %       y = C*x + D*u
    % 
    %   sys = StateSpaceSystem(A,B,C,D,xi) initialize a state-space model with A,B,C,D
    %   matrices and xi initial conditions.
    %
    %   sys = StateSpaceSystem(A,B,C,D) initialize a state-space model with A,B,C,D
    %   matrices and 0 initial conditions.
    %
    %   sys = StateSpaceSystem(A,B) initialize a state-space model with A,B,eye(n),0 
    %   matrices and 0 initial conditions.
    %
    %  Authors: Vincenzo D'Angelo, Ayman El Qemmah, Franco Angelo Torchiaro
    %  Credits: Alessandro Casavola, Francesco Tedesco
    %  Copyright 2021 - CoGoV.
    
    properties
        A  % State-space model A matrix
        B  % State-space model B matrix
        C  % State-space model C matrix
        D  % State-space model D matrix
        nx % State dimension
        nu % Input dimension
        ny % Output dimension
        t  % Simulation time array
        u  % Simulation inputs array
        x  % Simulation states array
        y  % Simulation outputs array
        xi % Initial conditions
    end
    
    
    methods
        function obj = StateSpaceSystem(A,B,C,D,xi)
            %StateSpaceSystem - Constructor
            if nargin >= 2
                obj.A = A;
                obj.B = B;
            end
            if nargin >= 3
                obj.C = C;
            else
                obj.C = eye(size(A,1));
            end
            if nargin >= 4
                obj.D = D;
            else
                obj.D = zeros(size(obj.C,1),size(B,2));
            end
            if nargin >= 5
                obj.xi = xi;
            else
                obj.xi = zeros(size(A,1),1);
            end
            obj.nx = size(obj.A,1);
            obj.nu = size(obj.B,2);
            obj.ny = size(obj.C,1);
        end
            
        
        function sim(obj,u,T)
            %SIM(u,T) simulate the system for T seconds with constant input u.
            %   Simulation start from the last initial condition xi and from
            %   the last time value. Results of simulation are stored in
            %   t, u, x, y class variables.  
            Ti = 0;
            if ~isempty(obj.t)
                Ti = obj.t(end);
            end
            sim_params.t = 0;
            sim_params.T = T;
            sim_params.xi = obj.xi;
            sim_params.u = u;
            sim_params.A = obj.A;
            sim_params.B = obj.B;
            sim_params.C = obj.C;
            sim_params.D = obj.D;
            assignin('base','sim_params',sim_params);
            
            sim('state_space_system');
            
            obj.t = [obj.t t_out'+Ti];
            obj.u = [obj.u u.*ones(length(u),length(t_out))];
            obj.x = [obj.x x_out'];
            obj.y = [obj.y y_out'];
            obj.xi = x_out(end,:)'; % update initial conditions
        end
        
        
        function reset(obj,xi)
            %RESET() reset the system at t=0 with initial conditions 0.
            %
            %RESET(xi) reset the system at t=0 with initial conditions xi.
            if nargin < 2
                if ~isempty(obj.x)
                    obj.xi = obj.x(:,1);
                else
                    obj.xi = zeros(obj.nx,1);
               end
            else
                obj.xi = xi;
            end
            obj.t = [];
            obj.u = [];
            obj.x = [];
            obj.y = [];
        end        
    end
end

