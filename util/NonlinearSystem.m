classdef NonlinearSystem < ModelSystem
    %NONLINEARSYSTEM class
    %   Define a time-continuous nonlinear dynamic system:
    %       dx = f(x,u)
    %        y = g(x,u)
    %
    %   sys = NONLINEARSYSTEM(f,g,nx,nu,ny,xi) initialize a nonlinear system with state-transition
    %   matlab function f, output matlab function g and xi as initial.
    %   conditions. States, Input, Output dimension: nx, nu, ny.
    %
    %   sys = NONLINEARSYSTEM(f,g,nx,nu,ny) initialize a nonlinear system with state-transition
    %   matlab function f, output matlab function g and 0 as initial conditions. States, Input,
    %   Output dimension: nx, nu, ny.
    %
    %   sys = NONLINEARSYSTEM(f,nx,nu) initialize a nonlinear system with state-transition matlab
    %   function f, output matlab function 'full_state' and 0 as initial conditions. States, 
    %   Input dimension: nx, nu (ny=nx).
    %
    %  Authors: Vincenzo D'Angelo, Ayman El Qemmah, Franco Angelo Torchiaro
    %  Credits: Alessandro Casavola, Francesco Tedesco
    %  Copyright 2021 - CoGoV.
    
    properties
        f  % State-transition matlab function
        g  % Output matlab function
        nx % States number
        nu % Inputs number
        ny % Outputs number
        t  % Simulation time array
        u  % Simulation inputs array
        x  % Simulation states array
        y  % Simulation outputs array
        xi % Initial conditions
    end
    
    
    methods
        function obj = NonlinearSystem(f,g,nx,nu,ny,xi)
            %NONLYNEARSYSTEM - Constructor
            %   Creates an istance of a nonlinear system.
            obj.f = f;
            if nargin <= 3 % shift params names nx=g, nu=nx
                obj.g = 'full_state';
                obj.nx = g;
                obj.nu = nx;
                obj.ny = obj.nx;
            end
            obj.xi = zeros(obj.nx,1);
            if nargin > 3
                obj.g = g;
                obj.nx = nx;
                obj.nu = nu;
                obj.ny = ny;
            end
            if nargin == 6
                obj.xi = xi;
            end
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
            sim_params.f = obj.f;
            sim_params.nx = obj.nx;
            sim_params.nu = obj.nu;
            sim_params.ny = obj.ny;
            assignin('base','sim_params',sim_params);
            
            sim('nonlinear_system');
            
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
                    obj.xi = zeros(obj.x,1);
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
