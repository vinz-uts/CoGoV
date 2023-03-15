classdef NonlinearSystem < ModelSystem
    %% NONLINEAR SYSTEM CLASS
    %  Define a time-continuous nonlinear dynamic system:
    %       dx = f(x,u)
    %        y = g(x,u)
    
    properties
        f  % state-transition matlab function
        g  % output matlab function
        nx % states number
        nu % inputs number
        ny % outputs number
        t  % simulation time array
        u  % simulation inputs array
        x  % simulation states array
        y  % simulation outputs array
        xi % initial conditions
    end
    
    
    methods
        function obj = NonlinearSystem(f,g,nx,nu,ny,xi)
            % NonlinearSystem - Constructor
            % Create an istance of a nonlinear system.
            % >> sys = NonlinearSystem(f,g,nx,nu,ny,xi)
            %    Initialize a nonlinear system with state-transition matlab
            %    function f, output matlab function g and xi as initial
            %    conditions. States, Input, Output dimension: nx, nu, ny.
            % >> sys = NonlinearSystem(f,g,nx,nu,ny)
            %    Initialize a nonlinear system with state-transition matlab
            %    function f, output matlab function g and 0 as initial
            %    conditions. States, Input, Output dimension: nx, nu, ny.
            % >> sys = NonlinearSystem(f,nx,nu)
            %    Initialize a nonlinear system with state-transition matlab
            %    function f, output matlab function 'full_state' and 0 as
            %    initial conditions. States, Input dimension: nx, nu
            %    (ny=nx).
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
            % sim - Simulate the system
            %   Simulate the system for T seconds with initial conditions xi and
            %   constant input u.
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
            % reset - Reset the system
            % Reset the system at t=0 with initial conditions xi.
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
