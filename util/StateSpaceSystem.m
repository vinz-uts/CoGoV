classdef StateSpaceSystem < handle
    %% STATE SPACE SYSTEM
    %  Define a time-continuous dynamic system with a state-space model:
    %       dx = A*x + B*u
    %        y = C*x + D*u
    
    properties
        A  % state-space model A matrix
        B  % state-space model B matrix
        C  % state-space model C matrix
        D  % state-space model D matrix
        nx % state dimension
        nu % input dimension
        ny % output dimension
        t  % simulation time array
        u  % simulation inputs array
        x  % simulation states array
        y  % simulation outputs array
        xi % initial conditions
    end
    
    
    methods
        function obj = StateSpaceSystem(A,B,C,D,xi)
            % StateSpaceSystem - Constructor
            % Create an istance of a state-space system.
            % >> sys = StateSpaceSystem(A,B,C,D,xi)
            %    Initialize a state-space model with A,B,C,D matrix and xi
            %    initial conditions.
            % >> sys = StateSpaceSystem(A,B,C,D)
            %    Initialize a state-space model with A,B,C,D matrix and 0
            %    initial conditions.
            % >> sys = StateSpaceSystem(A,B)
            %    Initialize a state-space model with A,B,eye(n),0 matrix
            %    and 0 initial conditions.
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
           % reset - Reset the system
           % Reset the system at t=0 with initial conditions xi.
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

