classdef ControlledSystem_LQI < handle
    %% CONTROLLED SYSTEM LQI
    %  Define a time-discrete dynamic closed-loop system with a state-space model:
    %  z(k+1) = Φ*z(k) + G*r(k)
    %   y(k)  = Hy*z(k)
    %   c(k)  = Hc*z(k) + L*r(k)
     
    properties
        sys % SystemModel
        Tc % sampling time
        Fa % Optimal feedback control gain
        Cy % tracking system outputs
        Phi % closed-loop state-space model Φ matrix
        G % closed-loop state-space model G matrix
        Hc % closed-loop state-space model Hc matrix
        L % closed-loop state-space model L matrix
        xc % simulation controller-states array
        r % simulation references array
        xci % controller initial conditions
        int
    end
    
    
    methods
        function obj = ControlledSystem_LQI(sys,Tc,Fa,Cy,Phi,G,Hc,L)
            % ControlledSystem_LQI - Constructor
            % Create an istance of a controlled system with a optimal
            % control law.
            % >> ctrl_sys = ControlledSystem_LQI(sys,Tc,Fa,Cy,Phi,G,Hc,L)
            obj.sys = sys;
            obj.Tc = Tc;
            obj.Fa = Fa;
            obj.Cy = Cy;
            obj.Phi = Phi;
            obj.G = G;
            obj.Hc = Hc;
            obj.L = L;
            obj.xci = zeros(size(Fa,1),1);
            obj.int = Discrete_integrator(obj.xci, Tc, 0.5); % the integration gain must be chosen carefully 0.5 R-stab 2 LQI
        end
            
        
        function sim(obj,r,T)
            % sim - Simulate the system
            %   Simulate the system for T seconds with initial conditions
            %   xi and constant references r appling a optimal linear
            %   control law.
            
            N = ceil(T/(obj.Tc)); % simulation steps number
            e = obj.xci; % initial error value
            nt = length(obj.sys.t); % length of last simulation
            %nx = size(obj.sys.A,1);
            nx = obj.sys.nx;

            for i=1:N
                u = -obj.Fa(:,1:nx)*obj.sys.xi - obj.Fa(:,nx+1:end)*e; % - R-stab + LQI
                obj.sys.sim(u,obj.Tc); % simulate system
                
                obj.r = [obj.r r.*ones(size(obj.G,2),(length(obj.sys.t)-nt))];
                obj.xc = [obj.xc e.*ones(size(obj.G,2),(length(obj.sys.t)-nt))];
%                e = e + (r - obj.Cy*obj.sys.xi); % update incremental errors
                e = obj.int.integrate((r - obj.Cy*obj.sys.xi));
                nt = length(obj.sys.t); % update length of simulation vectors
            end
            obj.xci = e; % update controller states
        end
        
        
        function reset(obj,xi,xci)
            % reset - Reset the system
            % Reset the system at t=0 with initial conditions [xi,xci].
            if nargin < 2
                obj.sys.reset();
                if ~isempty(obj.xc)
                    obj.xci = obj.xc(:,1);
                else
                    obj.xci = zeros(size(obj.G,2),1);
                end
            else
               obj.sys.reset(xi);
               obj.xci = xci;
           end
           obj.r = [];
           obj.xc = [];
        end
        
    end
    
end
