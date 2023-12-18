classdef ControlledSystem_LQI < ControlledSystem
    %CONTROLLEDSYSTEM_LQI class
    %   Define a time-discrete dynamic closed-loop system with a state-space model:
    %      z(k+1) =  Φ*z(k) + G*r(k)
    %      y(k)   = Hy*z(k)
    %      c(k)   = Hc*z(k) + L*r(k)
    %
    %   ctrl_sys = CONTROLLEDSYSTEM_LQI(sys,Tc,Fa,Cy,Phi,G,Hc,L) creates an istance of a
    %   controlled system with an optimal LQI control. sys is the open loop ModelSystem.
    %   Sampling time is Tc, Fa=[Fb|Ff] is the optimal control gain which include Feedback
    %   and Feedforward gains. Cy is the output matrix. Phi, G, Hc, L are cloosed loop
    %   matrices.
    %
    %  
    %  Authors: Vincenzo D'Angelo, Ayman El Qemmah, Franco Angelo Torchiaro
    %  Credits: Alessandro Casavola, Francesco Tedesco
    %  Copyright 2021 - CoGoV.
     
    properties
        %sys % SystemModel
        %Tc  % sampling time
        %Fa  % Optimal feedback control gain
        %Cy  % tracking system outputs
        %Phi % closed-loop state-space model Φ matrix
        %G   % closed-loop state-space model G matrix
        %Hc  % closed-loop state-space model Hc matrix
        %L   % closed-loop state-space model L matrix
        %xc  % simulation controller-states array
        %r   % simulation references array
        %xci % controller initial conditions
        int % Discrete integrator for error integration or accumulation
    end
    
    
    methods
        function obj = ControlledSystem_LQI(sys,Tc,Fa,Cy,Phi,G,Hc,L)
            %CONTROLLEDSYSTEM_LQI - Constructor
            obj.sys = sys;
            obj.Tc = Tc;
            obj.Fa = Fa;
            obj.Cy = Cy;
            obj.Phi = Phi;
            obj.G = G;
            obj.Hc = Hc;
            obj.L = L;
            obj.xci = zeros(size(Fa,1),1);
            obj.int = DiscreteIntegrator(obj.xci, Tc, 1); % Tc = 1 accumulation D-LQI, Tc = Tc_sys integration R stability
        end
            
        
        function u = sim(obj,r,T)
            %u = SIM(r,T) simulate the system for T seconds with initial conditions
            %   xi and constant references r appling a optimal linear control law.
            %   Results of simulation are stored in sys.t, sys.u, sys.x, sys.y, r,
            %   xc class variables.
            N = ceil(T/(obj.Tc)); % simulation steps number
            e = obj.xci; % initial error value
            obj.int.set_initial_state(e); % update integrator states
            nt = length(obj.sys.t); % length of last simulation
            %nx = size(obj.sys.A,1);
            nx = obj.sys.nx;
            for i=1:N
                u = -obj.Fa(:,1:nx)*obj.sys.xi + obj.Fa(:,nx+1:end)*e;
                obj.sys.sim(u,obj.Tc); % simulate system
                obj.r = [obj.r r.*ones(size(obj.G,2),(length(obj.sys.t)-nt))];
                obj.xc = [obj.xc e.*ones(size(obj.G,2),(length(obj.sys.t)-nt))];
                e = obj.int.integrate((r - obj.Cy*obj.sys.xi),'forward'); % update incremental errors
                nt = length(obj.sys.t); % update length of simulation vectors
            end
            obj.xci = e; % update controller states
        end
        
        
        function reset(obj,xi,xci)
            %RESET() reset the system at t=0 with initial conditions xi=0 and xci=0.
            %
            %RESET(xi,xci) reset the system at t=0 with initial conditions xi and xci.
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
