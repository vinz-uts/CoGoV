function plot_simulation(sys,opt)
    %% PLOT_SIMULATION - plot simulation data
    %  Plot simulation data from a state-space system or a controlled-system
    %  with a LQI control law.
    %  opt = ['x','u','e','r'] = ['states','inputs','errors','references'].
    if nargin < 2
        opt = ['x','u','e','r']; % plot all
    end
    if isa(sys,'ControlledSystem_LQI')
        ne = size(sys.Cy,1);
        % Tracking errors
        if contains(opt,'e')
            figure(3);  hold on;
            for i=1:ne
                subplot(ne,1,i);  plot(sys.sys.t,sys.xc(i,:));
            end
        end
        % References
        if contains(opt,'r')
            figure(4);  hold on;
            for i=1:ne
                subplot(ne,1,i);  plot(sys.sys.t,sys.r(i,:));
            end
            sys = sys.sys;
        end
    end
    nx = sys.nx; %nx = size(sys.A,1);
    nu = sys.nu; %nu = size(sys.B,2);
    % States plots
    if contains(opt,'x')
        figure(1);  hold on;
        for i=1:nx
            subplot(nx,1,i);  plot(sys.t,sys.x(i,:));
        end
    end
    % Inputs plots
    if contains(opt,'u')
        figure(2);  hold on;
        for i=1:nu
            subplot(nu,1,i);  plot(sys.t,sys.u(i,:));
        end
    end    
end
