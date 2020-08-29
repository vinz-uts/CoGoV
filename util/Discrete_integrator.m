classdef Discrete_integrator < handle
    %% Discrete Integrator
    % Concrete class used to perform discrete integration
    % The following integration method are supported
    % ---------- Forward Euler
    % ---------- Backward Euler
    % ---------- Trapezoidal
    % In the integrator function use the following values of the method
    % input parameter in order to choose and integration method:
    % - method = Forward Euler; for Forward Euler Method integration
    % - method = Backward Euler; for Forward Euler Backward integration
    % - method = Trapezoidal; for Forward Trapezoidal Method integration
    % If no method is specified, the default one is used (Backward Euler)
    % If the method specified is not known, the default one is used (Backward Euler)
    
    
    properties
        x_next      % x(n+1) integrator next state
        x_corr      % x(n) integrator current state
        Tc          % sampling time
        K           % integrator gain
    end
    
    methods
        % Class constructor
        % input:
        % x0        - initial state
        % Tc        - sampling time
        % K         - gain
        % output: 
        % obj       - Discrete Integrator instance
        function obj = Discrete_integrator(x0, Tc, K)
            obj.Tc = Tc;
            obj.x_corr = x0;
            obj.x_next = x0;
            obj.K = K;
        end
        
        % Compute the integration operation according to a specific method
        % input:
        % u         - input value
        % method    - integration method
        % output:
        % y     - integrator output value
        function y = integrate(obj, u, method)
            
            if(nargin < 3)
                method = 'Forward';
            end
            
            % integration
            if(strcmpi(method, 'Forward'))
                % Step n:          y(n)   = x(n)
                %                  x(n+1) = x(n) + K*T*u(n)
                
                obj.x_next = obj.x_corr + obj.K*obj.Tc*u;
                y   = obj.x_next;
            elseif(strcmpi(method, 'Trapezoidal'))
                % Step n:          y(n)   = x(n) + K*T/2*u(n)
                %                  x(n+1) = y(n) + K*T/2*u(n)
                
                y   = obj.x_corr + obj.K*obj.Tc/2*u;
                obj.x_next = y + obj.K*obj.Tc/2*u;
            else
                if(not(strcmpi(method, 'Backward')))
                    warning('Attention no string match for integration method. Backward used');
                end         
                 %Step n:          y(n)   = x(n) + K*T*u(n)
                 %                 x(n+1) = y(n
                 
                 y   = obj.x_corr + obj.K*obj.Tc*u;
                 obj.x_next = y;
            end
            
            % integrator state update
            obj.x_corr = obj.x_next;
        end
    end
end

