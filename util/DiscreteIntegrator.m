classdef DiscreteIntegrator < handle
    %DISCRETEINTEGRATOR class
    %   Concrete class used to perform discrete integration.
    %   The following integration method are supported:
    %      * Forward Euler
    %      * Backward Euler
    %      * Trapezoidal
    %   In the integrator function use the following values of the method
    %   input parameter in order to choose and integration method:
    %      * method = Forward Euler; for Forward Euler Method integration
    %      * method = Backward Euler; for Forward Euler Backward integration
    %      * method = Trapezoidal; for Forward Trapezoidal Method integration
    %   If no method is specified or is not known, the default one is used (Backward Euler).
    %   
    %   int = DiscreteIntegrator(x0, Tc, K) create a discrete time integrator with x0
    %   initial state, sampling time Tc and integrator gain K.
    %
    %
    %  Authors: Vincenzo D'Angelo, Ayman El Qemmah, Franco Angelo Torchiaro
    %  Credits: Alessandro Casavola, Francesco Tedesco
    %  Copyright 2021 - CoGoV.
    
    properties
        x_next      % x(n+1) integrator next state
        x_curr      % x(n) integrator current state
        Tc          % sampling time
        K           % integrator gain
    end
    
    
    methods
        
        function obj = DiscreteIntegrator(x0, Tc, K)
            %DISCRETEINTEGRATOR - Constructor      
            if(not(iscolumn(x0)))
                x0 = x0';
            end
            obj.Tc = Tc;
            obj.x_curr = x0;
            obj.x_next = x0;
            obj.K = K;
        end
       
        
        function obj = set_initial_state(obj, xin)
            %SET_INITIAL_STATE(xin) set the integrator initial state condition at xin.
            if(not(iscolumn(xin)))
                xin = xin';
            end
            obj.x_curr = xin;
            obj.x_next = xin;
        end
        
        
        function y = integrate(obj, u, method)
            %y = INTEGRATE(u) compute the 'Forward' integration operation for the fixed value u for Tc.
            %
            %
            %y = INTEGRATE(u, method) compute the integration operation for the fixed value u for Tc,
            %   according with the chosen method.
            if(nargin < 3)
                method = 'Forward';
            end
            if(not(iscolumn(u)))
                u = u';
            end
            % integration
            if(strcmpi(method, 'Forward'))
                % Step n:          y(n)   = x(n)
                %                  x(n+1) = x(n) + K*T*u(n)
                obj.x_next = obj.x_curr + obj.K*obj.Tc*u;
                y   = obj.x_next;
            elseif(strcmpi(method, 'Trapezoidal'))
                % Step n:          y(n)   = x(n) + K*T/2*u(n)
                %                  x(n+1) = y(n) + K*T/2*u(n)
                y   = obj.x_curr + obj.K*obj.Tc/2*u;
                obj.x_next = y + obj.K*obj.Tc/2*u;
            else
                if(not(strcmpi(method, 'Backward')))
                    warning('Attention no string match for integration method. Backward used');
                end         
                 %Step n:          y(n)   = x(n) + K*T*u(n)
                 %                 x(n+1) = y(n
                 y   = obj.x_curr + obj.K*obj.Tc*u;
                 obj.x_next = y;
            end
            % integrator state update
            obj.x_curr = obj.x_next;
        end     

    end
    
end

