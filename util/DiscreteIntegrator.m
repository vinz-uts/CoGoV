% Copyright 2021 - CoGoV.
% Licensed under the Academic Free License, Version 3.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
% https://opensource.org/license/afl-3-0-php/
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
% 
% Authors: Vincenzo D'Angelo, Ayman El Qemmah, Franco Angelo Torchiaro
% Credits: Alessandro Casavola, Francesco Tedesco


classdef DiscreteIntegrator < handle
    %% DISCRETE INTEGRATOR CLASS
    % Concrete class used to perform discrete integration.
    % The following integration method are supported:
    %   - Forward Euler
    %   - Backward Euler
    %   - Trapezoidal
    % In the integrator function use the following values of the method
    % input parameter in order to choose and integration method:
    %   - method = Forward Euler; for Forward Euler Method integration
    %   - method = Backward Euler; for Forward Euler Backward integration
    %   - method = Trapezoidal; for Forward Trapezoidal Method integration
    % If no method is specified or is not known, the default one is used (Backward Euler).
    
    
    properties
        x_next      % x(n+1) integrator next state
        x_curr      % x(n) integrator current state
        Tc          % sampling time
        K           % integrator gain
    end
    
    
    methods
        
        function obj = DiscreteIntegrator(x0, Tc, K)
            % Class constructor
            % input:
            % x0        - initial state
            % Tc        - sampling time; if Tc = 1 the integrator is in
            %             accumulation mode
            % K         - gain
            % output:
            % obj       - Discrete Integrator instance      
            if(not(iscolumn(x0)))
                x0 = x0';
            end
            
            obj.Tc = Tc;
            obj.x_curr = x0;
            obj.x_next = x0;
            obj.K = K;
        end
       
        
        function obj = set_initial_state(obj, xin)
            % Set the initial state condition
            % input:
            % xin       - initial values
            if(not(iscolumn(xin)))
                xin = xin';
            end
            
            obj.x_curr = xin;
            obj.x_next = xin;
        end
        
        
        function y = integrate(obj, u, method)
            % Compute the integration operation according to a specific method
            % input:
            % u         - input value
            % method    - integration method
            % output:
            % y     - integrator output value
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

