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
