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


function plot_trajectory(x,y,th,N,d,color)
    %% PLOT_TRAJECTORY - plot trajectory with oriented points
    %  Plot the trajectory of [x,y,ϑ] data with sparse oriented points.
    %  N := points density
    %  d := point dimension
    %  Usage:
    %  >> plot_trajectory(x,y,th)
    %  >> plot_trajectory(x,y,th,N)
    %  >> plot_trajectory(x,y,th,N,d)
    
    N_ = 33; % points density
    d_ = 0.07; % "triangle" dimension
    if nargin > 3
        N_ = N;
    end
    if nargin > 4
        d_ = d;
    end
    
    v = d_.*[1 -1 -0.6 -1 1;0 -1 0 1 0];% base "triangle" vertices
    
    %figure();
   
    
    plot(x,y);
    
    c = ishold;

    
    
    hold on; % plot trajectory
    axis equal
    
    % plot points orientation
    for i = 1:N_:length(x)
        R = [ cos(th(i)) -sin(th(i)) ;
              sin(th(i)) cos(th(i)) ];
        p_i = [x(i),y(i)]'+R*v;
        plot(p_i(1,:),p_i(2,:),color);
    end
        
    if(c)
        hold off;
    else
        hold on;
    end

