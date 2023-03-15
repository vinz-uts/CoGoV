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


classdef (Abstract) ModelSystem < handle
    %% MODEL SYSTEM ABSTRACT CLASS
    %  Define an abstract class to model a time-continuous LTI/NTI system:
    %       dx = f(x,u)
    %        y = g(x,u)
    
    properties (Abstract)
        nx % states number
        nu % inputs number
        ny % outputs number
        t  % simulation time array
        u  % simulation inputs array
        x  % simulation states array
        y  % simulation outputs array
        xi % initial conditions
    end
    
    methods (Abstract)
        sim(obj,u,T)  % simulate the system
        reset(obj,xi) % reset the system
    end
    
end
