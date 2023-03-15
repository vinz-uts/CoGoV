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


classdef (Abstract) ControlledSystem < handle
    %% CONTROLLED SYSTEM ABSTRACT CLASS
    %  Define an abstract class to model time-discrete dynamic closed-loop
    %  system with a SystemModel.
    %  z(k+1) = Φ*z(k) + G*r(k)
    %   y(k)  = Hy*z(k)
    %   c(k)  = Hc*z(k) + L*r(k)
     
    properties
        sys % SystemModel instance
        Tc % sampling time
        Fa % optimal feedback control gain
        Cy % tracking system outputs
        Phi % closed-loop state-space model Φ matrix
        G % closed-loop state-space model G matrix
        Hc % closed-loop state-space model Hc matrix
        L % closed-loop state-space model L matrix
        xc % simulation controller-states array
        r % simulation references array
        %int % DiscreteIntegrator for error integration or accumulation
        xci % controller initial conditions
    end
    
    methods (Abstract)
        sim(obj,r,T)  % simulate the system
        reset(obj,xi,xci) % reset the system
    end
    
end
