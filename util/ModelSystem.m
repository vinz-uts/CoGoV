classdef (Abstract) ModelSystem < handle
    %%MODELSYSTEM abstract class
    %  Define an abstract class to model a time-continuous LTI/NTI system:
    %    dx = f(x,u)
    %     y = g(x,u)
    %
    % Authors: Vincenzo D'Angelo, Ayman El Qemmah, Franco Angelo Torchiaro
    % Credits: Alessandro Casavola, Francesco Tedesco
    % Copyright 2021 - CoGoV.

    properties (Abstract)
        nx % Number of states
        nu % Number of inputs
        ny % Number of outputs
        t  % Simulation time array
        u  % Simulation inputs array
        x  % Simulation states array
        y  % Simulation outputs array
        xi % Initial conditions
    end
    
    methods (Abstract)
        sim(obj,u,T)  % Simulate the system
        reset(obj,xi) % Reset the system
    end
    
end
