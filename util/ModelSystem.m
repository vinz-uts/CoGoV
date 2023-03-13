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