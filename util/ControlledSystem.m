classdef (Abstract) ControlledSystem < handle
    %CONTROLLEDSYSTEM abstract class
    %   Defines an abstract class to model time-discrete dynamic closed-loop
    %   system with a SystemModel.
    %      z(k+1) =  Φ*z(k) + G*r(k)
    %      y(k)   = Hy*z(k)
    %      c(k)   = Hc*z(k) + L*r(k)
    %
    %  Authors: Vincenzo D'Angelo, Ayman El Qemmah, Franco Angelo Torchiaro
    %  Credits: Alessandro Casavola, Francesco Tedesco
    %  Copyright 2021 - CoGoV.
     
    properties
        sys % SystemModel instance
        Tc  % Sampling time
        Fa  % Feedback control gain
        Cy  % Tracking system outputs
        Phi % Closed-loop state-space model Φ matrix
        G   % Closed-loop state-space model G matrix
        Hc  % Closed-loop state-space model Hc matrix
        L   % Closed-loop state-space model L matrix
        xc  % Simulation controller-states array
        r   % Simulation references array
        %int % DiscreteIntegrator for error integration or accumulation
        xci % Controller initial conditions
    end
    
    methods (Abstract)
        sim(obj,r,T)      % Simulate the system
        reset(obj,xi,xci) % Reset the system
    end
    
end
