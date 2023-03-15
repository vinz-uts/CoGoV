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