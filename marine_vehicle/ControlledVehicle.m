classdef ControlledVehicle < handle
    %% CONTROLLED VEHICLE
    %  Define an abstraction of a controlled vehicle with a command
    %  governor.
    
    properties
        ctrl_sys % ControlledSystem_LQI
        cg % command governor
		g % last computed references
        color % turn color
    end
    
    
    methods
        function obj = ControlledVehicle(ctrl_sys)
            % ControlledVehicle - Constructor
            % Create an istance of a controlled vehicle.
            % >> vehicle = ControlledVehicle(ctrl_sys)
            %    Initialize a controlled vehicle with initial conditions
            obj.ctrl_sys = ctrl_sys;
        end
        
        
        function init_position(obj,x,y)
            % init_position - init the vehicle initial position
            % Set the vehicle and controller initial conditions relative to
            % a steady position [x,y].
            f = obj.ctrl_sys.Fa(:,size(obj.ctrl_sys.sys.A,1)+1:end);
            F = obj.ctrl_sys.Fa(:,1:size(obj.ctrl_sys.sys.A,1));
            obj.ctrl_sys.sys.xi = [x,y,0,0]';
            obj.ctrl_sys.xci = f\F*obj.ctrl_sys.sys.xi;
            obj.g = [x,y]';
        end
        
    end
    
end

