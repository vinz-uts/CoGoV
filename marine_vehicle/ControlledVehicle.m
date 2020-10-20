classdef ControlledVehicle < handle
    %% CONTROLLED VEHICLE
    %  Defines an abstraction of a controlled vehicle with a command
    %  governor.
    
    properties
        ctrl_sys % ControlledSystem_LQI
        cg % command governor
		g % last computed references
        color % round color
        planner % Planner of the vehicle
    end
    
    
    methods
        function obj = ControlledVehicle(ctrl_sys)
            % ControlledVehicle - Constructor
            % Create an istance of a controlled vehicle.
            % >> vehicle = ControlledVehicle(ctrl_sys)
            %    Initialize a controlled vehicle with initial conditions
            obj.ctrl_sys = ctrl_sys;
        end
        
        
        function init_position(obj,x,y,th)
            % init_position - init the vehicle initial position
            % Set vehicle and controller initial conditions relative to
            % a steady position [x,y] or [x,y,ϑ].
            f = obj.ctrl_sys.Fa(:,obj.ctrl_sys.sys.nx+1:end);
            F = obj.ctrl_sys.Fa(:,1:obj.ctrl_sys.sys.nx);
            if nargin == 3
                obj.ctrl_sys.sys.xi = [x,y,0,0]';
                obj.g = [x,y]';
            else
                obj.ctrl_sys.sys.xi = [x,y,th,0,0,0]';
                obj.g = [x,y,th]';
            end
            obj.ctrl_sys.xci = f\F*obj.ctrl_sys.sys.xi;

            
        end
        
    end
    
end

