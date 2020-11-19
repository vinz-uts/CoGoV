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
        pending_plugin % Indicates the index of the current plugin request
                       % -1 equals no requests
        freeze  % If setted to 1 the vehicle freezes its old reference
        parent % Indicates the parent node in the ST structure,
               % if 0 then the vehicle is the root node
    end
    
    
    methods
        function obj = ControlledVehicle(ctrl_sys)
            % ControlledVehicle - Constructor
            % Create an istance of a controlled vehicle.
            % >> vehicle = ControlledVehicle(ctrl_sys)
            %    Initialize a controlled vehicle with initial conditions
            obj.ctrl_sys = ctrl_sys;
            obj.pending_plugin = -1;
            obj.freeze = 0;
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

