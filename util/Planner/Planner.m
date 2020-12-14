classdef (Abstract) Planner < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        p_old                       % Old position of vehicle 
        r_old                       % Old reference
        radius                      % Radius for intermediate reference 
        %%% recovery properties
        is_in_recovery              % If the recovery reference has been reached or not
        counter                     % How much time the previous refence is still the same 
        rec_tolerance               % How many times you tolerate to stay still
        standstill                  % Dynamic value of old position freezing
        tol                         % Tolerance ([m]) used to understand if the vehicle has reached the reference
        recovery_from_collision     % To differentiate between Circular recovery or a custom one
        %%%
    end
    
    methods (Abstract)
        % Abstract methods that a planner must implement
        
        % Used to specify an initialization policy
        r = initialize_old_reference(obj, p);
        
        
        % Used to specify the actions to be performed when the reference has not yet been reached
        [r, theta] = compute_referecence_when_not_reached(obj, p);
        
        % Used to specify the actions to be performed to calculate a reference under normal use conditions
        [r, theta] = compute_standard_reference(obj, p)
        
        % Method used to understand if a reference has been reached
        res = reference_reached(obj, p);
    end
    
    methods 
        function obj = Planner(varargin)
            % Variable arguments
            % recovery  - if specified activate the recovery procedure
            %             if the vehicle does not move and requests 
            %             a new reference for standstill times, then the
            %             procedure is started.
            % rec_tolerance - tolerance used to understand if the vehicle
            %                 is stuck (meters)
            % rec_from_collision - used to choose between 2 different
            %                      recovery methods
            %                      (true recovery with anticollision
            %                      circular trajectory)
            %                      (false recovery with higher references)
            
            %%%%% Default Values %%%%%
            obj.standstill = -1;
            obj.tol = 0.1;
            obj.counter  = 0;
            obj.rec_tolerance = 0.001;
            obj.radius = 0.5; 
            %%%%%%%%%%%%
            
            %%%%%%% Variable arguments management %%%%%%%
            validnames = {'recovery', 'rec_tolerance', 'rec_from_collision','radius'};
            
            nargs = length(varargin);
            params = varargin(1:2:nargs);   values = varargin(2:2:nargs);
            
            for name = params
                pos = strcmp(name{:}, params);
                switch name{:}
                     case validnames{1}
                         obj.standstill =  values{pos};
                    case validnames{2}
                        obj.rec_tolerance =  values{pos};
                    case validnames{3}
                        obj.recovery_from_collision =  values{pos};
                    case validnames{4}
                        obj.radius = values{pos};
                end
            end 
            %%%%%%%%%%%%
            
        end
        
       function [r, theta] = compute_reference(obj, sys, xa)
            % Function used to compute a reference
            % sys - ControlledSystem
            % xa  - Augmented state
            
            if(not(iscolumn(xa)))
                xa = xa';
            end
            % Extract current state
            p = sys.ctrl_sys.sys.xi(1:2);
            if(isempty(obj.p_old))
                obj.p_old = p;
            end
            
            % Initialize the previous reference if necessary
            if(isempty(obj.r_old))
                [r_] = obj.initialize_old_reference(p);
                if(not(iscolumn(r_)))
                    obj.r_old = r_';
                else
                     obj.r_old = r_;
                end
            end       
            
            % The reference is not updated if the current one has not been reached
            % If the vehicle does not reach the reference for too long a
            % recovery procedure is activated
            if(obj.is_in_recovery) % planner in recovery mode (recovery reference not reached)
                r = obj.r_old;
                if(obj.exit_from_recovery(p, r))
                    obj.is_in_recovery = false;
                    obj.counter = 0;
                end
           
            elseif(not(obj.reference_reached(p)) && not(obj.need_recovery())) % standard reference not reached 
                %%% Reference does not change
                [r, theta] = obj.compute_referecence_when_not_reached(p);
                %%%%
                
                if(obj.need_to_count_for_recovery(p)) %%% Increase counter for recovery management
                    obj.counter = obj.counter + 1; 
                end
                
            elseif(obj.need_recovery()) % If the vehicle is stuck for whatever reason, start a recovery procedure
                
                if(obj.recovery_from_collision) % anticollision recovery
                    [r, theta] = obj.compute_anticollision_recovery(sys, xa);
                    obj.is_in_recovery = true;
                    
                else % alternative recovery
                    [r, theta] = obj.compute_alternative_recovery(sys, xa);
                end
                
                obj.counter = 0; % reset the recovery counter
            else % standart behavior, update the reference normally
                [r, theta] = compute_standard_reference(obj, p);
                
                % reset interanl state for recovery management
                obj.counter = 0;
            end
                        % save state
            if(not(iscolumn(r)))
                r = r';
            end
            
            if(norm(p - r) > obj.radius)
               r = inner_reference(obj, p, r);
            end
             
                        % save state
            if(not(iscolumn(r)))
                r = r';
            end

            
            
            obj.r_old = r;
            obj.p_old = p;
       end
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       
         function ris = inner_reference(obj, p, r) % inherited abstract method
            if(r(1) - p(1) > 0)
                direction = 1;
            else
                direction = -1;
            end
            
            if((r(1) - p(1)) == 0)
                slope = inf;
                intercept = p(1);
                if(r(2) - p(2) < 0)
                    direction = 1;
                else
                    direction = -1;
                end
            else
                slope = (r(2) - p(2))/(r(1) - p(1));
                intercept = -slope*p(1) + p(2);
            end
            
            [xout,yout] = linecirc(slope, intercept,p(1), p(2), obj.radius);
            % Choose the right poit to use as an intermediate reference
            if(slope == inf)
                if(direction < 0)
                    if(yout(1) > yout(2))
                        ris(1) = xout(1);
                        ris(2) = yout(1);
                    else
                        ris(1) = xout(2);
                        ris(2) = yout(2);
                    end
                else
                    if(yout(1) < yout(2))
                        ris(1) = xout(1);
                        ris(2) = yout(1);
                    else
                        ris(1) = xout(2);
                        ris(2) = yout(2);
                    end
                end
            else
                if(direction < 0)
                    if(xout(1) < xout(2))
                        ris(1) = xout(1);
                        ris(2) = yout(1);
                    else
                        ris(1) = xout(2);
                        ris(2) = yout(2);
                    end
                else
                    if(xout(1) > xout(2))
                        ris(1) = xout(1);
                        ris(2) = yout(1);
                    else
                        ris(1) = xout(2);
                        ris(2) = yout(2);
                    end
                end
            end
        end
        
       %%%%%%%%%%%%% Recovery methods %%%%%%%%%%%%%%%%%%%%%
       function [r_recovery, theta] = compute_anticollision_recovery(obj, sys, xa)
            % Extract current state
            p = sys.ctrl_sys.sys.xi(1:2);
            
            %%% Number of Neighboors
            num_neig = length(xa)/(length(sys.ctrl_sys.sys.xi) + length(sys.ctrl_sys.xci)) -1;
            
            %%% Find nearest neighboor
            closest_neig = 1;
            min_dist = 500;
            
            for i=1:num_neig
                pos = (i)*(length(sys.ctrl_sys.sys.xi) + 2) + 1;
                if(norm(p-xa(pos:pos+1)) < min_dist)
                    min_dist= norm(p-xa(pos:pos+1));
                    closest_neig = pos;
                end
            end
            
            %%% Coordinates extraction
            p_neig = xa(closest_neig:closest_neig + 1);
            
            %%%%% Computation of the recovery referrence
            p_middle = [(p(1)+p_neig(1))/2;(p(2)+p_neig(2))/2];
            
            rho_tmp = norm(p-p_middle)*2;
            
            theta_tmp = atan2(p(2) - p_middle(2), p(1) - p_middle(1));
            if(theta_tmp < 0)
                theta_tmp = 2*pi + theta_tmp;
            end
%             
            r_x_pos = p_middle(1)+rho_tmp*cos(theta_tmp+(pi/2+0.069));
            r_y_pos = p_middle(2)+rho_tmp*sin(theta_tmp+(pi/2+0.069));
            r_tmp_pos = [r_x_pos;r_y_pos];
            
            r_x_neg = p_middle(1)+rho_tmp*cos(theta_tmp-(pi/2+0.069));
            r_y_neg = p_middle(2)+rho_tmp*sin(theta_tmp-(pi/2+0.069));
            r_tmp_neg = [r_x_neg;r_y_neg];   
            
%             if(norm(obj.r_old-[r_x_pos;r_y_pos])< norm(obj.r_old-[r_x_neg;r_y_neg]))
%                 r_recovery = [r_x_pos;r_y_pos];
%             else
%                 r_recovery = [r_x_neg;r_y_neg];
%             end
            
            if(norm(p(2)-obj.r_old(2)) > norm(p(1)-obj.r_old(1))) % I prefer vertical minim.
                if(norm(p(2)-r_y_pos)>norm(p(2)-r_y_neg))
                     r_recovery = r_tmp_pos;
                else
                    r_recovery = r_tmp_neg;
                end
            else % I prefer orizontal mini.
                if(norm(p(1)-r_x_pos)>norm(p(1)-r_x_neg))
                    r_recovery = r_tmp_pos;
                else
                    r_recovery = r_tmp_neg;
                end
            end
            
            theta = atan2(r_recovery(2) - p(2), r_recovery(1) - p(1));
       end
       
       function   res = exit_from_recovery(obj, p, r)
           res = norm(p(1) - r(1)) < obj.tol || norm(p(2) - r(2))< obj.tol;
       end
       
       function   res = need_recovery(obj)
           res = obj.is_recovery_enabled() && obj.counter >= obj.standstill;
       end
       
       function   res = need_to_count_for_recovery(obj, p)
           res = obj.is_recovery_enabled() && norm(p - obj.p_old) < obj.rec_tolerance;
       end

       function res = is_recovery_enabled(obj)
           % Check if recovery is active
           res = not(obj.standstill == -1);
       end
       
       function [r, theta] = compute_alternative_recovery(~, ~, ~)
           % If needed it can be implemented by other classes
           r = [];
           theta = [];
           length(r);
           length(theta);
           error('Not implemented');
       end
       
       
       %%%%%%%%%%%%%%%%%%%%%%%%%%
       
    end
end

