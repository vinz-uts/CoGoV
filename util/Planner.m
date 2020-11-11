classdef (Abstract) Planner < handle
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        p_old
        r_old
        
        is_in_recovery
        counter
        rec_tolerance
        standstill
        tol
        recovery_from_collision
    end
    
    methods (Abstract)
        r = initialize_old_reference(obj, p);
        [r, theta] = compute_referecence_when_not_reached(obj, p);
        [r, theta] = compute_standard_reference(obj, p)
        res = reference_reached(obj, p);
    end
    
    methods
        function obj = Planner(varargin)
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
%             obj.delta_rho = 0;
            obj.counter  = 0;
%             obj.rho_step = 0.011;
            obj.rec_tolerance = 0.001;
            %%%%%%%%%%%%
            
             %%%%%%% Variable arguments management %%%%%%%
            validnames = {'recovery', 'rec_tolerance', 'rec_from_collision', 'rho_step'};
            
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
                end
            end   
            
        end
        
       function [r, theta] = compute_reference(obj, sys, xa)
            % Function used to compute a reference
            % sys - ControlledSystem
            % xa  - Augmented state
            
            % Extract current state
            p = sys.ctrl_sys.sys.xi(1:2);
            if(isempty(obj.p_old))
                obj.p_old = p;
            end
            % Initialize the previous reference if necessary
            if(isempty(obj.r_old))
                [obj.r_old] = obj.initialize_old_reference(p);
            end       
            
            % The reference is not updated if the current one has not been reached
            % If the vehicle does not reach the reference for too long a
            % recovery procedure is activated
            if(obj.is_in_recovery)
                r = obj.r_old;
                if(obj.exit_from_recovery(p, r))
                    obj.is_in_recovery = false;
                    obj.counter = 0;
%                     obj.delta_rho = 0;
                end
           
            elseif(not(obj.reference_reached(p)) && not(obj.need_recovery()))
                %%% Reference does not change
                [r, theta] = obj.compute_referecence_when_not_reached(p);
                %%%%
                
                if(obj.need_to_count_for_recovery(p)) %%% Increase counter for recovery management
                    obj.counter = obj.counter + 1; 
                end
                
            elseif(obj.need_recovery())
                
                if(obj.recovery_from_collision) % anticollision recovery
                    [r, theta] = obj.compute_anticollision_recovery(sys, xa);
                    obj.is_in_recovery = true; 
                    
                else % alternative recovery
                    [r, theta] = obj.compute_alternative_recovery(sys, xa);
                end
                    obj.counter = 0;
            else % update the reference normally
                [r, theta] = compute_standard_reference(obj, p);
                
                % reset interanl state for recovery management
                obj.counter = 0;
            end
            % save state
            obj.r_old = r;
            obj.p_old = p;
       end
       %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
       
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
        
       function r_recovery = compute_delta_rho_recovery(obj, delta_rho)
           theta_v = obj.xy2polar(obj.r_old(1), obj.r_old(2));
           theta_ref = theta_v;
           rho_ref = obj.evaluate(theta_ref);
           r_recovery = zeros(2, 1);
           [r_recovery(1), r_recovery(2)] = obj.polar2xy(rho_ref + delta_rho, theta_ref);
       end
       
       function res = is_recovery_enabled(obj)
           % Check if recovery is active
           res = not(obj.standstill == -1);
       end
       
       function [r, theta] = compute_alternative_recovery(obj, sys, xa)
           error('Not implemented');
       end
       %%%%%%%%%%%%%%%%%%%%%%%%%%
       
    end
end

