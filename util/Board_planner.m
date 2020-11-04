classdef Board_planner < handle
    
    properties 
        points   % A vector containing all points that have to be reached
        point_iterator % Index of curret point to be reached
        recovery_tol % A minimum number of calls to activate recovery procedure
        p_old       % Old position
        r_old       % Old reference
        tol         % Tolerance of distance
        is_recovery_active % Indicates if the recovery procedure is active
        counter % Counts how many times the vehicle stayed still 
    end
    
    methods 
        
     function obj = Board_planner(points, recovery_tol)
            obj.points = points; 
            obj.recovery_tol = recovery_tol; 
            obj.point_iterator = 1;
            obj.tol = 0.1;
            obj.counter = 0;
            obj.is_recovery_active = false; 
     end
     
     function r = compute_reference(obj, sys, xa)
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
                obj.r_old = obj.points(obj.point_iterator:obj.point_iterator+1)';
            end       

            % The reference is not updated if the current one has not been reached
            % If tehe vehicle does not reach the reference for too long a
            % recovery procedure is activated
            
            if(obj.is_recovery_active)
                r = obj.r_old;
                if(norm(p(1) - r(1)) < obj.tol || norm(p(2) - r(2)) < obj.tol )
                    obj.is_recovery_active = false;
                    obj.counter=0;
                    r = obj.points(obj.point_iterator:obj.point_iterator+1)';
                end
           
            elseif(norm(p - obj.r_old) >= obj.tol && (obj.counter < obj.recovery_tol) ) 
                %%% Reference does not change
                r = obj.r_old;
                %%%%
                
                %%% Increase counter for recovery management
                if(norm(p - obj.p_old) < 0.005)
                    obj.counter = obj.counter + 1; 
                end

            elseif(obj.counter >= obj.recovery_tol)
                
                obj.is_recovery_active = true;
                r = compute_recovery(obj, sys, xa);
                obj.counter = 0;
                
            else % update the reference normally
                
                obj.point_iterator = obj.point_iterator + 2;
                
                if(obj.point_iterator >= length(obj.points))
                    obj.point_iterator = 1;
                end
                r = obj.points(obj.point_iterator:obj.point_iterator+1)';
                obj.counter = 0;
                
            end
            % save state
            obj.r_old = r;
            obj.p_old = p;
     end
        
     function r_recovery = compute_recovery(obj, sys, xa)
            
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
            
            
        end
     
    end
end