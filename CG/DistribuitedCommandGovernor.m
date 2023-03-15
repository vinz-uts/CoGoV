classdef DistribuitedCommandGovernor < CommandGovernor
    %% DISTRIBUITED COMMAND GOVERNOR
    %  Distribuite Command Governor for multi-agents nets. Computes
    %  the nearest reference g to r that statify local and global (with
    %  other system) constraints.
    
    properties
        U % proximity constraints matrix - matrix for OR-ed constraints
        hi % proximity constraints vector - vector for OR-ed constraints
        alpha_t
        old_g
    end
    
    
    methods
        function obj = DistribuitedCommandGovernor(Phi,G,Hc,L,T,gi,U,hi,Psi,k0,solver)
            % DistribuitedCommandGovernor - Constructor
            % Create an instance of a Distribuited Command Governor.
            obj = obj@CommandGovernor(Phi,G,Hc,L,T,gi,Psi,k0);
            if nargin > 10 
                obj.check_solver(solver);
            end
            obj.U = U;
            obj.hi = hi;
            obj.alpha_t = 0;
        end
        
        
        function [g, ris, hype, alpha] = compute_cmd(obj,x,r,g_n,cloud_points,hyp, path_x, path_y, alpha_neig)
            % compute_cmd - calculate the reference g.
            % Calculate the nearest reference g to r start from initial
            % global conditions x and g_n reference for the other systems.
            % if hyp is not passed, they the hyperplane computed
            % takes into account only the vehicle position 
            % if hyp is passed but empty, then the hyperplane computed
            % takes into account also the neighbors position 
            % if hyp is passed and not empty, then the hyperplane is 
            % not computed and taken into account into the contraints
            
            % An hyperplane is represented as ay*x = by, where x represent
            % the cartesian coordinates, ay represents the normal and by
            % represents the shift 
        
            if(nargin > 6)
                max_delta = 0.5;
                g = [];
                ris = [];
                for delta_t = max_delta:-0.001:0
%                     p = [path_x(obj.alpha_t + delta_t), path_y(obj.alpha_t + delta_t)]';
                    p = [ppval(obj.alpha_t + delta_t, path_x), ppval(obj.alpha_t + delta_t, path_y)]';
                    p_x = x(1:2);
%                     if(delta_t < 0.3)
%                         delta_t
%                     end
                    if(norm(p - p_x) > 0.01)
                        continue;
                    end
                ok = 0;
                for alpha = alpha_neig
                    if((obj.alpha_t + delta_t) - alpha > 0.02)
                        break;
                    end
                        ok = 1;
                end
                if(ok == 0)
                    continue
                end
                    delta_t;
                    obj.alpha_t = obj.alpha_t + delta_t;
                    g = p;
                    break;
                end
%                 delta_t = sdpvar(1,1);
%                 cnstr = [];
%                 p = [ppval(obj.alpha_t + delta_t, path_x), ppval(obj.alpha_t + delta_t, path_y)]';
%                 p_x = x(1:2);
%                 cnstr = [cnstr norm(p - p_x) <= 0.1];
%                 cnstr = [cnstr obj.alpha_t + delta_t <= Tf];
%                 for alpha = alpha_neig
%                       cnstr = [cnstr (obj.alpha_t + delta_t) - alpha <= 0.5];
%                 end
%                 obj_fun = -delta_t;
% 
%                 options = sdpsettings('verbose', 1, 'solver', 'BMIBNB', 'usex0', 0, 'cachesolvers', 1);
%                 
%                 ris = optimize(cnstr,obj_fun,options);
%                 
%                 obj.alpha_t = obj.alpha_t + double(delta_t);
%                 delta_t = double(delta_t);
%                 g = [path_x(obj.alpha_t + delta_t), path_y(obj.alpha_t + delta_t)]';
%                 delta_t = 0.013;
%                 obj.alpha_t = obj.alpha_t + delta_t;
%                 p = [ppval(obj.alpha_t + delta_t, path_x), ppval(obj.alpha_t + delta_t, path_y)]';
%                 g = p;
%                 
                alpha = obj.alpha_t;
                hype = [];
                
                return;
                
            end
            
            g = sdpvar(length(r),1);
            w = [g;g_n];
            b = binvar(size(obj.U,1)*obj.k0,1);
            d = binvar(size(obj.U,1),1);
            mu = 10000;
            hype = [];
            
            
            %%% LOCAL AND PROXIMITY STEADY STATE CONSTRAINTS  
            cnstr = obj.T*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) <= obj.gi;
            %%% Input analysis in order to consider Obstacle Avoidance
            
            % STEADY STATE OBSTACLE AVOIDANCE CONSTRAINTS 
            if(nargin > 4 && nargin < 6)  % Obstacle avoidance           
                %%% Extracting Self Position to Generate Hyperplane
                [ay, by] = obj.find_hyperplane_cg(x(1:2), 0.3, obj.old_g, cloud_points);
                hype = hyperplane(ay, by);
                % Vehicles Reference Steady State Constraints 
                security_margin = 0.1; % in meters
                cnstr = [cnstr ay'*w(1:2)<=(by-security_margin)];                     
            elseif(nargin == 6)
                if(isempty(hyp))
                    %%% Extracting Neighbors Positions Info to Generate Hyperplane
                    positions = [x(1:length(obj.Phi_):end), x(2:length(obj.Phi_):end)]';
                    if(r(2) > 10)
                        positions = positions(:, 1:2);
                        g_nn = g_n(1:2);
                    else
                        positions = [positions(:, 1), positions(:, end)];
                        g_nn = g_n(end-1:end);
                    end
                    [ay, by] = obj.find_hyperplane_cg(positions, 0.3, [obj.old_g, g_nn], cloud_points);
                    hype = hyperplane(ay, by);
                    % Vehicles Reference Steady State Constraints 
                    security_margin = 0.1; % in meters
                    cnstr = [cnstr ay'*w(1:2)<=(by-security_margin)];
                else
                    positions = [x(1:length(obj.Phi_):end), x(2:length(obj.Phi_):end)]';
                    positions = positions(:, 1);
%                     [ay,by] = obj.find_hyperplane_cg(positions, 0.3, [obj.old_g, g_n], cloud_points, hyp);
%                     hype = hyperplane(ay, by);
                    hype = hyp;
                    [ay, by] = double(hype);
                    % Vehicles Reference Steady State Constraints 
                    security_margin = 0.1; % in meters
                    if(isempty(hype))
                        disp(hype);
                    end
                    cnstr = [cnstr ay'*w(1:2)<=(by-security_margin)];
                end
            end
            %
            
            % STEADY STATE ANTICOLLISION CONSTRAINTS
            for i=1:(size(obj.U,1)/4)
                cnstr = [cnstr obj.U((i-1)*4+1,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+1)-mu*d((i-1)*4+1)];
                cnstr = [cnstr obj.U((i-1)*4+2,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+2)-mu*d((i-1)*4+2)];
                cnstr = [cnstr obj.U((i-1)*4+3,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+3)-mu*d((i-1)*4+3)];
                cnstr = [cnstr obj.U((i-1)*4+4,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+4)-mu*d((i-1)*4+4)];
                cnstr = [cnstr sum( d((i-1)*4+(1:4)) ) <= 3];
            end

            %%% TRANSIENT CONSTRAINTS
            for k = 1:obj.k0
                
                % Local and Proximity Constraints 
                cnstr = [cnstr obj.T*(obj.Rk(:, :, k)*w) <= obj.gi - obj.T*obj.bk(:, :, k)*x];
                
                % Obstacle Avoidance Constraints 
                if(nargin > 4)  
                    xk = obj.bk(:, :, k)*x+obj.Rk(:, :, k)*w-obj.L*w;
                    cnstr = [cnstr ay'*xk(1:2) <= (by)];
                end
                
                % Anticollision Constraints
                for i=1:(size(obj.U,1)/4)
                    cnstr = [cnstr (obj.U((i-1)*4+1,:)*(obj.Rk(:, :, k)*w)) >= obj.hi((i-1)*4+1)-mu*b((k-1)*size(obj.U,1)+(i-1)*4+1) - obj.U((i-1)*4+1,:)*obj.bk(:, :, k)*x]; % se i vicini
                    cnstr = [cnstr (obj.U((i-1)*4+2,:)*(obj.Rk(:, :, k)*w)) >=  obj.hi((i-1)*4+2)-mu*b((k-1)*size(obj.U,1)+(i-1)*4+2) - obj.U((i-1)*4+2,:)*obj.bk(:, :, k)*x];
                    cnstr = [cnstr (obj.U((i-1)*4+3,:)*(obj.Rk(:, :, k)*w)) >=  obj.hi((i-1)*4+3)-mu*b((k-1)*size(obj.U,1)+(i-1)*4+3) - obj.U((i-1)*4+3,:)*obj.bk(:, :, k)*x];
                    cnstr = [cnstr (obj.U((i-1)*4+4,:)*(obj.Rk(:, :, k)*w)) >=  obj.hi((i-1)*4+4)-mu*b((k-1)*size(obj.U,1)+(i-1)*4+4) - obj.U((i-1)*4+4,:)*obj.bk(:, :, k)*x];
                    
                    cnstr = [cnstr sum( b((k-1)*size(obj.U,1)+(i-1)*4+(1:4)) ) <= 3];
                    
                    
                end
            end
            
            % Objective function
            obj_fun = (r-g)'*obj.Psi*(r-g);

            
            % Solver options
            assign(g, r); % initial guessing (possible optimization speed up)
            
            options = sdpsettings('verbose',0,'solver',obj.solver_name,'usex0',0,'cachesolvers',1);
           
            ris = optimize(cnstr,obj_fun,options);

            if(ris.problem ~= 0)
                fprintf("WARN: Problem %d \n %s\n", ris.problem, ris.info);
                g = obj.old_g;
            else
                g = double(g);
                obj.old_g = g;
            end
            
            clear('yalmip');
        end
        
        function [a_ris, b_ris, diagnos] = find_hyperplane_cg(~, y, d_min, g, cloud_points, hyp)
            % Function useful to find hyperplane to separate obstacle and vehicles
            % position
            a = sdpvar(2,1);
            b = sdpvar(1,1);
            rr = sdpvar(1,1);
            
            V = [];
            
            % Constraints to keep all the points on one side
            for i = 1:length(cloud_points(1, :))
                V = [V,(a'*cloud_points(:, i))>=(b+d_min)];
            end
                    

           % The Vehicle (or Vehicles) to the other side of the hyperplane
            for i = 1:length(y(1,:))
                V = [V,(a'*y(:, i))<=b-0.009];
            end
            
            for i = 1:length(g(1,:))
                V = [V,(a'*g(:, i))<=b-0.009];
            end
            V = [V, a(1) >= 0.1 ];
            
            % Restrinction on the number of possible hyperplanes
            if(nargin == 6)
                [a_r, b_r] = double(hyp);
                m = - a_r(1)/a_r(2) <= 0;
                if(m)
                     V = [V, a(1) >= 0.1 ];
                     V = [V, a(2) >= 0.5];
                else
                    V = [V, a(1) >= 0.1];
                    V = [V, a(2) <= -0.5];
                end
%                 eps = 0.05;
%                 V = [V, a(1) >= a_r(1) - eps ];
%                 V = [V, a(1) <= a_r(1) + eps ];
%                 V = [V, a(2) >= a_r(2) - eps];
%                 V = [V, a(2) <= a_r(2) + eps ];
            end
            V = [V,(norm(a,2)<=1)];
                    
            % Calculation of the closest point
            tmpx = cloud_points(1,:)-y(1);
            tmpy = cloud_points(2,:)-y(2);
            tmp = [tmpx; tmpy];
            norm_vect = vecnorm(tmp);
            [m, i] = min(norm_vect);
            
            minX = min(cloud_points(1, :));
            maxX = max(cloud_points(1, :));
            minY = min(cloud_points(2, :));
            maxY = max(cloud_points(2, :));
            
            delta = 0.1;
            
            % If the obstacle is not between the vehicle and its target
            % then it is useful to try to engulf the reference into the
            % admissible region
            if(not(g(1)>= minX && g(1)<= maxX && y(1)+delta >= minX && y(1)-delta<= maxX || ...
                    g(2) >= minY && g(2) <= maxY && y(2) + delta>= minY && y(2) - delta <= maxY) )
                
                % V = [V,(a'*g <= b + rr)];
                % In order to weight less the effort to engulf the
                % reference into the objectives' function, a small
                % coefficient is introduced 
                
                reference_ob_weight = 0.000006; 
                safe_vehicle_distance = 0.09;
                ob_fun = reference_ob_weight*norm(rr)-norm(a'*y(:,1)-b+safe_vehicle_distance,inf); 
            else
            % If the obstacle is between the vehicle and its target 
                ob_fun = norm(a'*cloud_points(:, i) - b);
            end
            
            % Optimization
            opt = sdpsettings('verbose',0,'solver','gurobi');
            diagnos = optimize(V, ob_fun, opt);
            
            a_ris = double(a);
            b_ris = double(b);
            
            %disp(a_ris)
            if(diagnos.problem ~= 0)
                warning('Hyperplane Not Found');
                fprintf("WARN: Problem %d \n %s\n", diagnos.problem, diagnos.info);
            end
            if (norm(a_ris) == 0 && b_ris == 0)
                warning('Hyperplane Not Found');
            end 
            
            % Debug purpose
%             hy = hyperplane(a_ris,b_ris);
%             plot(hy);
        end
    end
end
