classdef DistribuitedCommandGovernorCAFULL < CommandGovernor
    %% DISTRIBUITED COMMAND GOVERNOR
    %  Distribuite Command Governor for multi-agents nets. Computes
    %  the nearest reference g to r that statify local and global (with
    %  other system) constraints.
    
    properties
        U % proximity constraints matrix - matrix for OR-ed constraints
        hi % proximity constraints vector - vector for OR-ed constraints
    end
    
    
     
    methods
        function obj = DistribuitedCommandGovernorCAFULL(Phi,G,Hc,L,T,gi,U,hi,Psi,k0,solver)
            % DistribuitedCommandGovernor - Constructor
            % Create an instance of a Distribuited Command Governor.
            obj = obj@CommandGovernor(Phi,G,Hc,L,T,gi,Psi,k0);
            if nargin > 10 
                obj.check_solver(solver);
            end
            obj.U = U;
            obj.hi = hi;
        end
        
        
        function [g, ris, hype] = compute_cmd(obj,x,r,g_n, cloud_points)
            % compute_cmd - calculate the reference g.
            % Calculate the nearest reference g to r start from initial
            % global conditions x and g_n reference for the other systems.
            g = sdpvar(length(r),1);
            w = [g;g_n];
            b = binvar(size(obj.U,1)*obj.k0,1);
            d = binvar(size(obj.U,1),1);
            mu = 10000;
            hype = [];
            
            %%% Steady State Constraints 
            cnstr = obj.T*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) <= obj.gi;
           
            if(nargin > 4)  % Obstacle avoidance
                slack_var = 0;
                a = sdpvar(2,1);
                b = sdpvar(1,1);
                %             r = sdpvar(1,1);
%                 rr = sdpvar(1,1);
                
                for i = 1:length(cloud_points(1, :))
                    cnstr = [cnstr, a'*cloud_points(:, i) >= b + 0.4];
                end
                
               	cnstr = [cnstr, a'*x(1:2) <= b];
                
                % Restrinction on the number of possible hyperplane
                cnstr = [cnstr, norm(a,2) <= 1];        
                cnstr = [cnstr a'*g <= (b)];
            end
            
            for i=1:(size(obj.U,1)/4)
                cnstr = [cnstr obj.U((i-1)*4+1,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+1)-mu*d((i-1)*4+1)];
                cnstr = [cnstr obj.U((i-1)*4+2,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+2)-mu*d((i-1)*4+2)];
                cnstr = [cnstr obj.U((i-1)*4+3,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+3)-mu*d((i-1)*4+3)];
                cnstr = [cnstr obj.U((i-1)*4+4,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+4)-mu*d((i-1)*4+4)];
                cnstr = [cnstr sum( d((i-1)*4+(1:4)) ) <= 3];
            end

            %%% Transient Constraints
            for k = 1:obj.k0
                cnstr = [cnstr obj.T*(obj.Rk(:, :, k)*w) <= obj.gi - obj.T*obj.bk(:, :, k)*x];
                
                if(nargin > 4)  % Obstacle avoidance
                    xk = obj.bk(:, :, k)*x + obj.Rk(:, :, k)*w - obj.L*w;
                    cnstr = [cnstr a'*xk(1:2) <= (b-slack_var)];
                end
                
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
            
            options = sdpsettings('verbose',0,'solver',obj.solver_name,'usex0',1,'cachesolvers',1);
           
            ris = optimize(cnstr,obj_fun,options);
            g = double(g);
            if(nargin > 4) 
                a_ris = double(a);
                b_ris = double(b);
                
                hype = hyperplane(a_ris,b_ris);

            end

            if(ris.problem ~= 0)
                fprintf("WARN: Problem %d \n %s\n", ris.problem, ris.info);
                g = [];
            end
            
            clear('yalmip');
        end
        
        function [a_ris, b_ris, diagnos] = find_hyperplane_cg(obj, y, d_min, g, cloud_points)
            % Function useful to find hyperplane to separate obstacle and vehicles
            % position
            a = sdpvar(2,1);
            b = sdpvar(1,1);
%             r = sdpvar(1,1);
            rr = sdpvar(1,1);
            r = 0;
            
            V = [];
            % Constraints to keep all the points on one side
            for i = 1:length(cloud_points(1, :))
                V = [V, a'*cloud_points(:, i) >= b - r + d_min];
            end
            
            % and the veichle to the other side of the hyperplane
            V = [V, a'*y <= b];
            
            % Restrinction on the number of possible hyperplane
            V = [V, norm(a,2) <= 1];
                    
            % Calculation of the closest point
            tmpx = cloud_points(1,:) - y(1);
            tmpy = cloud_points(2,:) - y(2);
            tmp = [tmpx; tmpy];
            norm_vect = vecnorm(tmp);
            [m, i] = min(norm_vect);
            
            minX = min(cloud_points(1, :));
            maxX = max(cloud_points(1, :));
            minY = min(cloud_points(2, :));
            maxY = max(cloud_points(2, :));
            
            delta = 0.1;
            if(not(g(1) >= minX && g(1) <= maxX && y(1) + delta >= minX && y(1) - delta <= maxX || ...
                    g(2) >= minY && g(2) <= maxY && y(2) + delta>= minY && y(2) - delta <= maxY))
                
                V = [V, a'*g <= b + rr];
%                 V = [V, -0.1 <= r <= 0.1];
                ob_fun = norm(r,2) + norm(rr) + norm(a'*cloud_points(:, i) - b) ;
            else
                
%                 V = [V, -0.1 <= r <= 0.1];
                ob_fun = norm(r,2)  + norm(a'*cloud_points(:, i) - b);
            end
            
            % Optimization
            opt = sdpsettings('verbose',0,'solver','gurobi');
            diagnos = optimize(V, ob_fun, opt);
            
            a_ris = double(a);
            b_ris = double(b);
            
            fprintf('rr=%d\n r = %d\n', double(rr), double(r));
            if (norm(a_ris) == 0 && b_ris == 0)
                warning('Hyperplane Not Found');
            end
            
            % hy = hyperplane(aris,bris);
            % plot(hy);
        end
    end
end
