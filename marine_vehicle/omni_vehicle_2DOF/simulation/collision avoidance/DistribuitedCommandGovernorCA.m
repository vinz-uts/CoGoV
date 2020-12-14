classdef DistribuitedCommandGovernorCA < CommandGovernor
    %% DISTRIBUITED COMMAND GOVERNOR
    %  Distribuite Command Governor for multi-agents nets. Computes
    %  the nearest reference g to r that statify local and global (with
    %  other system) constraints.
    
    properties
        U % proximity constraints matrix - matrix for OR-ed constraints
        hi % proximity constraints vector - vector for OR-ed constraints
    end
    
    
    methods
        
        function obj = DistribuitedCommandGovernorCA(Phi,G,Hc,L,T,gi,U,hi,Psi,k0,solver)
            % DistribuitedCommandGovernor - Constructor
            % Create an instance of a Distribuited Command Governor.
            obj = obj@CommandGovernor(Phi,G,Hc,L,T,gi,Psi,k0);
            if nargin > 10 
                obj.check_solver(solver);
            end
            obj.U = U;
            obj.hi = hi;
        end
        
        
        function [g, ris] = compute_cmd(obj,x,r,g_n,ve1,ve2,ve3,ve4)
            % compute_cmd - calculate the reference g.
            % Calculate the nearest reference g to r start from initial
            % global conditions x and g_n reference for the other systems.
            g = sdpvar(length(r),1);
            w = [g;g_n];
            b = binvar(size(obj.U,1)*obj.k0,1);
            d = binvar(size(obj.U,1),1);
            rr = sdpvar(1,1);
            mu = 10000;
            cnstr = obj.T*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) <= obj.gi;
            
            [aris,bris] = find_hyperplane(ve1,ve2,ve3,ve4,x(1:2)); 
            
            ay=aris;
            by=bris;
            
            for i=1:(size(obj.U,1)/4)
                cnstr = [cnstr obj.U((i-1)*4+1,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+1)-mu*d((i-1)*4+1)];
                cnstr = [cnstr obj.U((i-1)*4+2,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+2)-mu*d((i-1)*4+2)];
                cnstr = [cnstr obj.U((i-1)*4+3,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+3)-mu*d((i-1)*4+3)];
                cnstr = [cnstr obj.U((i-1)*4+4,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+4)-mu*d((i-1)*4+4)];
                cnstr = [cnstr sum( d((i-1)*4+(1:4)) ) <= 3];
            end
            
            %%%% old code uncomment to test
%             xk = x;
            %%%
            
            for k = 1:obj.k0
                %%%% HERE CA CONSTRAINTS
                xk = obj.bk(:, :, k)*x + obj.Rk(:, :, k)*w - obj.L*w; 
                
                cnstr = [cnstr obj.T*(obj.Rk(:, :, k)*w) <= obj.gi - obj.T*obj.bk(:, :, k)*x];
                cnstr = [cnstr ay'*xk(1:2) <= (by-rr)];
                
                for i=1:(size(obj.U,1)/4)
                    cnstr = [cnstr (obj.U((i-1)*4+1,:)*(obj.Rk(:, :, k)*w)) >= obj.hi((i-1)*4+1)-mu*b((k-1)*size(obj.U,1)+(i-1)*4+1) - obj.U((i-1)*4+1,:)*obj.bk(:, :, k)*x]; % se i vicini
                    cnstr = [cnstr (obj.U((i-1)*4+2,:)*(obj.Rk(:, :, k)*w)) >=  obj.hi((i-1)*4+2)-mu*b((k-1)*size(obj.U,1)+(i-1)*4+2) - obj.U((i-1)*4+2,:)*obj.bk(:, :, k)*x];
                    cnstr = [cnstr (obj.U((i-1)*4+3,:)*(obj.Rk(:, :, k)*w)) >=  obj.hi((i-1)*4+3)-mu*b((k-1)*size(obj.U,1)+(i-1)*4+3) - obj.U((i-1)*4+3,:)*obj.bk(:, :, k)*x];
                    cnstr = [cnstr (obj.U((i-1)*4+4,:)*(obj.Rk(:, :, k)*w)) >=  obj.hi((i-1)*4+4)-mu*b((k-1)*size(obj.U,1)+(i-1)*4+4) - obj.U((i-1)*4+4,:)*obj.bk(:, :, k)*x];
                    
                    cnstr = [cnstr sum( b((k-1)*size(obj.U,1)+(i-1)*4+(1:4)) )<= 3];
                end
            end
            
            cnstr = [cnstr ay'*g <= (by)];
            
            % Objective function
            obj_fun = (r-g)'*obj.Psi*(r-g);
            
            % Solver options
            assign(g, r); % initial guessing (possible optimization speed up)
            
            options = sdpsettings('verbose',0,'solver','gurobi','usex0',1,'cachesolvers',1);
            
            ris = optimize(cnstr,obj_fun,options);
            g = double(g);
            
            if(ris.problem ~= 0)
                fprintf("WARN: Problem %d \n %s\n", ris.problem, ris.info);
                g = [];
            end
            
            clear('yalmip');
        end
    end
end