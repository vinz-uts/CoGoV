classdef CentralizedCommandGovernor < CommandGovernor
    %% CENTRALIZED COMMAND GOVERNOR
    %  Centralized Command Governor for multi-agents nets. Computes
    %  the nearest references g_i to r_i that statify local and global (with 
    %  other system) constraints.
    
    properties
        U % proximity constraints matrix - matrix for OR-ed constraints
        hi % proximity constraints vector - vector for OR-ed constraints
    end
    
    
    methods
        function obj = CentralizedCommandGovernor(Phi,G,Hc,L,T,gi,U,hi,Psi,k0)
            % CentralizedCommandGovernor - Constructor
            % Create an instance of a Centralized Command Governor.
            obj = obj@CommandGovernor(Phi,G,Hc,L,T,gi,Psi,k0);
            obj.U = U;
            obj.hi = hi;
        end
        
        
        function [g, ris] = compute_cmd(obj,x,r)
            % compute_cmd - calculate the reference g.
            % Calculate the nearest references g_i to r_i start from initial
            % global conditions x.
            try
                w = sdpvar(length(r),1);
                b = binvar((size(obj.U,1)/4)*(obj.k0)*4,1);
                d = binvar(size(obj.U,1),1);
                mu = 1000;
                cnstr = obj.T*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) <= obj.gi;
                for i=1:(size(obj.U,1)/4)
                    cnstr = [cnstr obj.U((i-1)*4+1,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+1)-mu*d((i-1)*4+1)];
                    cnstr = [cnstr obj.U((i-1)*4+2,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+2)-mu*d((i-1)*4+2)];
                    cnstr = [cnstr obj.U((i-1)*4+3,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+3)-mu*d((i-1)*4+3)];
                    cnstr = [cnstr obj.U((i-1)*4+4,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+4)-mu*d((i-1)*4+4)];
                    cnstr = [cnstr sum(d(((i-1)*4+1):((i-1)*4+4))) <= 3];
                    %cnstr = [cnstr 3.5 <= 3];
                end
                xk = x;
                
             
                for k = 1:obj.k0
                    xk = obj.Phi*xk+obj.G*w;  % xk = (obj.Phi)^k * x0 + sum(i=1,k-1) (obj.Phi^i*obj.G)*w
                    cnstr = [cnstr obj.T*(obj.Hc*xk+obj.L*w) <= obj.gi];
                    for i=1:(size(obj.U,1)/4)
                        cnstr = [cnstr (obj.U((i-1)*4+1,:)*(obj.Hc*xk+obj.L*w)) >= obj.hi((i-1)*4+1)-mu*b((k-1)*size(obj.U,1)+(i-1)*4+1)]; % se i vicini 
                        cnstr = [cnstr (obj.U((i-1)*4+2,:)*(obj.Hc*xk+obj.L*w)) >=  obj.hi((i-1)*4+2)-mu*b((k-1)*size(obj.U,1)+(i-1)*4+2)];
                        cnstr = [cnstr (obj.U((i-1)*4+3,:)*(obj.Hc*xk+obj.L*w)) >=  obj.hi((i-1)*4+3)-mu*b((k-1)*size(obj.U,1)+(i-1)*4+3)];
                        cnstr = [cnstr (obj.U((i-1)*4+4,:)*(obj.Hc*xk+obj.L*w)) >=  obj.hi((i-1)*4+4)-mu*b((k-1)*size(obj.U,1)+(i-1)*4+4)];
                        cnstr = [cnstr sum( b((k-1)*size(obj.U,1)+(i-1)*4+1:(k-1)*size(obj.U,1)+(i-1)*4+4)) <= 3];
                        (k-1)*size(obj.U,1)+(i-1)*4+1:(k-1)*size(obj.U,1)+(i-1)*4+4
                        %                          ((k-1)*4+(k+i-2)*4+1):((k-1)*4+(k+i-2)*4+4)
                    end
                end
                
                % Objective function
                obj_fun = (r-w)'*obj.Psi*(r-w);
                % Solver options
                assign(w, r*100);
                
                options = sdpsettings('verbose',0,'solver','gurobi','usex0',1);

                ris = solvesdp(cnstr,obj_fun,options);
                g = double(w);
                if(ris.problem ~= 0)
                   fprintf(...
                   "WARNING! Problem %d visit \n https://www.gurobi.com/documentation/9.0/refman/optimization_status_codes.html \n %s\n", ris.problem, ris.info...
                   );
                    g = [];      
                end
                
            catch Exc
                warning('Exception thrown during optimization: \n info: %s \n', getReport(Exc));
                ris = [];
                g = [];
            end
        end    
    end
end
