classdef DistribuitedCommandGovernor < CommandGovernor
    %% DISTRIBUITED COMMAND GOVERNOR
    %  Distribuite Command Governor for multi-agents nets. Computes
    %  the nearest reference g to r that statify local and global (with 
    %  other system) constraints.
    
    properties
        U % proximity constraints matrix - matrix for OR-ed constraints
        hi % proximity constraints vector - vector for OR-ed constraints
    end
    
    
    methods
        function obj = DistribuitedCommandGovernor(Phi,G,Hc,L,T,gi,U,hi,Psi,k0)
            % DistribuitedCommandGovernor - Constructor
            % Create an instance of a Distribuited Command Governor.
            obj = obj@CommandGovernor(Phi,G,Hc,L,T,gi,Psi,k0);
            obj.U = U;
            obj.hi = hi;
        end
        
        
        function [g,s] = compute_cmd(obj,x,r,g_n)
            % compute_cmd - calculate the reference g.
            % Calculate the nearest reference g to r start from initial
            % global conditions x and g_n reference for the other systems.
            try
                g = sdpvar(length(r),1);
                w = [g;g_n];
                b = binvar(size(obj.U,1)*(obj.k0+1),1);
                d = binvar(size(obj.U,1),1);
                mu = 1000;
                cnstr = obj.T*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) <= obj.gi;
                for i=1:(size(obj.U,1)/4)
                    cnstr = [cnstr obj.U((i-1)*4+1,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi-mu*d((i-1)*4+1)];
                    cnstr = [cnstr obj.U((i-1)*4+2,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi-mu*d((i-1)*4+2)];
                    cnstr = [cnstr obj.U((i-1)*4+3,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi-mu*d((i-1)*4+3)];
                    cnstr = [cnstr obj.U((i-1)*4+4,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi-mu*d((i-1)*4+4)];
                    cnstr = [cnstr sum(d(((i-1)*4+1):((i-1)*4+4))) <= 3];
                    %cnstr = [cnstr 3.5 <= 3];
                end
                xk = x;
                for k = 1:obj.k0
                    xk = obj.Phi*xk+obj.G*w;
                    cnstr = [cnstr obj.T*(obj.Hc*xk+obj.L*w) <= obj.gi];
                    for i=1:(size(obj.U,1)/4)
                        cnstr = [cnstr (obj.U((i-1)*4+1,:)*(obj.Hc*xk+obj.L*w)) >= obj.hi-mu*b((k-1)*4+(i-1)*4+1)];
                        cnstr = [cnstr (obj.U((i-1)*4+2,:)*(obj.Hc*xk+obj.L*w)) >= obj.hi-mu*b((k-1)*4+(i-1)*4+2)];
                        cnstr = [cnstr (obj.U((i-1)*4+3,:)*(obj.Hc*xk+obj.L*w)) >= obj.hi-mu*b((k-1)*4+(i-1)*4+3)];
                        cnstr = [cnstr (obj.U((i-1)*4+4,:)*(obj.Hc*xk+obj.L*w)) >= obj.hi-mu*b((k-1)*4+(i-1)*4+4)];
                        cnstr = [cnstr sum(b(((k-1)*4+(i-1)*4+1):((k-1)*4+(i-1)*4+4))) <= 3];
                    end
                end
                
                % Objective function
                obj_fun = (r-g)'*obj.Psi*(r-g);
                
                % Solver options
                assign(g,[100,100]');
                
                options = sdpsettings('verbose',0,'solver','gurobi','usex0',1);

                s = solvesdp(cnstr,obj_fun,options);
                g = double(g);
                
                if(s.problem ~= 0)
                    fprintf("WARNING! Problem %d visit \n https://www.gurobi.com/documentation/9.0/refman/optimization_status_codes.html \n ",s.problem);
                    g = [];      
                end
                
                      
            catch Exc
                disp('WARN: infeasible');
                s = [];
                g = [];
            end
        end    
    end
end
