classdef DistribuitedCommandGovernor < CommandGovernor
    %% DISTRIBUITED COMMAND GOVERNOR
    %  Distribuite Command Governor for multi-agents nets. Computes
    %  the nearest reference g to r that statify local and global (with 
    %  other system) constraints.
    
    properties
        U % proximity constraints matrix
        hi % proximity constraints vector
    end
    
    methods
        function obj = DistribuitedCommandGovernor(Phi,G,Hc,L,T,gi,U,hi,Psi,k0,delta,dist_flag)
            % DistribuitedCommandGovernor - Constructor
            % Create an instance of a Distribuited Command Governor.
            obj = obj@CommandGovernor(Phi,G,Hc,L,T,gi,Psi,k0,delta,dist_flag);
            if nargin > 10
                obj.U = U;
                obj.hi = hi;
                % C_k set extension
                c = sdpvar(size(Hc,1),1);   c_d = sdpvar(size(Hc,1),1);
                C_set = T*c <= gi;
                B_del = norm(c)^2 <= delta; % δ-radius ball
                
                b = binvar(size(U,1)/2,1);
                B_diag = [];
                for i=1:(size(U,1)/2)
                    B_diag = blkdiag(B_diag,b(i),~b(i));
                end
                obj.U = B_diag*U;
                obj.hi = B_diag*hi;
                
                obj.C_k = robustify([ismember(c+c_d,C_set),ismember(c_d,B_del),uncertain(c_d)]); % set constriction
            end
        end
        
        function g = compute_cmd(obj,x,r,g_n)
            % compute_cmd - calculate the reference g.
            % Calculate the nearest reference g to r start from initial
            % global conditions x and g_n reference for the other systems.
            g = sdpvar(length(r),1);
            w = [g;g_n];
            cnstr = ismember((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w,obj.C_k);
            cnstr = [cnstr obj.U*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w)<=obj.hi];
            xk = x;
            if obj.dist_flag == 0
                % Case: without disturbance
                for k = 1:obj.k0
                    xk = obj.Phi*xk+obj.G*w;
                    cnstr = [cnstr ismember(obj.Hc*xk+obj.L*w,obj.C_k)];
                    cnstr = [cnstr obj.U*(obj.Hc*xk+obj.L*w)<=obj.hi];
                end
            else
                % Case: with disturbance
                for k = 1:obj.k0
                    xk = obj.Phi*xk+obj.G*w;
                    cnstr = [cnstr ismember(obj.Hc*xk+obj.L*w,obj.C_k{k})];
                end
            end
            % Objective function
            obj_fun = (r-g)'*obj.Psi*(r-g);
            % Solver options
            options = sdpsettings('verbose',0,'solver','bnb');
            
            solvesdp(cnstr,obj_fun,options);
            g = double(g);
        end
    end
end

