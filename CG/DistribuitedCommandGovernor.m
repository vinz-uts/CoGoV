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
        function obj = DistribuitedCommandGovernor(Phi,G,Hc,L,T,gi,U,hi,Psi,k0,delta,dist_flag)
            % DistribuitedCommandGovernor - Constructor
            % Create an instance of a Distribuited Command Governor.
            obj = obj@CommandGovernor(Phi,G,Hc,L,T,gi,Psi,k0,delta,dist_flag);
            if nargin > 10
                % Introducing binary slack variables for OR-ed constraints
                %b = binvar(size(U,1)/2,1);
                %B_diag = [];
                %for i=1:(size(U,1)/2)
                %    B_diag = blkdiag(B_diag,b(i),~b(i));
                %end
                %obj.U = B_diag*U; % proximity constraints matrix
                %obj.hi = B_diag*hi; % proximity constraints vector
                obj.U = U;
                obj.hi = hi;
            end
        end
        
        function g = compute_cmd(obj,x,r,g_n)
            % compute_cmd - calculate the reference g.
            % Calculate the nearest reference g to r start from initial
            % global conditions x and g_n reference for the other systems.
            try
                g = sdpvar(length(r),1);
                %b = binvar(size(obj.U,1)/2,1);
                w = [g;g_n];
                %cnstr = ismember((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w,obj.C_k);
                cnstr = obj.T*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) <= obj.gi;

                for i=1:(size(obj.U,1)/2)
                    %cnstr = [cnstr obj.U((i-1)*2+1,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w)<=obj.hi((i-1)*2+1) | ...
                    %    obj.U((i-1)*2+2,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w)<=obj.hi((i-1)*2+2)];
                    %cnstr = [cnstr implies(b(i),obj.U((i-1)*2+1,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w)<=obj.hi((i-1)*2+1)) ...
                    %    implies((1-b(i)),obj.U((i-1)*2+2,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w)<=obj.hi((i-1)*2+2))];
                    %cnstr = [cnstr implies(b(i),obj.U((i-1)*2+1,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w)<=obj.hi((i-1)*2+1))];
                    %cnstr = [cnstr implies((~b(i)),obj.U((i-1)*2+2,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w)<=obj.hi((i-1)*2+2))];
                    cnstr = [cnstr norm(obj.U((i-1)*2+1,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w),Inf)>=-obj.hi((i-1)*2+1)];
                end
                xk = x;
                if obj.dist_flag == 0
                    % Case: without disturbance
                    for k = 1:obj.k0
                        xk = obj.Phi*xk+obj.G*w;
                        cnstr = [cnstr ismember(obj.Hc*xk+obj.L*w,obj.C_k)];
                        %cnstr = [cnstr obj.U*(obj.Hc*xk+obj.L*w)<=obj.hi];
                        for i=1:(size(obj.U,1)/2)
                             %cnstr = [cnstr (obj.U((i-1)*2+1,:)*(obj.Hc*xk+obj.L*w)<=obj.hi((i-1)*2+1)|obj.U((i-1)*2+2,:)*(obj.Hc*xk+obj.L*w)<=obj.hi((i-1)*2+2))];
                             %cnstr = [cnstr implies(b(i),obj.U((i-1)*2+1,:)*(obj.Hc*xk+obj.L*w)<=obj.hi((i-1)*2+1)) ...
                             %   implies((~b(i)),obj.U((i-1)*2+2,:)*(obj.Hc*xk+obj.L*w)<=obj.hi((i-1)*2+2))];
                             %cnstr = [cnstr implies(b(i),((obj.U((i-1)*2+1,:)*(obj.Hc*xk+obj.L*w))<=obj.hi((i-1)*2+1)))];
                             %%cnstr = [cnstr implies(b(i),obj.U((i-1)*2+1,:)*((obj.Hc*xk+obj.L*w))<=obj.hi((i-1)*2+1))]
                             %cnstr = [cnstr implies(b(i),obj.U((i-1)*2+1,:)*((obj.Hc*xk+obj.L*w))<=obj.hi((i-1)*2+1))];
                             cnstr = [cnstr norm(obj.U((i-1)*2+1,:)*(obj.Hc*xk+obj.L*w),Inf)>=-obj.hi((i-1)*2+1)];
                        end
                    end
                else
                    % Case: with disturbance
                    for k = 1:obj.k0
                        xk = obj.Phi*xk+obj.G*w;
                        cnstr = [cnstr ismember(obj.Hc*xk+obj.L*w,obj.C_k{k})];
                        %cnstr = [cnstr obj.U*(obj.Hc*xk+obj.L*w)<=obj.hi]; % verify
                        for i=1:(size(obj.U,1)/2)
                             cnstr = [cnstr obj.U((i-1)*2+1,:)*(obj.Hc*xk+obj.L*w)<=obj.hi((i-1)*2+1) | ...
                             obj.U((i-1)*2+2,:)*(obj.Hc*xk+obj.L*w)<=obj.hi((i-1)*2+2)];
                        end
                    end
                end
                % Objective function
                obj_fun = (r-g)'*obj.Psi*(r-g);
                % Solver options
                options = sdpsettings('verbose',0,'solver','bmibnb');

                solvesdp(cnstr,obj_fun,options);
                g = double(g);
            catch Exc
                disp('WARN: infeasible');
                g = [];
            end
        end    
    end
end
