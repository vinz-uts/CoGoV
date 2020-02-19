classdef CommandGovernor
    %% COMMAND GOVERNOR
    %  Command Governor computes the nearest reference g to r that statify
    %  the constrains.
    
    properties
        Phi % closed-loop model Φ matrix
        G % closed-loop model G matrix
        Hc % closed-loop model Hc matrix
        L % closed-loop model L matrix
        T % constraints matrix
        gi % constraints vector
        Psi % reference weight Ψ matrix
        k0 % prediction steps number
        dist_flag % disturbance flag 
        C_k % constraints set
    end
    
    
    methods
        function obj = CommandGovernor(Phi,G,Hc,L,T,gi,Psi,k0,delta,dist_flag)
            % CommandGovernor - Constructor
            % Create an instance of a Command Governor
            obj.Phi = Phi;
            obj.G = G;
            obj.Hc = Hc;
            obj.L = L;
            obj.T = T;
            obj.gi = gi;
            obj.Psi = Psi;
            obj.k0 = k0;
            obj.dist_flag = dist_flag;
            
            % C_k sets construction
            c = sdpvar(size(Hc,1),1);   c_d = sdpvar(size(Hc,1),1);
            C_set = T*c <= gi;
            B_del = norm(c)^2 <= delta; % δ-radius ball
            obj.C_k = robustify([ismember(c+c_d,C_set),ismember(c_d,B_del),uncertain(c_d)]); % set constriction
            
        end
        
        
        function g = compute_cmd(obj,x,r)
            % compute_cmd - calculate the reference g.
            % Calculate the nearest reference g to r start from initial
            % conditions x.
            w = sdpvar(length(r),1);
            cnstr = ismember((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w,obj.C_k);
            xk = x;
            if obj.dist_flag == 0
                % Case: without disturbance
                for k = 1:obj.k0
                    xk = obj.Phi*xk+obj.G*w;
                    cnstr = [cnstr ismember(obj.Hc*xk+obj.L*w,obj.C_k)];
                end
            else
                % Case: with disturbance
                for k = 1:obj.k0
                    xk = obj.Phi*xk+obj.G*w;
                    cnstr = [cnstr ismember(obj.Hc*xk+obj.L*w,obj.C_k{k})];
                end
            end
            % Objective function
            obj_fun = (r-w)'*obj.Psi*(r-w);
            % Solver options
            options = sdpsettings('verbose',0,'solver','sedumi');
            
            solvesdp(cnstr,obj_fun,options);
            g = double(w);
        end  
    end
end

