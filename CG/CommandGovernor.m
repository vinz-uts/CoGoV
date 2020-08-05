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
    end
    
    
    methods
        function obj = CommandGovernor(Phi,G,Hc,L,T,gi,Psi,k0)
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
        end
        
        
        function g = compute_cmd(obj,x,r)
            % compute_cmd - calculate the reference g.
            % Calculate the nearest reference g to r start from initial
            % conditions x.
            try
                w = sdpvar(length(r),1);
                cnstr = obj.T*(obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w <= obj.gi;
                xk = x;

                for k = 1:obj.k0
                    xk = obj.Phi*xk+obj.G*w;
                    cnstr = [cnstr obj.T*(obj.Hc*xk+obj.L*w) <= obj.gi];
                end

                % Objective function
                obj_fun = (r-w)'*obj.Psi*(r-w);
                % Solver options
                options = sdpsettings('verbose',0,'solver','gurobi');

                solvesdp(cnstr,obj_fun,options);
                g = double(w);
            catch Exc
                disp('WARN: infeasible');
                g = [];
            end
        end  
    end
end

