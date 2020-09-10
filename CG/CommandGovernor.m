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
        solvername % name of the numerical solver 
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
            obj.solvername = check_solver(); % check for 'gurobi' solver
        end
        
        
        function [g, ris] = compute_cmd(obj,x,r)
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
                options = sdpsettings('verbose',0,'solver',obj.solvername);
                
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

