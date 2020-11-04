classdef CommandGovernorCAC < handle
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
        solver_name % name of the numerical solver
        Rk % Rk matrix for code speed up
        bk % bk matrix for code speed up
    end
    
    properties (Constant)
        default_solver = 'gurobi';
    end
    
    
    methods
        function obj = CommandGovernorCAC(Phi,G,Hc,L,T,gi,Psi,k0,solver)
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
            if nargin > 8
                obj.solver_name = obj.check_solver(solver);
            else
                obj.solver_name = obj.default_solver;
            end
            
            [obj.Rk, obj.bk] = obj.compute_matrix();
        end
        
        
        function [g, ris] = compute_cmd(obj,x,r,ve)
            % compute_cmd - calculate the reference g2
            % Calculate the nearest reference g to r start from initial
            % conditions x.
            w = sdpvar(length(r),1);
            
            
%             a = sdpvar(2,1);
%             b = sdpvar(1,1);
           
%             
            
            cnstr = obj.T*(obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w <= obj.gi;
            xk = x;
            
            if(not(isempty(ve)))
                 rr =sdpvar(1,1);
                [aris,bris] = find_hyperplane(ve(:,1),ve(:,2),ve(:,3),ve(:,4),x(1:2)); %how to identify position of vehicle 
                a=aris;
                b=bris;
            end

            
            
            for k = 1:obj.k0
                xk = obj.Phi*xk+obj.G*w;
                cnstr = [cnstr obj.T*(obj.Hc*xk+obj.L*w) <= obj.gi];
                
                if(not(isempty(ve)))
                    cnstr = [cnstr a'*xk(1:2) <= (b-rr)];
                end
            end
            
            if(not(isempty(ve)))
                cnstr = [cnstr a'*w <= (b)];
            end
%             V1 = a'*ve1 >= b +rr;
%             V2 = a'*ve2 >= b +rr;
%             V3 = a'*ve3 >= b +rr;
%             V4 = a'*ve4 >= b +rr;
%             V6 = norm(a,2) <= 1;

%             cnstr=[cnstr V1 V2 V3 V4 V6];
            
            % Objective function
            obj_fun = (r-w)'*obj.Psi*(r-w);
            % Solver options
            options = sdpsettings('verbose',0,'solver','gurobi'); %obj.solver_name
            
            ris = solvesdp(cnstr,obj_fun,options);
            g = double(w);

            
            if(ris.problem ~= 0)
                fprintf(...
                    "WARNING! Problem %d visit \n https://www.gurobi.com/documentation/9.0/refman/optimization_status_codes.html \n %s\n", ris.problem, ris.info...
                    );
                g = [];
            end
            
            clear('yalmip');
        end
    end
    
    methods (Access = protected)
        function check_solver(obj, solver_name)
            % check_solver - check if solver is available 
            % If not present, default solver is setted 

            obj.solver_name = obj.default_solver;

            [solvers,found] = getavailablesolvers(0);

            for i = 1:length(solvers)
                if (strcmpi(solvers(i).tag,solver_name)==1 && found(i))
                    obj.solver_name = solver_name;
                    break
                end
            end
            fprintf("Solver setted: %s\n",obj.solver_name);
        end % check_solver
        
        function [Rk, bk] = compute_matrix(obj)
            % code optimization
            % computation of matrices for direct state evolution
            % output:
            % - Rk : is a three-dimensional matrix in which every 'slice' is
            %           related to a k-prediction.
            %           Rk(:, :, k) is multiplied by w
            % - bk : is a three-dimensional matrix in which every 'slice' is
            %           related to a k-prediction.
            %           this term (premultiplied by x0) has to be substructed
            %           to the known term
            
            % preallocation for code speed up
            bk = zeros(length(obj.Hc(:, 1)), length(obj.Phi(1, :)), obj.k0);
            [m, n] = size(obj.L);
            Rk = zeros(m, n, obj.k0);
            
            for k = 1:obj.k0 % for each prediction compute a Rk, bk
                bk(:, :, k) = (obj.Hc*(obj.Phi^k));
                
                Rk_temp = zeros(length(obj.Phi), length(obj.G(1, :)));
                for j = 0:k-1
                    Rk_temp = Rk_temp + (obj.Phi^j)*obj.G;
                end
                Rk(:, :, k) = obj.Hc*Rk_temp + obj.L;
            end
        end % private methods
    end  
end

