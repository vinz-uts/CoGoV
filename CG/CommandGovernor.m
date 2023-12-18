classdef CommandGovernor < handle
    %COMMANDGOVERNOR class
    %   Command Governor computes the nearest reference g to r that statify
    %   the constrains.
    %  
    %   cg = COMMANDGOVERNOR(Phi,G,Hc,L,Psi,k0) creates an instance of 
    %      Command Governor with the closed-loop matrices Phi,G,Hc,L. Psi is
    %      the reference weight matrix, k0 is the prediction horizon.
    %  
    %   cg = COMMANDGOVERNOR(Phi,G,Hc,L,Psi,k0, solver) creates an instance of 
    %      Command Governor with the closed-loop matrices Phi,G,Hc,L. Psi is
    %      the reference weight matrix, k0 is the prediction horizon. A different
    %      solver can be specified with the last parameter. Default solver is 'bmibnb'.
    % 
    %  Authors: Vincenzo D'Angelo, Ayman El Qemmah, Franco Angelo Torchiaro
    %  Credits: Alessandro Casavola, Francesco Tedesco
    %  Copyright 2021 - CoGoV.

    properties
        Phi         % Closed-loop model Φ matrix
        G           % Closed-loop model G matrix
        Hc          % Closed-loop model Hc matrix
        L           % Closed-loop model L matrix
        T           % Constraints matrix
        gi          % Constraints vector
        Psi         % Reference weight Ψ matrix
        k0          % Prediction steps number
        solver_name % Name of the numerical solver
        Rk          % Rk matrix for code speed up
        bk          % bk matrix for code speed up
    end
    
    properties (Constant)
        default_solver = 'bmibnb';
    end
    
    
    methods
        function obj = CommandGovernor(Phi,G,Hc,L,T,gi,Psi,k0,solver)
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
        
        
        function [g, ris] = compute_cmd(obj,x,r)
            %COMPUTE_CMD - calculate the reference g.
            %   [g,ris] = COMPUTE_CMD(x,r) Calculate the nearest reference g to r start from initial
            %   conditions x. Output of optimization problem solving is returned in ris.
            
            w = sdpvar(length(r),1);
            
            %%% Uncomment for normal behaviour of CG
            cnstr = obj.T*(obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w <= obj.gi;
            xk = x;

            for k = 1:obj.k0
                xk = obj.Phi*xk+obj.G*w;
                cnstr = [cnstr obj.T*(obj.Hc*xk+obj.L*w) <= obj.gi];
            end
            
            % Objective function
            obj_fun = (r-w)'*obj.Psi*(r-w);
            % Solver options
            options = sdpsettings('verbose',0,'solver',obj.solver_name);
            
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
            %CHECK_SOLVER - check if solver is available
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
        
        function [Rk, bk] = compute_matrix(obj, Phi, Hc, G, L, k0)
            %COMPUTE_MATRIX computes Rk, bk matrices for code optimization
            %    Computation of matrices for direct state evolution
            %       * Rk : is a three-dimensional matrix in which every 'slice' is
            %         related to a k-prediction. Rk(:, :, k) is multiplied by w.
            %       * bk : is a three-dimensional matrix in which every 'slice' is
            %         related to a k-prediction. This term (premultiplied by x0) has
            %         to be substructed to the known term.
            
            % preallocation for code speed up
            if(nargin>1)
                bk = zeros(length(Hc(:, 1)), length(Phi(1, :)), k0);
                [m, n] = size(L);
                Rk = zeros(m, n, k0);
                for k = 1:k0 % for each prediction compute a Rk, bk
                    bk(:, :, k) = (Hc*(Phi^k));
                    
                    Rk_temp = zeros(length(Phi), length(G(1, :)));
                    for j = 0:k-1
                        Rk_temp = Rk_temp + (Phi^j)*G;
                    end
                    Rk(:, :, k) = Hc*Rk_temp + L;
                end
            else
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
            end
        end % private methods
    end
end

