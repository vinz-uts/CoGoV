% Copyright 2021 - CoGoV.
% Licensed under the Academic Free License, Version 3.0 (the "License");
% you may not use this file except in compliance with the License.
% You may obtain a copy of the License at
% https://opensource.org/license/afl-3-0-php/
% Unless required by applicable law or agreed to in writing, software
% distributed under the License is distributed on an "AS IS" BASIS,
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
% See the License for the specific language governing permissions and
% limitations under the License.
% 
% Authors: Vincenzo D'Angelo, Ayman El Qemmah, Franco Angelo Torchiaro
% Credits: Alessandro Casavola, Francesco Tedesco


classdef CommandGovernor < handle
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
            % compute_cmd - calculate the reference g.
            % Calculate the nearest reference g to r start from initial
            % conditions x.
            
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
        
        function [Rk, bk] = compute_matrix(obj, Phi, Hc, G, L, k0)
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

