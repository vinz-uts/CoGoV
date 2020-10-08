classdef DynamicDistribuitedCommandGovernor < DistribuitedCommandGovernor
    %% DYNAMIC DISTRIBUITED COMMAND GOVERNOR
    %  Distribuite Command Governor for dynamic multi-agents nets. Computes
    %  the nearest reference g to r that statify local and global (with
    %  other system) constraints.
    
    properties
        id % vehicle id
        Phi_ % single vehicle model Φ matrix
        G_ % single vehicle model G matrix
        Hc_ % single vehicle model Hc matrix
        L_ % single vehicle model L matrix
        %T % constraints matrix
        %gi % constraints vector
        %U % proximity constraints matrix - matrix for OR-ed constraints
        %hi % proximity constraints vector - vector for OR-ed constraints
        %Psi % reference weight Ψ matrix
        %k0 % prediction steps number
        %solver_name % name of the numerical solver
        neigh % neighbours list
        datacheck % matrices useful for check function
    end
    
    properties(Access = protected)
        %Rk % Rk matrix for code speed up
        %bk % bk matrix for code speed up
        nc % size of single vehicle constraints vector
    end
    
    methods
        function obj = DynamicDistribuitedCommandGovernor(id,Phi,G,Hc,L,Psi,k0,solver)
            % DynamicDistribuitedCommandGovernor - Constructor
            % Create an instance of a DynamicDistribuitedCommandGovernor.
            % Φ,G,Hc,L are the matrices of the single vehicle
            obj = obj@DistribuitedCommandGovernor(Phi,G,Hc,L,[],[],[],[],Psi,k0);
            obj.Phi_ = Phi;     obj.G_ = G;
            obj.Hc_ = Hc;       obj.L_ = L;
            obj.id = id;
            obj.nc = size(Hc,1);
            obj.U = [];
            obj.hi = [];
            obj.neigh = [];
            if nargin > 7
                obj.check_solver(solver);
            end
            cnstr = zeros(4,2*obj.nc);
            cnstr(1,1) =  1;    cnstr(1,obj.nc+1) = -1;
            cnstr(2,1) = -1;    cnstr(2,obj.nc+1) =  1;
            cnstr(3,2) =  1;    cnstr(3,obj.nc+2) = -1;
            cnstr(4,2) = -1;    cnstr(4,obj.nc+2) =  1;
            
            Phi_t = blkdiag(Phi,Phi);
            G_t = blkdiag(G,G);
            L_t = blkdiag(L,L);
            Hc_t = blkdiag(Hc,Hc);
            [Rk_t, bk_t] = obj.compute_matrix(Phi_t, Hc_t, G_t, L_t, k0);
            obj.datacheck = struct('Phi',Phi_t,'G',G_t,'L',L_t,'Hc',Hc_t,'U',cnstr,'k0',k0,'Rk',Rk_t,'bk',bk_t);
            
        end
        
        
        %         function [g, ris] = compute_cmd(obj,x,r,g_n)
        %             % compute_cmd - calculate the reference g.
        %             % Calculate the nearest reference g to r start from initial
        %             % global conditions x and g_n reference for the other systems.
        %             g = sdpvar(length(r),1);
        %             w = [g;g_n];
        %             b = binvar(size(obj.U,1)*obj.k0,1);
        %             d = binvar(size(obj.U,1),1);
        %             mu = 10000;
        %             cnstr = obj.T*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) <= obj.gi;
        %             for i=1:(size(obj.U,1)/4)
        %                 cnstr = [cnstr obj.U((i-1)*4+1,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+1)-mu*d((i-1)*4+1)];
        %                 cnstr = [cnstr obj.U((i-1)*4+2,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+2)-mu*d((i-1)*4+2)];
        %                 cnstr = [cnstr obj.U((i-1)*4+3,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+3)-mu*d((i-1)*4+3)];
        %                 cnstr = [cnstr obj.U((i-1)*4+4,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+4)-mu*d((i-1)*4+4)];
        %                 cnstr = [cnstr sum( d((i-1)*4+(1:4)) ) <= 3];
        %             end
        %
        %             for k = 1:obj.k0
        %                 cnstr = [cnstr obj.T*(obj.Rk(:, :, k)*w) <= obj.gi - obj.T*obj.bk(:, :, k)*x];
        %                 for i=1:(size(obj.U,1)/4)
        %                     cnstr = [cnstr (obj.U((i-1)*4+1,:)*(obj.Rk(:, :, k)*w)) >= obj.hi((i-1)*4+1)-mu*b((k-1)*size(obj.U,1)+(i-1)*4+1) - obj.U((i-1)*4+1,:)*obj.bk(:, :, k)*x];
        %                     cnstr = [cnstr (obj.U((i-1)*4+2,:)*(obj.Rk(:, :, k)*w)) >= obj.hi((i-1)*4+2)-mu*b((k-1)*size(obj.U,1)+(i-1)*4+2) - obj.U((i-1)*4+2,:)*obj.bk(:, :, k)*x];
        %                     cnstr = [cnstr (obj.U((i-1)*4+3,:)*(obj.Rk(:, :, k)*w)) >= obj.hi((i-1)*4+3)-mu*b((k-1)*size(obj.U,1)+(i-1)*4+3) - obj.U((i-1)*4+3,:)*obj.bk(:, :, k)*x];
        %                     cnstr = [cnstr (obj.U((i-1)*4+4,:)*(obj.Rk(:, :, k)*w)) >= obj.hi((i-1)*4+4)-mu*b((k-1)*size(obj.U,1)+(i-1)*4+4) - obj.U((i-1)*4+4,:)*obj.bk(:, :, k)*x];
        %                     cnstr = [cnstr sum( b((k-1)*size(obj.U,1)+(i-1)*4+(1:4)) )<= 3];
        %                 end
        %             end
        %
        %             % Objective function
        %             obj_fun = (r-g)'*obj.Psi*(r-g);
        %
        %             % Solver options
        %             assign(g, r); % initial guessing (possible optimization speed up)
        %
        %             options = sdpsettings('verbose',0,'solver',obj.solver_name,'usex0',1,'cachesolvers',1);
        %
        %             ris = optimize(cnstr,obj_fun,options);
        %             g = double(g);
        %
        %             if(ris.problem ~= 0)
        %                 fprintf("WARN: Problem %d \n %s\n", ris.problem, ris.info);
        %                 g = [];
        %             end
        %
        %             clear('yalmip');
        %         end
        
        
        function add_vehicle_cnstr(obj,varargin)
            % add_vehicle_cnstr - add single vehicle constraints
            % varargin := 'position',[x_max,y_max,ϑ_max], ...
            %             'speed',   [vx_max,vy_max,vϑ_max], ...
            %             'thrust',  [Tx_max,Ty_max,Tϑ_max]
            T = [];     gi = [];
            validnames = {'position','speed','thrust'};
            
            nargs = length(varargin);
            params = varargin(1:2:nargs);   values = varargin(2:2:nargs);
            
            for name = params
                validatestring(name{:}, validnames); % raise an Exception if the name is not valid
                pos = strmatch(name{:}, params);
                if length(values{pos}) == 1
                    values{pos} = repmat(values{pos},2,1);
                end
                cnstr = zeros(2*length(values{pos}),obj.nc);
                idx = strmatch(name{:},validnames);
                for i=1:length(values{pos})
                    cnstr((i-1)*2+1,(idx-1)*obj.nc/3+i) =  1;
                    cnstr((i-1)*2+2,(idx-1)*obj.nc/3+i) = -1;
                end
                gi_ = repmat(values{pos},2,1);
                T = [ T ; cnstr];   gi = [ gi ; gi_(:)];
            end
            obj.T  = [obj.T ; T zeros(size(T,1),obj.nc*length(obj.neigh))];
            obj.gi = [obj.gi; gi];
            
            [obj.Rk, obj.bk] = obj.compute_matrix();
        end
        
        function [rs, rns] = compute_virtual_cmd(obj,r,g_n,dmax,dmin) %,xn1)
            %%% rs is the virtual reference for vehicle i 
            %%% rns is the virtuaal reference for vehicle N+1
            %%% r is equal to previous reference g(t-1)
            w1 = sdpvar(length(r),1);
            wn1 = sdpvar(length(r),1);
            w = [w1;g_n];
            d = binvar(size(obj.U,1),1);
            mu = 10000;
            
            %%% We need to satisfy steady state constraints with neighboors
            cnstr = obj.T*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) <= obj.gi;
            
            cnstr = [cnstr obj.T(1:8,1:6)*((obj.Hc_/(eye(size(obj.Phi_,1))-obj.Phi_)*obj.G_+obj.L_)*wn1) <= obj.gi(1:8)];
%             xk = xn1;
%             
%              for k = 1:10
%                 %%%% old code uncomment to test
%                 xk = obj.Phi_*xk+obj.G_*wn1;  % xk = (obj.Phi)^k * x0 + sum(i=1,k-1) (obj.Phi^i*obj.G)*w
%                 cnstr = [cnstr obj.T(1:8,1:6)*(obj.Hc_*xk+obj.L_*wn1) <= obj.gi(1:8)];
%              end
%             
            for i=1:(size(obj.U,1)/4)
                cnstr = [cnstr obj.U((i-1)*4+1,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+1)-mu*d((i-1)*4+1)];
                cnstr = [cnstr obj.U((i-1)*4+2,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+2)-mu*d((i-1)*4+2)];
                cnstr = [cnstr obj.U((i-1)*4+3,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+3)-mu*d((i-1)*4+3)];
                cnstr = [cnstr obj.U((i-1)*4+4,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+4)-mu*d((i-1)*4+4)];
                cnstr = [cnstr sum( d((i-1)*4+(1:4)) ) <= 3];
            end
            
            %%% We need to satisfy steady state constraints with new
            %%% vehicle
            
            if(not(isempty(dmax)))
                check_gi = dmax*ones(size(obj.datacheck.U(:,1),1),1);
                cnstr = obj.datacheck.U*((obj.datacheck.Hc/(eye(size(obj.datacheck.Phi,1))-obj.datacheck.Phi)*obj.datacheck.G+obj.datacheck.L)*[w1;wn1]) <= check_gi;
            end
            
            if(not(isempty(dmin)))
                check_hi = dmin*ones(size(obj.datacheck.U(:,1),1),1);
                dnew = binvar(size(obj.datacheck.U,1),1);
                cnstr = [cnstr obj.datacheck.U(1,:)*((obj.datacheck.Hc/(eye(size(obj.datacheck.Phi,1))-obj.datacheck.Phi)*obj.datacheck.G+obj.datacheck.L)*[w1;wn1]) >= check_hi(1)-mu*dnew(1)];
                cnstr = [cnstr obj.datacheck.U(2,:)*((obj.datacheck.Hc/(eye(size(obj.datacheck.Phi,1))-obj.datacheck.Phi)*obj.datacheck.G+obj.datacheck.L)*[w1;wn1]) >= check_hi(2)-mu*dnew(2)];
                cnstr = [cnstr obj.datacheck.U(3,:)*((obj.datacheck.Hc/(eye(size(obj.datacheck.Phi,1))-obj.datacheck.Phi)*obj.datacheck.G+obj.datacheck.L)*[w1;wn1]) >= check_hi(3)-mu*dnew(3)];
                cnstr = [cnstr obj.datacheck.U(4,:)*((obj.datacheck.Hc/(eye(size(obj.datacheck.Phi,1))-obj.datacheck.Phi)*obj.datacheck.G+obj.datacheck.L)*[w1;wn1]) >= check_hi(4)-mu*dnew(4)];
                cnstr = [cnstr sum( dnew((1:4)) ) <= 3];
            end
            
             % Objective function
            obj_fun = (r-w1)'*obj.Psi*(r-w1);
             % Solver options
            assign(w1, r); % initial guessing (possible optimization speed up)
%             assign(wn1, r); % initial guessing (possible optimization speed up)
            
            options = sdpsettings('verbose',0,'solver',obj.solver_name,'usex0',1,'cachesolvers',1);
            
            ris = optimize(cnstr,obj_fun,options);
            rs = double(w1);
            rns = double(wn1);
            
            if(ris.problem ~= 0)
                fprintf("WARN: Problem %d \n %s\n", ris.problem, ris.info);
                rs = [];
                rns = [];
            end
            
            clear('yalmip');
        end
        
        function add_swarm_cnstr(obj,id,varargin)
            % add_swarm_cnstr - add 2 vehicles constraints
            % id := neighbour's ID
            % varargin := 'proximity',    d_max, ...
            %             'anticollision',d_min
            cnstr = zeros(4,2*obj.nc);
            cnstr(1,1) =  1;    cnstr(1,obj.nc+1) = -1;
            cnstr(2,1) = -1;    cnstr(2,obj.nc+1) =  1;
            cnstr(3,2) =  1;    cnstr(3,obj.nc+2) = -1;
            cnstr(4,2) = -1;    cnstr(4,obj.nc+2) =  1;
            
            validnames = {'proximity','anticollision'};
            
            nargs = length(varargin);
            params = varargin(1:2:nargs);   values = varargin(2:2:nargs);
            proximity_cnstr_flag = 0;
            
            for name = params
                validatestring(name{:}, validnames); % raise an Exception if the name is not valid
                pos = strmatch(name{:}, params);
                if strcmp(name{:},validnames{1}) % Proximity constraints
                    proximity_cnstr_flag = 1;
                    if ~isempty(obj.T)
                        obj.T  = [ obj.T zeros(size(obj.T,1),obj.nc);
                            cnstr(:,1:obj.nc) zeros(size(cnstr,1),size(obj.T,2)-obj.nc) cnstr(:,obj.nc+1:end) ];
                        obj.gi = [ obj.gi; values{pos}.*ones(4,1) ];
                    else
                        obj.T  = cnstr;
                        obj.gi = values{pos}.*ones(4,1);
                    end
                elseif strcmp(name{:},validnames{2}) % Anticollision constraints
                    if ~isempty(obj.U)
                        obj.U  = [ obj.U zeros(size(obj.U,1),obj.nc);
                            cnstr(:,1:obj.nc) zeros(size(cnstr,1),size(obj.U,2)-obj.nc) cnstr(:,obj.nc+1:end) ];
                        obj.hi = [ obj.hi; values{pos}.*ones(4,1) ];
                    else
                        obj.U = cnstr;
                        obj.hi = values{pos}.*ones(4,1);
                    end
                end
            end
            
            if ~proximity_cnstr_flag % extend T matrix
                obj.T = [ obj.T zeros(size(obj.T,1),obj.nc) ];
            end
            obj.Phi = blkdiag(obj.Phi,obj.Phi_);
            obj.G = blkdiag(obj.G,obj.G_);
            obj.Hc = blkdiag(obj.Hc,obj.Hc_);
            obj.L = blkdiag(obj.L,obj.L_);
            [obj.Rk, obj.bk] = obj.compute_matrix();
            
            obj.neigh = [obj.neigh id]; % add the 'id'-th vehicle to the neighbour list
        end
        
        
        function remove_swarm_cnstr(obj,id)
            % remove_swarm_cnstr - remove all constraints with vehicle 'id'
            % id := neighbour's ID
            ind = find(obj.neigh == id);
            if ind > 0
                col = ind*obj.nc+(1:obj.nc);
                [row,j] = find(obj.T(:,col)~=0);
                obj.T(:,col) = [];
                obj.T(row,:) = [];
                obj.gi(row) = [];
                
                [row,j] = find(obj.U(:,col)~=0);
                obj.U(:,col) = [];
                obj.U(row,:) = [];
                obj.hi(row) = [];
                
                n = size(obj.Phi,1);
                m = size(obj.G,2);  m_ = size(obj.G_,2);
                obj.Phi((n-obj.nc+1):n,:) = [];     obj.Phi(:,(n-obj.nc+1):n) = [];
                obj.G((n-obj.nc+1):n,:) = [];       obj.G(:,(m-m_+1):m) = [];
                obj.Hc((n-obj.nc+1):n,:) = [];      obj.Hc(:,(n-obj.nc+1):n) = [];
                obj.L((n-obj.nc+1):n,:) = [];       obj.L(:,(m-m_+1):m) = [];
                [obj.Rk, obj.bk] = obj.compute_matrix();
                
                obj.neigh(ind) = []; % remove the 'ind'-th neighbour form the list
            end
        end
        
        function pluggable = check(obj, x_i, x_n, g_i, g_n, dmax, dmin)
            w = [g_i;g_n];
            x = [x_i; x_n];
            
            if(not(isempty(dmax)))
                check_gi = dmax*ones(size(obj.datacheck.U(:,1),1),1);
                %%%% steady state vehicle constraints %%%%%
                if(not(obj.datacheck.U*((obj.datacheck.Hc/(eye(size(obj.datacheck.Phi,1))-obj.datacheck.Phi)*obj.datacheck.G+obj.datacheck.L)*w)  ...
                        <= check_gi))
                    pluggable = false;
                    return;
                end
                for k = 1:obj.datacheck.k0
                    if not(obj.datacheck.U*(obj.datacheck.Rk(:, :, k)*w) <= check_gi - obj.datacheck.U*obj.datacheck.bk(:, :, k)*x)
                        pluggable = false;
                        return;
                    end
                end
                %%%%%%%%%%%
            end
            
            if(not(isempty(dmin)))
                check_hi = dmin*ones(size(obj.datacheck.U(:,1),1),1);
                %%%%%%%% steady state swarm constraints %%%%%%%%
                for i=1:(size(obj.datacheck.U,1)/4)
                    if(not(obj.datacheck.U((i-1)*4+1,:)*((obj.datacheck.Hc/(eye(size(obj.datacheck.Phi,1))-obj.datacheck.Phi)*obj.datacheck.G+obj.datacheck.L)*w) >= check_hi((i-1)*4+1) || ...
                            obj.datacheck.U((i-1)*4+2,:)*((obj.datacheck.Hc/(eye(size(obj.datacheck.Phi,1))-obj.datacheck.Phi)*obj.datacheck.G+obj.datacheck.L)*w) >= check_hi((i-1)*4+2) ||...
                            obj.datacheck.U((i-1)*4+3,:)*((obj.datacheck.Hc/(eye(size(obj.datacheck.Phi,1))-obj.datacheck.Phi)*obj.datacheck.G+obj.datacheck.L)*w) >= check_hi((i-1)*4+3) ||...
                            obj.datacheck.U((i-1)*4+4,:)*((obj.datacheck.Hc/(eye(size(obj.datacheck.Phi,1))-obj.datacheck.Phi)*obj.datacheck.G+obj.datacheck.L)*w) >= check_hi((i-1)*4+4)))
                        pluggable = false;
                        return;
                    end
                end
                %%%%%%%%%%%%%%
                
                
                %%%%%%% transient constraints (both vehicle and swarm ones) %%%
                for k = 1:obj.datacheck.k0
                    for i=1:(size(obj.datacheck.U,1)/4)
                        if not((obj.datacheck.U((i-1)*4+1,:)*(obj.datacheck.Rk(:, :, k)*w)) >= check_hi((i-1)*4+1) - obj.datacheck.U((i-1)*4+1,:)*obj.datacheck.bk(:, :, k)*x ||...
                                (obj.datacheck.U((i-1)*4+2,:)*(obj.datacheck.Rk(:, :, k)*w)) >= check_hi((i-1)*4+2) - obj.datacheck.U((i-1)*4+2,:)*obj.datacheck.bk(:, :, k)*x ||...
                                (obj.datacheck.U((i-1)*4+3,:)*(obj.datacheck.Rk(:, :, k)*w)) >= check_hi((i-1)*4+3) - obj.datacheck.U((i-1)*4+3,:)*obj.datacheck.bk(:, :, k)*x ||...
                                (obj.datacheck.U((i-1)*4+4,:)*(obj.datacheck.Rk(:, :, k)*w)) >= check_hi((i-1)*4+4) - obj.datacheck.U((i-1)*4+4,:)*obj.datacheck.bk(:, :, k)*x )
                            pluggable = false;
                            return;
                        end
                    end
                end
                %%%%%%%%
                
            end
            %%% true if all constraints are satisfied %%%%%
            pluggable = true;
            
            
        end
        
        
    end
    
end
