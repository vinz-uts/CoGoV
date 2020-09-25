classdef DynamicDistribuitedCommandGovernor < CommandGovernor
    %% DYNAMIC DISTRIBUITED COMMAND GOVERNOR
    %  Distribuite Command Governor for dynamic multi-agents nets. Computes
    %  the nearest reference g to r that statify local and global (with
    %  other system) constraints.
    
    properties
        id % vehicle id
        %Phi % closed-loop model Φ matrix
        %G % closed-loop model G matrix
        %Hc % closed-loop model Hc matrix
        %L % closed-loop model L matrix
        %T % constraints matrix
        %gi % constraints vector
        U % proximity constraints matrix - matrix for OR-ed constraints
        hi % proximity constraints vector - vector for OR-ed constraints
        %Psi % reference weight Ψ matrix
        %k0 % prediction steps number
        %solver_name % name of the numerical solver
        neigh % neighbours list
    end
    
    properties(Access = protected)
        %Rk % Rk matrix for code speed up
        %bk % bk matrix for code speed up
        nc % size of single vehicle constraints vector
    end
    
    methods
        function obj = DynamicDistribuitedCommandGovernor(id,Phi,G,Hc,L,Psi,k0,solver)
            % DynamicDistribuitedCommandGovernor - Constructor
            % Create an instance of a Dynamic Distribuited Command Governor.
            % Φ,G,Hc,L are the matrices of the single vehicle
            obj = obj@CommandGovernor(Phi,G,Hc,L,[],[],Psi,k0);
            obj.id = id;
            obj.nc = size(Hc,1);
            obj.U = [];
            obj.hi = [];
            obj.neigh = [];
            if nargin > 7 
                obj.check_solver(solver);
            end
        end
        
        
        function [g, ris] = compute_cmd(obj,x,r,g_n)
            % compute_cmd - calculate the reference g.
            % Calculate the nearest reference g to r start from initial
            % global conditions x and g_n reference for the other systems.
            g = sdpvar(length(r),1);
            w = [g;g_n];
            b = binvar(size(obj.U,1)*obj.k0,1);
            d = binvar(size(obj.U,1),1);
            mu = 10000;
            cnstr = obj.T*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) <= obj.gi;
            for i=1:(size(obj.U,1)/4)
                cnstr = [cnstr obj.U((i-1)*4+1,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+1)-mu*d((i-1)*4+1)];
                cnstr = [cnstr obj.U((i-1)*4+2,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+2)-mu*d((i-1)*4+2)];
                cnstr = [cnstr obj.U((i-1)*4+3,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+3)-mu*d((i-1)*4+3)];
                cnstr = [cnstr obj.U((i-1)*4+4,:)*((obj.Hc/(eye(size(obj.Phi,1))-obj.Phi)*obj.G+obj.L)*w) >= obj.hi((i-1)*4+4)-mu*d((i-1)*4+4)];
                cnstr = [cnstr sum( d((i-1)*4+(1:4)) ) <= 3];
            end
            
            %%%% old code uncomment to test
%             xk = x;
            %%%
            
            for k = 1:obj.k0
                %%%% old code uncomment to test
%                 xk = obj.Phi*xk+obj.G*w;  % xk = (obj.Phi)^k * x0 + sum(i=1,k-1) (obj.Phi^i*obj.G)*w
%                 cnstr = [cnstr obj.T*(obj.Hc*xk+obj.L*w) <= obj.gi];
                %%%%
                
                cnstr = [cnstr obj.T*(obj.Rk(:, :, k)*w) <= obj.gi - obj.T*obj.bk(:, :, k)*x];
                for i=1:(size(obj.U,1)/4)
                    %%%% old code uncomment to test
%                     cnstr = [cnstr (obj.U((i-1)*4+1,:)*(obj.Hc*xk+obj.L*w)) >= obj.hi((i-1)*4+1)-mu*b((k-1)*size(obj.U,1)+(i-1)*4+1)]; % se i vicini
%                     cnstr = [cnstr (obj.U((i-1)*4+2,:)*(obj.Hc*xk+obj.L*w)) >=  obj.hi((i-1)*4+2)-mu*b((k-1)*size(obj.U,1)+(i-1)*4+2)];
%                     cnstr = [cnstr (obj.U((i-1)*4+3,:)*(obj.Hc*xk+obj.L*w)) >=  obj.hi((i-1)*4+3)-mu*b((k-1)*size(obj.U,1)+(i-1)*4+3)];
%                     cnstr = [cnstr (obj.U((i-1)*4+4,:)*(obj.Hc*xk+obj.L*w)) >=  obj.hi((i-1)*4+4)-mu*b((k-1)*size(obj.U,1)+(i-1)*4+4)];
%                   %%%%%%%

                    cnstr = [cnstr (obj.U((i-1)*4+1,:)*(obj.Rk(:, :, k)*w)) >= obj.hi((i-1)*4+1)-mu*b((k-1)*size(obj.U,1)+(i-1)*4+1) - obj.U((i-1)*4+1,:)*obj.bk(:, :, k)*x]; % se i vicini
                    cnstr = [cnstr (obj.U((i-1)*4+2,:)*(obj.Rk(:, :, k)*w)) >=  obj.hi((i-1)*4+2)-mu*b((k-1)*size(obj.U,1)+(i-1)*4+2) - obj.U((i-1)*4+2,:)*obj.bk(:, :, k)*x];
                    cnstr = [cnstr (obj.U((i-1)*4+3,:)*(obj.Rk(:, :, k)*w)) >=  obj.hi((i-1)*4+3)-mu*b((k-1)*size(obj.U,1)+(i-1)*4+3) - obj.U((i-1)*4+3,:)*obj.bk(:, :, k)*x];
                    cnstr = [cnstr (obj.U((i-1)*4+4,:)*(obj.Rk(:, :, k)*w)) >=  obj.hi((i-1)*4+4)-mu*b((k-1)*size(obj.U,1)+(i-1)*4+4) - obj.U((i-1)*4+4,:)*obj.bk(:, :, k)*x];
                    
                    cnstr = [cnstr sum( b((k-1)*size(obj.U,1)+(i-1)*4+(1:4)) )<= 3];
                end
            end
            
            % Objective function
            obj_fun = (r-g)'*obj.Psi*(r-g);
            
            % Solver options
            assign(g, r); % initial guessing (possible optimization speed up)
            
            options = sdpsettings('verbose',0,'solver',obj.solver_name,'usex0',1,'cachesolvers',1);
            
            ris = optimize(cnstr,obj_fun,options);
            g = double(g);
            
            if(ris.problem ~= 0)
                fprintf("WARN: Problem %d \n %s\n", ris.problem, ris.info);
                g = [];
            end
            
            clear('yalmip');
        end
    
        
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
%%
                idx = strmatch(name{:},validnames);
                for i=1:length(values{pos})
                    cnstr((i-1)*2+1,(idx-1)*obj.nc/3+i) =  1;
                    cnstr((i-1)*2+2,(idx-1)*obj.nc/3+i) = -1;
                end
%                 if strcmp(name{:},validnames{1}) % Position constraints
%                     for i=1:length(values{pos})
%                         cnstr((i-1)*2+1,i) =  1;
%                         cnstr((i-1)*2+2,i) = -1;
%                     end
%                 elseif strcmp(name{:},validnames{2}) % Speed constraints
%                     for i=1:length(values{pos})
%                         cnstr((i-1)*2+1,obj.nc/3+i) =  1;
%                         cnstr((i-1)*2+2,obj.nc/3+i) = -1;
%                     end
%                 elseif strcmp(name{:},validnames{3}) % Thrust constraints
%                     for i=1:length(values{pos})
%                         cnstr((i-1)*2+1,2*obj.nc/3+i) =  1;
%                         cnstr((i-1)*2+2,2*obj.nc/3+i) = -1;
%                     end
%                 end
                gi_ = repmat(values{pos},2,1);
                T = [ T ; cnstr];   gi = [ gi ; gi_(:)];
            end
            obj.T  = [obj.T ; T zeros(size(T,1),obj.nc*length(obj.neigh))];
            obj.gi = [obj.gi; gi];
        end
        
        function add_swarm_cnstr(obj,id,varargin)
            % add_swarm_cnstr - add 2 vehicles constraints
            % id := neighbour's ID
            % varargin := 'proximity',    d_max, ...
            %             'anticollision',d_min
            T = [];     gi = [];
            validnames = {'proximity','anticollision'};
            
            nargs = length(varargin);
            params = varargin(1:2:nargs);   values = varargin(2:2:nargs);
            %...working
        end
        
        function add_anticollision_cnstr(obj)
            
        end
        
    end
    
end
