function [Phi_,G_,Hc_,L_,T,gi,U,hi] = untitled(Phi,G,Hc,L,adj_matrix,i,varargin)
%% build_aug - Build the augmented system of a vehicle
%  Build the augmented system of the vehicle 'i' in a swarm.
%  {Φ,G,Hc,L} := matrices of a vehicle.
%  adj_matrix := matrix of the network.
%  cnstr := [d_min,d_max,vx_max,vy_max,T_max].
%  i := ID vehicle.

    vargs = varargin;
    nargs = length(vargs);
    names = vargs(1:2:nargs);
    values = vargs(2:2:nargs);

    validnames = {'d_max','d_min','v_max','T_max'};    
    for name = names
       validatestring(name{:}, validnames);
       i = strmatch(name{:}, names);
       eval(sprintf('%s=%d;',name{:},values{i}));
    end
    if exist('d_max')
        
    elseif exist('d_min')
        
    elseif exist('v_max')
        
    elseif exist('T_max')
        
    else
        
    end
    
    N = size(adj_matrix,1); % Numbers of vehicles
    
    %% Augmented System construction
    % Vehicle 'i'
    % Augmented neighbour matrix
    % | Φ(i)            |
    % |      Φ(j1)      |
    % |           Φ(jm) |
    Phi_ = Phi;    G_ = G;    Hc_ = Hc;    L_ = L;

    for j=1:N
        if adj_matrix(i,j) == 1 % i,j is neighbour
            Phi_ = blkdiag(Phi_,Phi);
            G_   = blkdiag(G_,G);
            Hc_  = blkdiag(Hc_,Hc);
            L_   = blkdiag(L_,L);
        end
    end

    %% Constraints construction
    % Position constraints
    % ||(x,y)_i-(x,y)_j||∞ ≤ d_max
    % ||(x,y)_i-(x,y)_j||∞ ≥ d_min
    % Speed constraints
    % vx_i ≤ vx_max
    % vy_i ≤ vy_max
    % 
    
    nc = size(Hc,1); % single vehicle c dimension
    nca = size(Hc_,1); % vehicle 'i' augmented-c dimension
    
        T = [];     gi = [];
        U = [];     hi = [];

        k = 0; % neighbour number
        for j=1:N
            if adj_matrix(i,j) == 1 % i,j is neighbour
                k = k+1;
                % Split ||.||∞
                cnstr = zeros(4,nca);
                % split modules x constraints
                % |-- i --       -- j --     |
                % | 1 0 ..  ...  -1 0 ..  ...|
                % |-1 0 ..  ...   1 0 ..  ...|
                cnstr(1,1) = 1;
                cnstr(1,(k*nc)+1) = -1;
                cnstr(2,1) = -1;
                cnstr(2,(k*nc)+1) = 1;
                % split modules y constraints
                % |-- i --       -- j --     |
                % |0  1 ..  ...  0 -1 ..  ...|
                % |0 -1 ..  ...  0  1 ..  ...|
                cnstr(3,2) = 1;
                cnstr(3,(k*nc)+2) = -1;
                cnstr(4,2) = -1;
                cnstr(4,(k*nc)+2) = 1;

                % Matrix for neighbour remoteness constraints
                % T*c ≤ gi
                if ~isempty(T)
                    T = [T;cnstr];
                    gi = [gi;[d_max,d_max,d_max,d_max]'];
                else
                    T = cnstr;
                    gi = [d_max,d_max,d_max,d_max]';
                end

                % Matrix for neighbour proximity constraints
                % U*c ≤ hi (row-by-row OR-ed constrains)
                if ~isempty(U)
                    U = [U;cnstr];
                    hi = [hi;[d_min,d_min,d_min,d_min]'];
                else
                    U = cnstr;
                    hi = [d_min,d_min,d_min,d_min]';
                end
            end
        end

        % thrust constraints
        % T_*c_ ≤ gi_       single vehicle constraints
        %      x  y    Tx Ty
        T_ = [ 
               0  0    1  0 ;
               0  0   -1  0 ;
               0  0    0  1 ;
               0  0    0 -1 ];
        gi_ = [T_max,T_max,T_max,T_max]';

        Ta = T_;    ga = gi_;
        for j=1:k
            Ta = blkdiag(Ta,T_);
            ga = [ga;gi_];
        end
        T = [Ta;T];     gi = [ga;gi];
    end


