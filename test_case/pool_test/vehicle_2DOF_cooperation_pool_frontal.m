%% Clear workspace
clear all;
close all;

%% Load vehicles' model matrices 
addpath('../../marine_vehicle');        addpath(genpath('../../util'));
addpath(genpath('../../tbxmanager'));   addpath('../../CG');

vehicle_2DOF_model

%% Vehicles
N = 3; % number of vehicles
%for i=1:N
	% Vehicle 1
	vehicle{1} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
    vehicle{1}.init_position(1,0);
	% Vehicle 2
	vehicle{2} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
    vehicle{2}.init_position(0,1);
	% Vehicle 3
	vehicle{3} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
    vehicle{3}.init_position(0,-1);
%end

%% Net configuration
%   1
%  / \
% 2   3
adj_matrix = [-1  1  0;
			   1 -1  0;
			   0 0 -1];

%% Vehicles constraints
% Vehicles swarm position constraints
% ||(x,y)_i-(x,y)_j||∞ ≤ d_max
% ||(x,y)_i-(x,y)_j||∞ ≥ d_min
d_max = 200; % maximum distance between vehicles - [m]
d_min = 0.3; % minimum distance between vehicles - [m] Con 0.2 gli da come riferimento [0 0]

% Vehicles input/speed constraints
Max_x = 2; % max abs of speed along x - [m/s]
Max_y = 2; % max abs of speed along y - [m/s]
T_max = 20; % max abs of motor thrust - [N]

%% Command Governor parameters
Psi = eye(2); % vehicle's references weight matrix
k0 = 35; % prediction horizon

%% Augmented System and Command Governor construction
for i=1:N
	% Vehicle i
    % Augmented neighbour matrix
    % | Φ(i)            |
    % |      Φ(j1)      |
    % |           Φ(jm) |
    Phi = vehicle{i}.ctrl_sys.Phi;
	G = vehicle{i}.ctrl_sys.G;
	Hc = vehicle{i}.ctrl_sys.Hc;
	L = vehicle{i}.ctrl_sys.L;
    
    for j=1:N
        if adj_matrix(i,j) == 1 % i,j is neighbour
			Phi = blkdiag(Phi,vehicle{j}.ctrl_sys.Phi);
			G = blkdiag(G,vehicle{j}.ctrl_sys.G);
			Hc = blkdiag(Hc,vehicle{j}.ctrl_sys.Hc);
			L = blkdiag(L,vehicle{j}.ctrl_sys.L);
        end
    end
    
    nc = size(vehicle{i}.ctrl_sys.Hc,1); % single vehicle c dimension
    nca = size(Hc,1); % vehicle i augmented-c dimension
    % Constraints construction
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
    
    % geometric and thrust constraints
    % T_*c_ ≤ gi_       single vehicle constraints
    %      x  y Vx Vy Tx Ty
    T_ = [ 1  0  0  0  0  0 ;
           -1 0  0  0  0  0 ;
           0  1  0  0  0  0 ;
           0 -1  0  0  0  0 ;
           0  0  0  0  1  0 ;
           0  0  0  0 -1  0 ;
           0  0  0  0  0  1 ;
           0  0  0  0  0 -1 ];
    gi_ = [Max_x,Max_x,Max_y,Max_y,T_max,T_max,T_max,T_max]';
    
    Ta = T_;    ga = gi_;
    for j=1:k
        Ta = blkdiag(Ta,T_);
        ga = [ga;gi_];
    end
    T = [Ta;T];     gi = [ga;gi];

    vehicle{i}.cg = DistribuitedCommandGovernor(Phi,G,Hc,L,T,gi,U,hi,Psi,k0);
end
