%% Clear workspace
clear all;
close all;

%% Load vehicles' model matrices 
addpath('../../marine_vehicle');        addpath(genpath('../../util'));
addpath(genpath('../../tbxmanager'));   addpath('../../CG');
vehicle_3DOF_model

%% Vehicles
N = 3; % number of vehicles
%for i=1:N
	% Vehicle 1
	vehicle{1} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
    vehicle{1}.init_position(1,0,0);
	% Vehicle 2
	vehicle{2} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
    vehicle{2}.init_position(0,1,0);
	% Vehicle 3
	vehicle{3} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
    vehicle{3}.init_position(0,-1,0);
%end

%% Net configuration
%   1
%  / \
% 2   3
adj_matrix = [-1  1  1;
			   1 -1  0;
			   1  0 -1];

%% Vehicles constraints
% Vehicles swarm position constraints
% ||(x,y)_i-(x,y)_j||∞ ≤ d_max
% ||(x,y)_i-(x,y)_j||∞ ≥ d_min
d_max = 2; % maximum distance between vehicles - [m]
d_min = 1; % minimum distance between vehicles - [m]

% Vehicles input/speed constraints
Vx = 2; % max abs of speed along x - [m/s]
Vy = 2; % max abs of speed along y - [m/s]
Vt = pi; % max abs of speed around z - [rad/s]
T_max = 100; % max abs of motor thrust - [N]

%% Command Governor parameters
% Vehicle's references weight matrix
Psi = [ 1  0  0 ;
        0  1  0 ;
        0  0  10 ];
Psi_ = repmat({Psi},1,N);   Psi = blkdiag(Psi_);
k0 = 10; % prediction horizon

%% Augmented System construction
% Augmented neighbour matrix
% | Φ(1)           |
% |      Φ(i)      |
% |           Φ(N) |
Phi = [];    G = [];    Hc = [];    L = [];
%Phi = vehicle{1}.ctrl_sys.Phi;
%G = vehicle{1}.ctrl_sys.G;
%Hc = vehicle{1}.ctrl_sys.Hc;
%L = vehicle{1}.ctrl_sys.L;
for i=1:N
    Phi = blkdiag(Phi,vehicle{i}.ctrl_sys.Phi);
	G = blkdiag(G,vehicle{i}.ctrl_sys.G);
	Hc = blkdiag(Hc,vehicle{i}.ctrl_sys.Hc);
	L = blkdiag(L,vehicle{i}.ctrl_sys.L);
end

%% Constraints construction
T = [];     gi = [];
U = [];     hi = [];
    
nc = size(vehicle{1}.ctrl_sys.Hc,1); % single vehicle c dimension
nca = nc*N; % augmented-c dimension

% Collision avoidance and proximity constraints
for i=1:N
    for j=i+1:N
        if adj_matrix(i,j) == 1 % i,j are neighbours
			% Split ||.||∞
			cnstr = zeros(4,nca);
            % split modules x constraints
            % |     -- i --       -- j --     |
            % |...   1 0 ..  ...  -1 0 ..  ...|
            % |...  -1 0 ..  ...   1 0 ..  ...|
            cnstr(1,((i-1)*nc)+1) = 1; 
            cnstr(1,((j-1)*nc)+1) = -1;
            cnstr(2,((i-1)*nc)+1) = -1; 
            cnstr(2,((j-1)*nc)+1) = 1;
            % split modules y constraints
            % |     -- i --       -- j --     |
            % |...  0  1 ..  ...  0 -1 ..  ...|
            % |...  0 -1 ..  ...  0  1 ..  ...|
            cnstr(3,((i-1)*nc)+2) = 1; 
            cnstr(3,((j-1)*nc)+2) = -1;
            cnstr(4,((i-1)*nc)+2) = -1; 
            cnstr(4,((j-1)*nc)+2) = 1;
            
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
end
    
% Speed and thrust constraints
% T_*c_ ≤ gi_       single vehicle constraints
%      x  y  ϑ Vx Vy Vϑ Tx Ty Tϑ
T_ = [ 0  0  0  1  0  0  0  0  0 ;
       0  0  0 -1  0  0  0  0  0 ;
       0  0  0  0  1  0  0  0  0 ;
       0  0  0  0 -1  0  0  0  0 ;
       0  0  0  0  0  1  0  0  0 ;
       0  0  0  0  0 -1  0  0  0 ;
       0  0  0  0  0  0  1  0  0 ;
       0  0  0  0  0  0 -1  0  0 ;
       0  0  0  0  0  0  0  1  0 ;
       0  0  0  0  0  0  0 -1  0 ;
       0  0  0  0  0  0  0  0  1 ;
       0  0  0  0  0  0  0  0 -1 ];
gi_ = [Vx,Vx,Vy,Vy,Vt,Vt,T_max,T_max,T_max,T_max,T_max,T_max]';
    
Ta = [];    ga = [];
for j=1:N
    Ta = blkdiag(Ta,T_);
    ga = [ga;gi_];
end
T = [Ta;T];     gi = [ga;gi];

cg = CentralizedCommandGovernor(Phi,G,Hc,L,T,gi,U,hi,Psi,k0);


%% Simulation
Tf = 5; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time
r{1} = [4,0.5,0]'; % position references
r{2} = [3,1,0]'; % position references
r{3} = [3,-1.5,0]'; % position references
NT = ceil(Tf/Tc_cg); % simulation steps number
epsilon = 0.1; % nearness precision
nr = size(r{1},1); % size of single vehicle reference

for t=1:NT
    xa = [];
    for i=1:N
        x = vehicle{i}.ctrl_sys.sys.xi; % vehicle current state
        xc = vehicle{i}.ctrl_sys.xci; % controller current state
        xa = [xa;x;xc];
        % Modify: modify the angular reference after the cg
        if norm([vehicle{i}.g(1)-x(1) vehicle{i}.g(2)-x(2)]) > epsilon
            r{i}(3) = atan2(vehicle{i}.g(2)-x(2),vehicle{i}.g(1)-x(1));
        end
        
        r_ = [];
        for j=1:N
            r_ = [r_;r{j}];
        end
        g = cg.compute_cmd(xa,r_);
        if ~isempty(g)
            vehicle{i}.g = g(((i-1)*nr)+1:((i-1)*nr)+nr);
        else
            disp('WARN: old references');
            t,i
        end
    end
    
    for i=1:N
        vehicle{i}.ctrl_sys.sim(vehicle{i}.g,Tc_cg);
    end

end


