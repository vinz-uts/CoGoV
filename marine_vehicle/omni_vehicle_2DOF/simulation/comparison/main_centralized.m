%% Clear workspace
clear;  close all;

%%%%%%%%%%%%%%%%%%%%% configurazion parameters %%%%%%%%%%%%%%
N = 4; % number of vehicles
init_radius = 7;
random_reference = true;
parent_change_enabled = false;

%%%%%%%%%%%%%%%%%%%% computed parameters %%%%%%%%%%%%%
theta_step = 2*pi/N;

%% Load vehicles' model matrices
addpath('../../../../marine_vehicle');      addpath('../../../../marine_vehicle/omni_vehicle_2DOF');
addpath(genpath('../../../../util'));       addpath('../../../../CG');
addpath(genpath('../../../../tbxmanager'));

vehicle_2DOF_model_2

%%%%%%% Position and input constraints
Hc = [ eye(2)        zeros(2,4)      ;
    -F              f            ];
L = zeros(4,2);
%%%%%%


%%%% Simulation Settings
%% Init vehicles


vehicle = cell(1, N);
r = cell(1, N);

for i=1:N
    vehicle{i} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
    %     vehicle{N-i+1} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
    
    vehicle{i}.init_position(init_radius*cos(theta_step*(i-1)), init_radius*sin(theta_step*(i-1)));
    %     vehicle{N-i+1}.init_position(init_radius*cos(theta_step*(i-1) + pi), init_radius*sin(theta_step*(i-1) + pi));
    
    if(not(random_reference))
        r{i} = [init_radius*cos(theta_step*(i-1) + pi); init_radius*sin(theta_step*(i-1) + pi)];
        vehicle{i}.planner = LinePlanner(r{i}, 'radius', 1.2);
    end
end

if(random_reference)
    refs = zeros(2, N);
    for i = 1:N
        refs(:, i) = vehicle{i}.ctrl_sys.sys.xi(1:2);
    end
    
    for i = 1:N
        index = round((rand()*(length(refs(1, :))-1))) + 1;
        r{i} = refs(:, index);
        refs(:, index) = [];
        
        vehicle{i}.planner = LinePlanner(r{i}, 'radius', 1.2);
    end
end
%% Net configuration
spanning_tree = zeros(N, N);

adj_matrix = ones(N,N);

for i = 1:N
    adj_matrix(i,i) = -1;
end

spanning_tree(1, 1) = -1;
spanning_tree(1, 2) = 1;
for i = 2:N-1
    spanning_tree(i, i) = -1;
    spanning_tree(i, i-1) = 1;
    spanning_tree(i, i+1) = 1;
end
spanning_tree(N, N) = -1;
spanning_tree(N, N-1) = 1;

%%%%%%%%%%%%%

%%%%%%%%% parent change algorithm %%%%%%%%%%
% It is assumed that vehicle 1 is the root of the spanning tree, and cant
% change
vehicle{1}.parent = 0;

for i=2:N
    vehicle{i}.parent = i-1;
end

%% Vehicles constraints
% Vehicles swarm position constraints
% ||(x,y)_i-(x,y)_j|| < d_max
% ||(x,y)_i-(x,y)_j|| > d_min
d_max = 20;  % maximum distance between vehicles - [m]
d_min = 0.1; % minimum distance between vehicles - [m]

% Vehicles input/speed constraints
Vx = 0.7; % max abs of speed along x - [m/s]
Vy = 0.7; % max abs of speed along y - [m/s]
T_max = 15; % max abs of motor thrust - [N]

%%%%%%%%%%%%%%%%%

%% Command Governor parameters
% Vehicle's references weight matrix
Psi = [ 1  0 ;
    0  1  ];
Psi_ = repmat({Psi},1,N);   Psi = blkdiag(Psi_{:});

k0 = 10; % prediction horizon

%% Augmented System construction
% Augmented neighbour matrix
% | Φ(1)           |
% |      Φ(i)      |
% |           Φ(N) |
Phi = [];    G = [];    Hc = [];    L = [];
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

% thrust constraints
% T_*c_ ≤ gi_       single vehicle constraints
%      x  y  Tx Ty
T_ = [ 0  0  1  0 ;
    0  0 -1  0 ;
    0  0  0  1 ;
    0  0  0 -1 ];
gi_ = [T_max,T_max,T_max,T_max]';

for i=N:-1:1
    T = [zeros(size(T_,1),(i-1)*size(T_, 1)) T_ zeros(size(T_,1),nca-nc-(i-1)*size(T_, 1)); T];   gi = [gi_;gi];
end
cg = CentralizedCommandGovernor(Phi,G,Hc,L,T,gi,U,hi,Psi,k0,'gurobi');


%% Simulation Colored Round CG
Tf = 8; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time
NT = ceil(Tf/Tc_cg); % simulation steps number

%% Communication Layer Initilization
pl = Phisical_Layer(N, 200, 0, 1);
pl.update(vehicle, 0);

% Vector used to consider eventual virtual references coming from the plug
% in operation
virtual = zeros(N,1);

% Vectors needed to analyze computational aspects
cputime= [];
yalmiptime=[];

nr = size(r{1},1); % size of single vehicle reference


round = 1;
for t=1:NT
    xa = [];
    for i=1:N
        x = vehicle{i}.ctrl_sys.sys.xi; % vehicle current state
        xc = vehicle{i}.ctrl_sys.xci; % controller current state
        xa = [xa;x;xc];
        
        r_ = [];
        for j=1:N
            r_ = [r_;r{j}];
        end
        
    end
    [g,s] = cg.compute_cmd(xa,r_);
    
    if ~isempty(g)
        cputime= [cputime,s.solvertime];
        yalmiptime=[yalmiptime,s.yalmiptime];
        for i=1:N
            vehicle{i}.g = g(((i-1)*nr)+1:((i-1)*nr)+nr);
        end
    else
        disp('WARN: old references');
        t,i
    end
    
    
    for i=1:N
        vehicle{i}.ctrl_sys.sim(vehicle{i}.g,Tc_cg);
    end
    
    for j=2:N
        pl.send_packet(j, 1, []);
        if(pl.look_for_packets(1))
            pl.get_packet(1);
            pl.send_packet(1, j, []);
            if(pl.look_for_packets(j))
                pl.get_packet(j);
            end
        end
    end
    pl.update(vehicle, t*Tc_cg);
    
    figure(1);
    
    %     axis([0 40 0 40]);
    for k=1:N
        % Trajectory
        %         axis([0 5 -4 4])
        plot(vehicle{k}.ctrl_sys.sys.x(1,:),vehicle{k}.ctrl_sys.sys.x(2,:), strcat('k', '-.'),'LineWidth',0.8);
        hold on;
        axis([-10 10 -10 10]);
        plot(vehicle{k}.ctrl_sys.sys.x(1,end),vehicle{k}.ctrl_sys.sys.x(2,end), strcat('k', 'o'),'MarkerFaceColor','k','MarkerSize',7);
        %%%% live plot %%%%
        plot(r{k}(1), r{k}(2), strcat('k', 'o'));
        plot(vehicle{k}.g(1), vehicle{k}.g(2), strcat('k', 'x'));
    end
    plot(pl);
    hold off;
    drawnow;
    
end

mean(cputime) 

figure;
hold on;
plot(1:NT,cputime);
title('Distance between vehicles 1 and vehicle 2');
xlabel('time [s]');
ylabel('distance [m]');

