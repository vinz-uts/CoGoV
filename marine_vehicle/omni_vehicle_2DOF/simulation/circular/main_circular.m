%% Clear workspace
clear all;
close all;

%% Load vehicles' model matrices
addpath('../../marine_vehicle');        addpath(genpath('../../util'));
addpath(genpath('../../tbxmanager'));   addpath('../../CG');

%% Comment/Uncomment to choose precompensation technique
vehicle_2DOF_model_2 % R-stability controller (continuous time desing)

% vehicle_2DOF_model % LQI controller (discrete time design)

%% Vehicles
N = 3; % number of vehicles

% Vehicle 1
vehicle{1} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{1}.init_position(1,0.5);
% Vehicle 2
vehicle{2} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{2}.init_position(1.5,0.75);
% Vehicle 3
vehicle{3} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{3}.init_position(0.5,0);

%%%%%%% Position and input constraints
Hc = [ eye(2)        zeros(2,4)      ;
        -F              f            ];  
L = zeros(4,2);


vehicle{1}.ctrl_sys.Hc = Hc;
vehicle{1}.ctrl_sys.L = L;

vehicle{2}.ctrl_sys.Hc = Hc;
vehicle{2}.ctrl_sys.L = L;

vehicle{3}.ctrl_sys.Hc = Hc;
vehicle{3}.ctrl_sys.L = L;
%%%%%%


%% Net configuration
%   1
%  / \
% 2   3
adj_matrix = [-1  1  1;
    1 -1  1;
    1 1 -1];

%% Vehicles constraints
% Vehicles swarm position constraints
% ||(x,y)_i-(x,y)_j||∞ ≤ d_max
% ||(x,y)_i-(x,y)_j||∞ ≥ d_min
d_max = 200; % maximum distance between vehicles - [m]
d_min = 0.1; % minimum distance between vehicles - [m] Con 0.2 gli da come riferimento [0 0]

% Vehicles input/speed constraints
T_max = 100; % max abs of motor thrust - [N]

%% Command Governor parameters
Psi = eye(2); % vehicle's references weight matrix
k0 = 10; % prediction horizon

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
    
    vehicle{i}.cg = DistribuitedCommandGovernor(Phi,G,Hc,L,T,gi,U,hi,Psi,k0,'gurobi');
end

%% Planner

center = [0,0.5]';
center2 = [0.5,0]';
center3 = [-0.5,0];

pl(1) = CircularPlanner(center, 1, 0.4, 1);
pl(2) =  CircularPlanner(center2, 1, 0.3, -1);
pl(3) =  CircularPlanner(center3, 1, 0.4, 1);

% Color the net
colors = [0,1];
vehicle{1}.color = colors(1);
vehicle{2}.color = colors(2);
vehicle{3}.color = colors(2);

%% Simulation Colored Round CG
Tf = 200; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time

NT = ceil(Tf/Tc_cg); % simulation steps number

figure(1);
hold on;
axis([-5, 5, -5, 5]);

dist = [];

round = 1;
for t=1:NT
    for i=1:N
        if vehicle{i}.color == colors(round)
            x = vehicle{i}.ctrl_sys.sys.xi; % vehicle current state
            xc = vehicle{i}.ctrl_sys.xci; % controller current state
            xa = [x;xc];
            g_n = [];
            for j=1:N
                if adj_matrix(i,j) == 1 % i,j is neighbour
                    g_n = [g_n;vehicle{j}.g];
                    x = vehicle{j}.ctrl_sys.sys.xi; % vehicle current state
                    xc = vehicle{j}.ctrl_sys.xci; % controller current state
                    xa = [xa;x;xc];
                end
            end
            
            
            plan = pl(i);
            [r{i}, pl(i)] = plan.compute_reference(vehicle{i}.ctrl_sys.sys);
            g = vehicle{i}.cg.compute_cmd(xa, r{i}, g_n);
            
            if ~isempty(g)
                vehicle{i}.g = g;
            else
                disp('WARN: old references');
                t,i
            end
        end
    end
    
    
    for i=1:N
        vehicle{i}.ctrl_sys.sim(vehicle{i}.g,Tc_cg);
    end
    round = rem(round,length(colors))+1;

    for k=1:N
        % Trajectory
        figure(1);  hold on;
        if(k==1)
            plot(vehicle{k}.ctrl_sys.sys.x(1,:),vehicle{k}.ctrl_sys.sys.x(2,:),'b.');
        end
        
        if(k==2)
            
            plot(vehicle{k}.ctrl_sys.sys.x(1,:),vehicle{k}.ctrl_sys.sys.x(2,:),'r.');
        end
        
        if(k==3)
            plot(vehicle{k}.ctrl_sys.sys.x(1,:),vehicle{k}.ctrl_sys.sys.x(2,:),'k.');
        end
        
        
    end
    
    if(t==1)
        legend('Trajectory v1', 'Trajectory v2','Trajectory v3','AutoUpdate','off');
        title('Intersected Circular Scenario Simulation');
        xlabel('x [m]');
        ylabel('y [m]');
    end
    drawnow;
    
    dist=[dist, norm((vehicle{1}.ctrl_sys.sys.x(1:2,end)-vehicle{2}.ctrl_sys.sys.x(1:2,end)))];
end

figure;
plot(0:Tc_cg:Tf-Tc_cg, dist);
title('Distance between vehicle 1 and vehicle 2');
xlabel('time [s]');
ylabel('distance [m]');




