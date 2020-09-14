%%
% Simulation of a potential collision where vehicle 1 and vehicle 2 have to
% swap their relative positions


%% Clear workspace
clear all;
close all;

%% Load vehicles' model matrices
addpath('../../marine_vehicle');        addpath(genpath('../../util'));
addpath(genpath('../../tbxmanager'));   addpath('../../CG');

%% Comment/Uncomment to choose precompensation technique
vehicle_2DOF_model_2 % R-stability controller (continuous time desing)

% vehicle_2DOF_model % LQI controller (discrete time design)


%%%%%%% Position and input constraints
Hc = [ eye(2)        zeros(2,4)      ;
        -F              f            ];  
L = zeros(4,2);
%%%%%%

%% Vehicles
N = 2; % number of vehicles
% Vehicle 1
vehicle{1} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{1}.init_position(1.3,0);
% Vehicle 2
vehicle{2} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{2}.init_position(1.6,0.6);



%% Net configuration
%  1-2
adj_matrix = [-1  1 ;
    1 -1 ];


%% Vehicles constraints
% Vehicles swarm position constraints
% ||(x,y)_i-(x,y)_j||∞ ≤ d_max
% ||(x,y)_i-(x,y)_j||∞ ≥ d_min
d_max = 200; % maximum distance between vehicles - [m]
d_min = 0.3; % minimum distance between vehicles - [m] Con 0.2 gli da come riferimento [0 0]

% Vehicles constraints

T_max = 20; % max abs of motor thrust - [N]

%% Command Governor parameters
% Vehicle's references weight matrix
Psi = [ 1  0 ;
    0  1  ];
Psitot=[];

for i=1:N
    Psitot= blkdiag(Psitot,Psi);
end

Psi = Psitot;
k0 = 10; % prediction horizon
%% Augmented System construction
% Augmented neighbour matrix
% | ?(1)           |
% |      ?(i)      |
% |           ?(N) |
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
			% Split ||.||?
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
            % T*c ? gi
            if ~isempty(T)
                T = [T;cnstr];
                gi = [gi;[d_max,d_max,d_max,d_max]'];
            else
                T = cnstr;
                gi = [d_max,d_max,d_max,d_max]';
            end
            
            % Matrix for neighbour proximity constraints
            % U*c ? hi (row-by-row OR-ed constrains)
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
%      x  y    Tx Ty
T_ = [
    0  0    1  0 ;
    0  0   -1  0 ;
    0  0    0  1 ;
    0  0    0 -1 ];
gi_ = [T_max,T_max,T_max,T_max]';

    
Ta = [];    ga = [];
for j=1:N
    Ta = blkdiag(Ta,T_);
    ga = [ga;gi_];
end
T = [Ta;T];     gi = [ga;gi];

cg = CentralizedCommandGovernor(Phi,G,Hc,L,T,gi,U,hi,Psi,k0,'gurobi');


%% Planner

center = [0,0]';

pl(1) =  CircularPlanner(center, 1.3, 0.4, 1);
pl(2) =  CircularPlanner(center, 1.3, 0.6, -1);

% Color the net
colors = [0,1];
vehicle{1}.color = colors(1);
vehicle{2}.color = colors(2);
zerr = [0,0]';

%% Simulation Colored Round CG
Tf = 75; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time

NT = ceil(Tf/Tc_cg); % simulation steps number
zerr = [0,0]';
figure(1);
hold on;
axis([-3, 3, -3,3]);

dist = [];

round = 1;

nr = 2; % size of single vehicle reference

r=[];

for t=1:NT
    for i=1:N
        plan = pl(i);
        
        r{i} = plan.compute_reference(vehicle{i}.ctrl_sys.sys);
       
    end
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
    
    g = cg.compute_cmd(xa,r_);
    
    if ~isempty(g)
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
    
    round = rem(round,length(colors))+1;
    
    for k=1:N
        % Trajectory
        figure(1);  hold on;
        
        if(k==1)
            plot(vehicle{1}.ctrl_sys.sys.x(1,:),vehicle{1}.ctrl_sys.sys.x(2,:),'b.');
        end
        
        if(k==2)
            
            plot(vehicle{2}.ctrl_sys.sys.x(1,:),vehicle{2}.ctrl_sys.sys.x(2,:),'r.');
        end
        
        
    end
    if(t==1)
        legend('Trajectory v1', 'Trajectory v2','AutoUpdate','off');
        title('Concentric Circular Scenario Simulation');
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
