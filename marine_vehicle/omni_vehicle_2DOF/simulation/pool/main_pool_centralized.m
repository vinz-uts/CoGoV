%%% Pool scenario

% We consider 3 vehicles moving in a pool starting from different initial positions and moving
% along straight line. When hitting the boundaries of the pool the new vehicle's trajectory starts
% with the same strike angle. Vehicles have to avoid collision among them.
% The topology is described by an Incidence matrix introduced below.


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
vehicle{1}.init_position(0.5,-1);
% Vehicle 2
vehicle{2} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{2}.init_position(0,1);
% Vehicle 3
vehicle{3} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{3}.init_position(0,-1);


%% Net configuration
%   1
%  / \
% 2   3
adj_matrix = [-1 1  1;
    1 -1  0;
    1  0 -1];

%% Vehicles constraints
% Vehicles swarm position constraints
% ||(x,y)_i-(x,y)_j||∞ ≤ d_max
% ||(x,y)_i-(x,y)_j||∞ ≥ d_min
d_max = 200; % maximum distance between vehicles - [m]
d_min = 0.3; % minimum distance between vehicles - [m] Con 0.2 gli da come riferimento [0 0]

% Vehicles input/speed constraints
Max_x = 2; % max position value along x - [m]
Max_y = 2; % max position value along y - [m]
T_max = 20; % max abs of motor thrust - [N]



%% Command Governor parameters
% Vehicle's references weight matrix
Psi = [ 1  0  ;
    0  1   ];
% Psi_ = repmat(Psi,1,N);
Psitot=[];

for i=1:N
    Psitot= blkdiag(Psitot,Psi);
end

Psi = Psitot;
%Psi = blkdiag(Psi, Psi,Psi);
k0 = 10; % prediction horizon


%% Augmented System construction
% Augmented neighbour matrix
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

% Speed and thrust constraints
% T_*c_ ? gi_       single vehicle constraints
%      x  y  ? Vx Vy V? Tx Ty T?
T_ = [ 1  0  0  0  0  0 ;
    -1 0  0  0  0  0 ;
    0  1  0  0  0  0 ;
    0 -1  0  0  0  0 ;
    0  0  0  0  1  0 ;
    0  0  0  0 -1  0 ;
    0  0  0  0  0  1 ;
    0  0  0  0  0 -1 ];

gi_ = [Max_x,Max_x,Max_y,Max_y,T_max,T_max,T_max,T_max]';

T = [T_ zeros(size(T_,1),nca-nc); T];   gi = [gi_;gi];

cg = CentralizedCommandGovernor(Phi,G,Hc,L,T,gi,U,hi,Psi,k0,'gurobi');

%% Planner

vehicle{1}.planner =  Border_Planner([Max_x, Max_y], 0.5, 'radius', 1);
vehicle{2}.planner =  Border_Planner([Max_x, Max_y], -0.7, 'radius', 1);
vehicle{3}.planner =  Border_Planner([Max_x, Max_y], 0.1, 'radius', 1);

% Color the net
colors = [0,1];
vehicle{1}.color = colors(1);
vehicle{2}.color = colors(2);
vehicle{3}.color = colors(2);

plot_color = ['b', 'g', 'k'];


%% Simulation Colored Round CG
Tf = 30; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time
NT = ceil(Tf/Tc_cg); % simulation steps number

%%% Set blockedd to true to ensure that vehicle 3 stands still
blockedd=false;
reference3= vehicle{3}.ctrl_sys.sys.xi;

% Perimeter plotting

x_plot = -Max_x:0.1:Max_x;
y_plot = -Max_y:0.1:Max_y;

axis([-Max_x-1, Max_x + 1, -Max_y - 1, Max_y + 1]);

% Vector needed to track distance between vehicles
dist12=[];
dist23=[];
dist13=[]; 

hold on;
nr = size(reference3(1:2),1); % size of single vehicle reference
r=[];

round = 1;

for t=1:NT
    for i=1:N
        r{i} =  vehicle{i}.planner.compute_reference(vehicle{i}, []);
        % If we want the third vehicle to stand still
        % we set variable blockedd to true
        if(i==3 && blockedd)
            r{3}=reference3(1:2);
        end
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
    % Live plot
    figure(1);  clf;
    
    hold on;
    
    % Left side plot
    plot(ones(size(y_plot))*x_plot(1), y_plot,'k');
    % Lower side plot
    plot(x_plot, ones(size(x_plot))*y_plot(1), 'k');
    % Right side plot
    plot(ones(size(y_plot))*x_plot(end), y_plot, 'k');
    % Upper side plot
    plot(x_plot, ones(size(x_plot))*y_plot(end), 'k');
    
    
    for k=1:N

        % Plot vehicles trajectory
        plot(vehicle{k}.ctrl_sys.sys.x(1,:),vehicle{k}.ctrl_sys.sys.x(2,:), strcat(plot_color(k), '-.'),'LineWidth',0.8);
        
        axis([-Max_x-1, Max_x + 1, -Max_y - 1, Max_y + 1]);

        % Plot vehicles position 
        plot(vehicle{k}.ctrl_sys.sys.x(1,end),vehicle{k}.ctrl_sys.sys.x(2,end), strcat(plot_color(k), 'o'),'MarkerFaceColor',plot_color(k),'MarkerSize',7);
        
        % Plot vehicles CG references
        plot(vehicle{k}.g(1), vehicle{k}.g(2), strcat(plot_color(k), 'x'));
    end
    
    if(t==1)
        title('Pool Scenario Simulation');
        xlabel('x [m]');
        ylabel('y [m]');
    end
    
    drawnow;
    dist12 =[dist12, norm((vehicle{1}.ctrl_sys.sys.x(1:2,end)-vehicle{2}.ctrl_sys.sys.x(1:2,end)))];
    dist23 =[dist23, norm((vehicle{2}.ctrl_sys.sys.x(1:2,end)-vehicle{3}.ctrl_sys.sys.x(1:2,end)))];
    dist13 =[dist13, norm((vehicle{1}.ctrl_sys.sys.x(1:2,end)-vehicle{3}.ctrl_sys.sys.x(1:2,end)))];
end

figure;
subplot(3, 1, 1);
plot(0:Tc_cg:Tf-Tc_cg, dist12);
title('Distance between vehicle 1 and vehicle 2');
xlabel('time [s]');
ylabel('distance [m]');

subplot(3, 1, 2);
plot(0:Tc_cg:Tf-Tc_cg, dist23);
title('Distance between vehicle 2 and vehicle 3');
xlabel('time [s]');
ylabel('distance [m]');

subplot(3, 1, 3);
plot(0:Tc_cg:Tf-Tc_cg, dist13);
title('Distance between vehicle 1 and vehicle 3');
xlabel('time [s]');
ylabel('distance [m]');
