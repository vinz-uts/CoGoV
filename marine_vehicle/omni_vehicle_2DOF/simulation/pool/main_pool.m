%%% Pool scenario

% We consider 3 vehicles moving in a pool starting from different initial positions and moving
% along straight line. When hitting the boundaries of the pool the new vehicle's trajectory starts
% with the same strike angle. Vehicles have to avoid collision among them.
% The topology is described by an Incidence matrix introduced below.


%% Clear workspace
clear all;
close all;

%% Load vehicles' model matrices
% addpath('../../marine_vehicle');        addpath(genpath('../../util'));
% addpath(genpath('../../tbxmanager'));   addpath('../../CG');

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
    1 -1  1;
    1  1 -1];

%% Vehicles constraints
% Vehicles swarm position constraints
% ||(x,y)_i-(x,y)_j||∞ ≤ d_max
% ||(x,y)_i-(x,y)_j||∞ ≥ d_min
d_max = 200; % maximum distance between vehicles - [m]
d_min = 0.3; % minimum distance between vehicles - [m] 

% Vehicles input/speed constraints
Max_x = 2; % max position value along x - [m]
Max_y = 2; % max position value along y - [m]
T_max = 50; % max abs of motor thrust - [N]

%% Command Governor parameters
Psi = eye(2); % vehicle's references weight matrix
k0 = 10; % prediction horizon

%% Dynamic Command Governor
for i=1:N
    vehicle{i}.cg = DynamicDistribuitedCommandGovernor(i,Phi,G,Hc,L,Psi,k0, 'gurobi');
    vehicle{i}.cg.add_vehicle_cnstr('position', [Max_x, Max_y],'thrust',T_max);
    for j=1:N
        if adj_matrix(i,j) == 1 % i,j is neighbour
            vehicle{i}.cg.add_swarm_cnstr(j,'proximity',d_max,'anticollision',d_min);
        end
    end
end

%% Planner
limits = [-Max_x, Max_x, Max_y, -Max_y];

pl(1) =  Border_Planner([Max_x, Max_y], 0.5, 'radius', 1);
pl(2) =  Border_Planner([Max_x, Max_y], -0.7, 'radius', 1);
pl(3) =  Border_Planner([Max_x, Max_y], 0.1, 'radius', 1);

% Color the net
colors = [0,1];
vehicle{1}.color = colors(1);
vehicle{2}.color = colors(2);
vehicle{3}.color = colors(2);

%% Simulation Colored Round CG
Tf = 150; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time
NT = ceil(Tf/Tc_cg); % simulation steps number

%%% Set blockedd to true to ensure that vehicle 3 stands still
blockedd=false;
reference3= vehicle{3}.ctrl_sys.sys.xi;

% Perimeter plotting
figure(1);
hold on;
x_plot = -Max_x:0.1:Max_x;
y_plot = -Max_y:0.1:Max_y;
plot(ones(size(y_plot))*x_plot(1), y_plot);
plot(x_plot, ones(size(x_plot))*y_plot(1));
plot(ones(size(y_plot))*x_plot(end), y_plot);
plot(x_plot, ones(size(x_plot))*y_plot(end));
axis([-Max_x-1, Max_x + 1, -Max_y - 1, Max_y + 1]);
dist=[];
hold on;

round = 1;

%% Simulation Cycle
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
            
            
            
            r = plan.compute_reference(vehicle{i}, xa);
            
            if(i==3 && blockedd)
                r=reference3(1:2);
            end
            
            
            g = vehicle{i}.cg.compute_cmd(xa, r, g_n);
            
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
            plot(vehicle{1}.ctrl_sys.sys.x(1,:),vehicle{1}.ctrl_sys.sys.x(2,:),'b.');
        end
        
        if(k==2)
            
            plot(vehicle{2}.ctrl_sys.sys.x(1,:),vehicle{2}.ctrl_sys.sys.x(2,:),'r.');
        end
        
        if(k==3)
            plot(vehicle{3}.ctrl_sys.sys.x(1,:),vehicle{3}.ctrl_sys.sys.x(2,:),'k.');
        end
        
        
    end
    if(t==1)
        legend('Left Border', 'Lower Border','Right Border', 'Upper Border','Trajectory v1', 'Trajectory v2','Trajectory v3','AutoUpdate','off');
        title('Pool Scenario Simulation');
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
