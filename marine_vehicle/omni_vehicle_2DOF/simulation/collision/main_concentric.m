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
N = 2; % number of vehicles

% Vehicle 1
vehicle{1} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{1}.init_position(1.3,0);
% Vehicle 2
vehicle{2} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{2}.init_position(-1,1);


%% Net configuration
%   1
%  / \
% 2   3
adj_matrix = [-1  1 ;
    1 -1] ;

%% Vehicles constraints
% Vehicles swarm position constraints
% ||(x,y)_i-(x,y)_j||∞ ≤ d_max
% ||(x,y)_i-(x,y)_j||∞ ≥ d_min
d_max = 200; % maximum distance between vehicles - [m]
d_min = 0.3; % minimum distance between vehicles - [m] Con 0.2 gli da come riferimento [0 0]

% Vehicles constraints
T_max = 100; % max abs of motor thrust - [N]                               
%% Command Governor parameters
Psi = eye(2); % vehicle's references weight matrix
k0 = 10; % prediction horizon

%% Dynamic Command Governor
for i=1:N
    vehicle{i}.cg = DynamicDistribuitedCommandGovernor(i,Phi,G,Hc,L,Psi,k0, 'gurobi');
    vehicle{i}.cg.add_vehicle_cnstr('thrust',T_max);
    for j=1:N
        if adj_matrix(i,j) == 1 % i,j is neighbour
            vehicle{i}.cg.add_swarm_cnstr(j,'proximity',d_max,'anticollision',d_min);
        end
    end
end
%% Planner
center = [0,0]';

xSamples = [1, 0.8, 0.5, 0, -0.25, -0.6, -0.75 -1, -1, -0.5, 0, 0.35, 0.7]';
ySamples = [0, 0.45, 0.7, 1, 0.9, 0.75, 0.3, 0, -0.5, -0.7, -1, -1, -0.45]';

% xSamples = [1, 0, -1, 0]';
% ySamples = [0, 1, 0, -1]';

ptp1 = Polar_trajectory_planner(1.3*xSamples, 1.3*ySamples, 'recovery', 15);

ptp2 = Polar_trajectory_planner(1.3*xSamples, 1.3*ySamples, 'clockwise', false, 'recovery', 15);

pl(1) =  ptp1;
pl(2) =  ptp2;

% plot(ptp1, 'k');
% plot(ptp2, 'r');


% Color the net
colors = [0,1];
vehicle{1}.color = colors(1);
vehicle{2}.color = colors(2);


%% Simulation Colored Round CG
Tf = 75; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time

NT = ceil(Tf/Tc_cg); % simulation steps number
zerr = [0,0]';
figure(1);
hold on;

axis([-3,3, -3,3]);

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
            
            r= plan.compute_reference(vehicle{i},xa);
            
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
    
    figure(1);  
    plot(ptp1, 'k');
    hold on;
    
    axis equal
    for k=1:N
        % Trajectory
        
        
        if(k==1)
            plot(vehicle{k}.ctrl_sys.sys.x(1,:),vehicle{k}.ctrl_sys.sys.x(2,:), '-.r','LineWidth',0.8);
            plot(vehicle{k}.ctrl_sys.sys.x(1,end),vehicle{k}.ctrl_sys.sys.x(2,end), 'or','MarkerFaceColor','r','MarkerSize',7);
            plot(r(1), r(2), 'or');
            plot(vehicle{k}.g(1), vehicle{k}.g(2), 'xr');
            
        end
        
        if(k==2)
            plot(vehicle{k}.ctrl_sys.sys.x(1,:),vehicle{k}.ctrl_sys.sys.x(2,:), '-.k','LineWidth',0.8);
            plot(vehicle{k}.ctrl_sys.sys.x(1,end),vehicle{k}.ctrl_sys.sys.x(2,end), 'ok','MarkerFaceColor','k','MarkerSize',7);
            plot(r(1), r(2), 'ok');
            plot(vehicle{k}.g(1), vehicle{k}.g(2), 'xk');
        end
        
        
    end
    hold off;
    
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

