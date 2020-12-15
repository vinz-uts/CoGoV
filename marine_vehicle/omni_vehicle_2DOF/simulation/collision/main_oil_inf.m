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
vehicle{1}.init_position(1,0.5);
% Vehicle 2
vehicle{2} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{2}.init_position(0.1,0.2);


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
d_min = 0.2; % minimum distance between vehicles - [m] Con 0.2 gli da come riferimento [0 0]

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
            % Uncomment to avoid collision
           vehicle{i}.cg.add_swarm_cnstr(j,'anticollision',d_min);
        end
    end
end


%% Planner
%%%%%%%%%%%%%  Loading data in order simulate the oil stain

% Loading set of points 
load('xSamples','xSamples');
load('ySamples','ySamples');
load('xSamples2','xSamples2');
load('ySamples2','ySamples2');
load('xSamples2a','xSamples2a');
load('ySamples2a','ySamples2a');
load('xSamples3','xSamples3');
load('ySamples3','ySamples3');


% We change dynamically the planner
ptp1_1 = Polar_trajectory_planner(xSamples, ySamples);
ptp1_2 = Polar_trajectory_planner(xSamples2, ySamples2);
ptp1_3 = Polar_trajectory_planner(xSamples2a, ySamples2a);
ptp1_4 = Polar_trajectory_planner(xSamples3, ySamples3, 'rec_from_collision', false);

ptp1 = [ptp1_1,ptp1_2, ptp1_3 ,ptp1_4];

ptp2_1 = Polar_trajectory_planner(xSamples, ySamples);
ptp2_2 = Polar_trajectory_planner(xSamples2, ySamples2);
ptp2_3 = Polar_trajectory_planner(xSamples2a, ySamples2a);
ptp2_4 = Polar_trajectory_planner(xSamples3, ySamples3, 'recovery', 20, 'rec_from_collision', false);
ptp2 = [ptp2_1,ptp2_2,ptp2_3,ptp2_4];

%%%%%% end of loading data section for planners 
vehicle{1}.planner =  ptp1(1);
vehicle{2}.planner =  ptp2(1);

% Color the net
colors = [0,1];
vehicle{1}.color = colors(1);
vehicle{2}.color = colors(2);

plot_color = ['b', 'g'];


%% Simulation Colored Round CG
Tf = 50; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time

theta = 0:0.1:2*pi;

NT = ceil(Tf/Tc_cg); % simulation steps number
dist = [];

r{1}=vehicle{1}.ctrl_sys.sys.xi(1:2);
r{2}=vehicle{2}.ctrl_sys.sys.xi(1:2);

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
            
            %%% Planners are manually changed in order to emulate the
            % changing shape of the oil stain
            
            if(t==95)
                vehicle{1}.planner= ptp1(2);
                vehicle{2}.planner = ptp2(2);
            end
            
            if(t==130)
                vehicle{1}.planner = ptp1(3);
                vehicle{2}.planner= ptp2(3);
            end
            
            if(t==150)
                vehicle{1}.planner = ptp1(4);
                vehicle{2}.planner = ptp2(4);
            end
            
            r{i} = vehicle{i}.planner.compute_reference(vehicle{i}, xa);
            
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
  
    figure(1);
    clf;
    plot(vehicle{1}.planner, 'k');
    hold on;
    plot(vehicle{2}.planner, 'k');
    
    for k=1:N
        % Trajectory
        plot_2Dof_vehicle(vehicle{k}, r{i}, d_min,'Color', plot_color(k))
        
        
        if(t==1)
            legend('Trajectory v1', 'Trajectory v2','AutoUpdate','off');
            title('Concentric Circular Scenario Simulation');
            xlabel('x [m]');
            ylabel('y [m]');
        end
        
        
    end
    drawnow;
    dist=[dist, norm((vehicle{1}.ctrl_sys.sys.x(1:2,end)-vehicle{2}.ctrl_sys.sys.x(1:2,end)))];
end

figure;
plot(0:Tc_cg:Tf-Tc_cg, dist);
title('Distance between vehicle 1 and vehicle 2');
xlabel('time [s]');
ylabel('distance [m]');

