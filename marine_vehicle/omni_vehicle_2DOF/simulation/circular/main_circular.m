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

center = [0,0.5]';
center2 = [0.5,0]';
center3 = [-0.5,0];


xSamples = [1, 0, -1, 0]';
ySamples = [0, 1, 0, -1]';

 

ptp1 = Polar_trajectory_planner(xSamples, ySamples);
ptp2 = Polar_trajectory_planner(xSamples, ySamples);
ptp3 = Polar_trajectory_planner(xSamples, ySamples);

 
ptp1.transform(1, center);
ptp2.transform(1, center2);
ptp3.transform(1, center3);

 

pl(1) = ptp1;
pl(2) =  ptp2;
pl(3) =  ptp3;

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
            r{i}= plan.compute_reference(vehicle{i}.ctrl_sys.sys);
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




