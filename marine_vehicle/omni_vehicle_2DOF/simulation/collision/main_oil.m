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
d_min = 0.05; % minimum distance between vehicles - [m] Con 0.2 gli da come riferimento [0 0]

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
%             vehicle{i}.cg.add_swarm_cnstr(j,'proximity',d_max);
            % Uncomment to avoid collision
            vehicle{i}.cg.add_swarm_cnstr(j,'proximity',d_max,'anticollision',d_min);
        end
    end
end
%% Planner
center = [0,0]';

% Loading set of points 
load('xSamples','xSamples');
load('ySamples','ySamples');
load('xSamples2','xSamples2');
load('ySamples2','ySamples2');
load('xSamples2a','xSamples2a');
load('ySamples2a','ySamples2a');
load('xSamples3','xSamples3');
load('ySamples3','ySamples3');

load('xSamplescirc1','xSamplescirc1');
load('ySamplescirc1','ySamplescirc1');
load('xSamplescirc2','xSamplescirc2');
load('ySamplescirc2','ySamplescirc2');

% xSamples = [1, 0, -1, 0]';
% ySamples = [0, 1, 0, -1]';

% We change dynamically the planner
ptp1_1 = Polar_trajectory_planner(xSamples, ySamples);
ptp1_2 = Polar_trajectory_planner(xSamples2, ySamples2);
ptp1_3 = Polar_trajectory_planner(xSamples2a, ySamples2a);
ptp1_4 = Polar_trajectory_planner(xSamples3, ySamples3);
ptp1_5 = Polar_trajectory_planner(xSamplescirc1, ySamplescirc1,'tolerance',0.1,'recovery',15,'rec_from_collision',true);
ptp1 = [ptp1_1,ptp1_2, ptp1_3 ,ptp1_4,ptp1_5];

ptp2_1 = Polar_trajectory_planner(xSamples, ySamples);
ptp2_2 = Polar_trajectory_planner(xSamples2, ySamples2);
ptp2_3 = Polar_trajectory_planner(xSamples2a, ySamples2a);
ptp2_4 = Polar_trajectory_planner(xSamples3, ySamples3,'tolerance',0.2,'recovery',6);
ptp2_5 = Polar_trajectory_planner(xSamplescirc2, ySamplescirc2,'tolerance',0.1 ,'recovery',15,'rec_from_collision',true);
ptp2_5.transform(1,[0.01,0]);
ptp2 = [ptp2_1,ptp2_2,ptp2_3,ptp2_4,ptp2_5];

pl(1) =  ptp1(1);
pl(2) =  ptp2(1);

% plot(ptp1, 'k');
% plot(ptp2, 'r');


% Color the net
colors = [0,1];
vehicle{1}.color = colors(1);
vehicle{2}.color = colors(2);


%% Simulation Colored Round CG
Tf = 50; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time

theta = 0:0.1:2*pi;

NT = ceil(Tf/Tc_cg); % simulation steps number
zerr = [0,0]';
figure(1);
hold on;
% 
% axis([0,1.5, 0,1.5]);

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
            if(t==95)
                pl(1) = ptp1(2);
                pl(2) = ptp2(2);
            end
            
            if(t==130)
                pl(1) = ptp1(3);
                pl(2) = ptp2(3);
            end
            
            if(t==150)
                pl(1) = ptp1(4);
                pl(2) = ptp2(4);
            end
            
            if(t==187)
                pl(1) = ptp1(5);
                pl(2) = ptp2(5);
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
%     
%     if(t==100)
%         pl(1) = ptp1(2);
%         pl(2) = ptp2(2);
%     end
%     
%     if(t==NT/2)
%         pl(1) = ptp1(3);
%         pl(2) = ptp2(3);
%     end

figure(1);

plot(pl(1), 'k');
hold on;
plot(pl(2), 'k');
hold on;
axis equal

for k=1:N
    % Trajectory
    
    
    if(k==1)
        plot(vehicle{k}.ctrl_sys.sys.x(1,:),vehicle{k}.ctrl_sys.sys.x(2,:), '-.r','LineWidth',0.8);
        plot(vehicle{k}.ctrl_sys.sys.x(1,end),vehicle{k}.ctrl_sys.sys.x(2,end), 'or','MarkerFaceColor','r','MarkerSize',10);
        x1 = vehicle{k}.ctrl_sys.sys.x(1,end) + d_min*cos(theta);
        y1 = vehicle{k}.ctrl_sys.sys.x(2,end) + d_min*sin(theta);
        plot(x1,y1,'--');
            plot(r(1), r(2), 'or');
            plot(vehicle{k}.g(1), vehicle{k}.g(2), 'xr');
            
        end
        
        if(k==2)
            plot(vehicle{k}.ctrl_sys.sys.x(1,:),vehicle{k}.ctrl_sys.sys.x(2,:), '-.k','LineWidth',0.8);
            plot(vehicle{k}.ctrl_sys.sys.x(1,end),vehicle{k}.ctrl_sys.sys.x(2,end), 'ok','MarkerFaceColor','k','MarkerSize',10);
            x1 = vehicle{k}.ctrl_sys.sys.x(1,end) + d_min*cos(theta);
            y1 = vehicle{k}.ctrl_sys.sys.x(2,end) + d_min*sin(theta);
            plot(x1,y1,'--');
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

