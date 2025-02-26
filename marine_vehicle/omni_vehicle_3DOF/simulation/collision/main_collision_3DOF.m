clear;
close all;
%%
% Simulation of a potential collision where vehicle 1 and vehicle 2 have to
% swap their relative positions


%% Load vehicles' model matrices 
% addpath('../../marine_vehicle');        addpath(genpath('../../util'));
% addpath(genpath('../../tbxmanager'));   addpath('../../CG');

%% Comment/Uncomment to choose precompensation technique
vehicle_3DOF_model_2 % R-stability controller (continuous time desing)

% vehicle_3DOF_model % LQI controller (discrete time design)


%% Vehicles
N = 2; % number of vehicles

% Vehicle 1
vehicle{1} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{1}.init_position(1,0,0);
% Vehicle 2
vehicle{2} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{2}.init_position(0,1,0);


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

% Vehicles input/speed constraints
T_max = 100; % max abs of motor thrust - [N]

%% Command Governor parameters
Psi = 1*eye(3); % vehicle's references weight matrix
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
% Color the net
colors = [0,1];
vehicle{1}.color = colors(1);
vehicle{2}.color = colors(2);


%% Simulation Colored Round CG
Tf = 10; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time
NT = ceil(Tf/Tc_cg); % simulation steps number
figure(1);
hold on;
axis([-3,3, -3, 3]);
dist = [];
round = 1;
cputime =[];
yalmiptime = [];

%%%%%%%%%% Crossed Collision References

% r{1}= [1,1.5,0]';
% r{2}= [1.5,1,0]';

%%%%%%%%%% Frontal Collision References

% Uncomment to test
ref1=vehicle{2}.ctrl_sys.sys.xi;
ref2=vehicle{1}.ctrl_sys.sys.xi;

r{1}= ref1(1:3);
r{2}= ref2(1:3);




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

            
            [g,s] = vehicle{i}.cg.compute_cmd(xa, r{i}, g_n);
            
            %%%%%%%% to check incorrect zero reference computed by CG
            if(i==1)
                plot(r{1}(1), r{1}(2), 'bo');
                if(not(isempty(g)))
                    plot(g(1), g(2), 'rx');
                end
            end
            if(i==2)
                plot(r{2}(1), r{2}(2), 'go');
                if(not(isempty(g)))
                    plot(g(1), g(2), 'bx');
                end
            end
            
            if ~isempty(g)
                vehicle{i}.g = g;
                cputime= [cputime,s.solvertime];
                yalmiptime=[yalmiptime,s.yalmiptime];
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
        figure(1);
        hold on;
        if(k==1)
            plot_trajectory(vehicle{1}.ctrl_sys.sys.x(1,:),vehicle{1}.ctrl_sys.sys.x(2,:),vehicle{1}.ctrl_sys.sys.x(3,:));
            plot(vehicle{1}.ctrl_sys.sys.x(1,end),vehicle{1}.ctrl_sys.sys.x(2,end),'o');
        else
            plot_trajectory(vehicle{2}.ctrl_sys.sys.x(1,:),vehicle{2}.ctrl_sys.sys.x(2,:),vehicle{2}.ctrl_sys.sys.x(3,:));
            plot(vehicle{2}.ctrl_sys.sys.x(1,end),vehicle{2}.ctrl_sys.sys.x(2,end),'x');
        end
        
    end
    
    if(t==1)
        title('Frontal Collision Simulation');
        xlabel('x [m]');
        ylabel('y [m]');
    end
    
    drawnow;
    dist = [dist, norm((vehicle{1}.ctrl_sys.sys.x(1:2,end)-vehicle{2}.ctrl_sys.sys.x(1:2,end)))];
end


figure;
plot(1:NT, dist);
hold on;
plot(1:NT,0.3*ones(1,length(1:NT)));
title('Distance between vehicles');
xlabel('time [s]');
ylabel('distance [m]');
