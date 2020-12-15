%% Clear workspace
clear all;
close all;

%% Load vehicles' model matrices 
addpath(genpath('../../../../util'));


vehicle_2DOF_model_2

%% Vehicles
N = 2; % number of vehicles

% Vehicle 1
vehicle{1} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{1}.init_position(1,0);
% Vehicle 2
vehicle{2} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{2}.init_position(0,1);

%% Net configuration
%   1
%  / \
% 2   3
adj_matrix = [-1  1;
			   1 -1];


%% Vehicles constraints
% Vehicles swarm position constraints
% ||(x,y)_i-(x,y)_j|| > d_min
d_min = 0.3; % minimum distance between vehicles - [m]

% Vehicles torque constraints
T_max = 20; % max abs of motor thrust - [N]

%% Command Governor parameters
Psi = 0.01*eye(2); % vehicle's references weight matrix
k0 = 10; % prediction horizon

%% Dynamic Command Governor
for i=1:N
    vehicle{i}.cg = DynamicDistribuitedCommandGovernor(i,Phi,G,Hc,L,Psi,k0, 'gurobi');
    vehicle{i}.cg.add_vehicle_cnstr('thrust',T_max);
    for j=1:N
        if adj_matrix(i,j) == 1 % i,j is neighbour
            vehicle{i}.cg.add_swarm_cnstr(j,'anticollision',d_min);
        end
    end
end


%% Color the net
colors = [0,1];
vehicle{1}.color = colors(1);
vehicle{2}.color = colors(2);

plot_color = ['b', 'g'];

%% Simulation Colored Round CG
Tf = 10; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time
NT = ceil(Tf/Tc_cg); % simulation steps number
round = 1;

%%%%% Data collection about optimization time %%%%%%%%
% names are self-explanatory
cputime =[];
yalmiptime = [];

dist = []; %%% check for collition constraints

%%%%%%%%%% Frontal Collision References
r{1} = vehicle{2}.ctrl_sys.sys.xi(1:2);
r{2} = vehicle{1}.ctrl_sys.sys.xi(1:2);

vehicle{1}.planner = LinePlanner(r{1}, 'radius', 0.6);
vehicle{2}.planner = LinePlanner(r{2}, 'radius', 0.6);

%%%%%% Sim cycle %%%%%%%%%

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
            
            r = vehicle{i}.planner.compute_reference(vehicle{i},xa); % perdo riferimento virtuale
            
            [g,s] = vehicle{i}.cg.compute_cmd(xa, r, g_n);
                        
            if ~isempty(g)
                vehicle{i}.g = g;
                %%%%%%%%%%%%
                
                %%%%% Data collection of optimization times %%%%%%
                cputime= [cputime,s.solvertime];
                yalmiptime=[yalmiptime,s.yalmiptime];
                %%%%%%%%%%%%%%
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
    
     
    % Live plot
    figure(1);  clf;
    
    hold on;
    
    
    %%%%%%% live plot %%%%%%%
    for k=1:N

    	% Plot vehicles trajectory
        plot(vehicle{k}.ctrl_sys.sys.x(1,:),vehicle{k}.ctrl_sys.sys.x(2,:), strcat(plot_color(k), '-.'),'LineWidth',0.8);
        
        % Plot vehicles position 
        plot(vehicle{k}.ctrl_sys.sys.x(1,end),vehicle{k}.ctrl_sys.sys.x(2,end), strcat(plot_color(k), 'o'),'MarkerFaceColor',plot_color(k),'MarkerSize',7);
        
        % Plot vehicles CG references
        plot(vehicle{k}.g(1), vehicle{k}.g(2), strcat(plot_color(k), 'x'));
        
    end
    
    if(t==1)
        title('Frontal Collision Simulation');
        xlabel('x [m]');
        ylabel('y [m]');
    end
    
    drawnow;
    %%%%%%%%%%%%%
    
    
    dist = [dist, norm((vehicle{1}.ctrl_sys.sys.x(1:2,end)-vehicle{2}.ctrl_sys.sys.x(1:2,end)))];
end


figure;
plot(1:NT, dist);
hold on;
plot(1:NT,d_min*ones(1,length(1:NT)));
title('Distance between vehicles');
xlabel('time [s]');
ylabel('distance [m]');

