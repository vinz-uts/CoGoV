%% Clear workspace
clear all;
close all;

%% Load vehicles' model matrices 

vehicle_2DOF_model_2

%% Vehicles
N = 4; % number of vehicles

% Vehicle 1
vehicle{1} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{1}.init_position(0.5,0);
% Vehicle 2
vehicle{2} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{2}.init_position(0,1);

vehicle{3} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{3}.init_position(0,-1);

vehicle{4} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{4}.init_position(1,2.2);
% end

%% Net configuration
%   1
%  / \
% 2   3
adj_matrix = [-1  1  1  0;
			   1 -1  0  0;
               1  0 -1  0;
               0  0  0 -1];

%% Vehicles constraints
% Vehicles swarm position constraints
% ||(x,y)_i-(x,y)_j||? ? d_max
% ||(x,y)_i-(x,y)_j||? ? d_min
d_max = 1.5; % maximum distance between vehicles - [m]
d_min = 0.3; % minimum distance between vehicles - [m]

% Vehicles input/speed constraints
Vx = 1; % max abs of speed along x - [m/s]
Vy = 1; % max abs of speed along y - [m/s]
T_max = 2; % max abs of motor thrust - [N]

%% Command Governor parameters
Psi = eye(2); % vehicle's references weight matrix
k0 = 10; % prediction horizon

%% Dynamic Command Governor
for i=1:N
    vehicle{i}.cg = DynamicDistribuitedCommandGovernor(i,Phi,G,Hc,L,Psi,k0, 'gurobi');
    vehicle{i}.cg.add_vehicle_cnstr('speed', [Vx, Vy]', 'thrust',T_max);
    for j=1:N
        if adj_matrix(i,j) == 1 % i,j is neighbour
            vehicle{i}.cg.add_swarm_cnstr(j,'proximity',d_max,'anticollision',d_min);
        end
    end
end


%% Color the net
colors = [0,1,2];
vehicle{1}.color = colors(1);
vehicle{2}.color = colors(2);
vehicle{3}.color = colors(2);
vehicle{4}.color = colors(3);

%% Simulation Colored Round CG
Tf = 10; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time
NT = ceil(Tf/Tc_cg); % simulation steps number
round = 1;

%%%%% Data collection about optimization time %%%%%%%%
% names are self-explanatory
cputime =[];
yalmiptime = [];

dist12 = []; %%% check for collition constraints
dist13 = []; %%% check for collition constraints
dist23 = []; %%% check for collition constraints

plot_color = ['b', 'g', 'k', 'r'];
plaggable = false;
alreadyplag = false;

%%%%%%%%%% Frontal Collision References
r{1} = vehicle{1}.ctrl_sys.sys.xi(1:2) + [2, 0]';
r{2} = vehicle{2}.ctrl_sys.sys.xi(1:2) + [2, 0]';
r{3} =  vehicle{3}.ctrl_sys.sys.xi(1:2) + [2, 0]';
% r{4} =  vehicle{4}.ctrl_sys.sys.xi(1:2) + [0, 2]';
r{4} = [1,-3]';

%%%%% Binary variables usefull to PnP operation
ask_to_freeze = [false, false, false, false]';
ask_to_plug = false;
pluggable = false;

%%%%% Virtual references 
r_stari = [];  
r_starn1 = [];

% %%%% Dynamic constraits management %%%%
% added13 = false;
% added23 = false;
% added12 = false;
% %%%%%%%%%%%

%%% Vector for distance between i and N+1 for plotting purposes

dist = [];


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
            
            [g,s] = vehicle{i}.cg.compute_cmd(xa, r{i}, g_n);
                        
            if ~isempty(g)
                vehicle{i}.g = g;
                %%%% live plot %%%%
                plot(r{i}(1), r{i}(2), strcat(plot_color(i), 'o'));
                plot(g(1), g(2), strcat(plot_color(i), 'x'));
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
    
    %%%%%%% live plot %%%%%%%
    for k=1:N
        % Trajectory
        figure(1);
        axis([0 5 -4 4]);
        hold on;
        plot(vehicle{k}.ctrl_sys.sys.x(1,:),vehicle{k}.ctrl_sys.sys.x(2,:), strcat(plot_color(k), '.'));
    end
    
    if(t==1)
        title('Frontal Collision Simulation');
        xlabel('x [m]');
        ylabel('y [m]');
    end
    dist = [dist, norm((vehicle{2}.ctrl_sys.sys.x(1:2,end)-vehicle{4}.ctrl_sys.sys.x(1:2,end)))];
    drawnow;
end



figure;
plot(1:NT, dist);
hold on;
plot(1:NT,d_min*ones(1,length(1:NT)));
title('Distance between vehicles i and N+1');
xlabel('time [s]');
ylabel('distance [m]');


% figure;
% subplot(3, 1, 1);
% plot(1:NT, dist12);
% hold on;
% plot(1:NT,d_min*ones(1,length(1:NT)));
% title('Distance between vehicles 1 2');
% xlabel('time [s]');
% ylabel('distance [m]');
% 
% %%%%%%%%%%%%
% subplot(3, 1, 2);
% plot(1:NT, dist13);
% hold on;
% plot(1:NT,d_min*ones(1,length(1:NT)));
% title('Distance between vehicles 1 3');
% xlabel('time [s]');
% ylabel('distance [m]');
% 
% %%%%%%%%%
% subplot(3, 1, 3);
% plot(1:NT, dist23);
% hold on;
% plot(1:NT,d_min*ones(1,length(1:NT)));
% title('Distance between vehicles 2 3');
% xlabel('time [s]');
% ylabel('distance [m]');
% 
