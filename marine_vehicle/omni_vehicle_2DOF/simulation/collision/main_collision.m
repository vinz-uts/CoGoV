%% Clear workspace
clear;
close all;

set(0, 'DefaultLineLineWidth', 3);
%% Load vehicles' model matrices 
%%% Loading vehicle parameters and control variables
addpath('../../../../marine_vehicle');      addpath('../../../../marine_vehicle/omni_vehicle_2DOF');
addpath(genpath('../../../../util'));       addpath('../../../../CG');
addpath(genpath('../../../../tbxmanager'));

linecirc(5, 5, 0, 0, 5);
vehicle_2DOF_model_2

%% Vehicles
N = 2; % number of vehicles

% Vehicle 1
vehicle{1} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{1}.init_position(-5.45,-6.45);
% Vehicle 2
vehicle{2} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{2}.init_position(-6.45,-5.45);

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
T_max = 0.485; % max abs of motor thrust - [N]

%% Command Governor parameters
Psi = 0.01*eye(2); % vehicle's references weight matrix
k0 = 25; % prediction horizon

%% Dynamic Command Governor
for i=1:N
    vehicle{i}.cg = DynamicDistribuitedCommandGovernor(i,Phi,G,Hc,L,Psi,k0, 'gurobi');
    vehicle{i}.cg.add_vehicle_cnstr('thrust', T_max);
    for j=1:N
        if adj_matrix(i,j) == 1 % i,j is neighbour
            vehicle{i}.cg.add_swarm_cnstr(j, 'anticollision', d_min);
        end
    end
end


%% Color the net
colors = [0,1];
vehicle{1}.color = colors(1);
vehicle{2}.color = colors(2);

plot_color = ['b', 'g'];

%% Simulation Colored Round CG
Tf = 15; % simulation time
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
%r{1} = [-10, 10]';
r{2} = vehicle{1}.ctrl_sys.sys.xi(1:2);

vehicle{1}.planner = LinePlanner(r{1}, 'radius', 0.6);
vehicle{2}.planner = LinePlanner(r{2}, 'radius', 0.6);

%%%%%% Sim cycle %%%%%%%%%
u1 = [];
u2 = [];
g_list = [];
writerObj = VideoWriter('oil_simulation.avi');
open(writerObj);
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
            
            if(i == 1)
                g_list = [g_list, vehicle{i}.g];
            end
        end
    end
    
    
    for i=1:N
        switch i
            case 1
                u1 = [u1, vehicle{i}.ctrl_sys.sim(vehicle{i}.g,Tc_cg)];
            case 2
                u2 = [u2, vehicle{i}.ctrl_sys.sim(vehicle{i}.g,Tc_cg)];
        end
    end
    round = rem(round,length(colors))+1;
    
     
    % Live plot
    figure(1);  
    clf;
    grid on;
    title('Frontal Collision Simulation');
    xlabel('x [m]');
    ylabel('y [m]');
    hold on;
    
    plot_struct = [];
    legend_list = {'vehicle 1 trajectory', 'vehicle 1 g', 'vehicle 2 trajectory', 'vehicle 2 g'};
    %%%%%%% live plot %%%%%%%
    for k=1:N

    	% Plot vehicles trajectory
        plot_struct = [plot_struct, plot(vehicle{k}.ctrl_sys.sys.x(1,:),vehicle{k}.ctrl_sys.sys.x(2,:), strcat(plot_color(k), '-.'))];
        
        % Plot vehicles position 
        plot(vehicle{k}.ctrl_sys.sys.x(1,end),vehicle{k}.ctrl_sys.sys.x(2,end), strcat(plot_color(k), 'o'),'MarkerFaceColor',plot_color(k),'MarkerSize',7);
        
        % Plot vehicles CG references
        plot_struct = [plot_struct, plot(vehicle{k}.g(1), vehicle{k}.g(2), strcat(plot_color(k), 'x'))];
        
    end
    
    legend(plot_struct, legend_list)
    drawnow;
    writeVideo(writerObj, getframe(gcf));
    %%%%%%%%%%%%%
    
    
    dist = [dist, norm((vehicle{1}.ctrl_sys.sys.x(1:2,end)-vehicle{2}.ctrl_sys.sys.x(1:2,end)))];
end
close(writerObj);

vehicle_data = [];
nx = 4;
u = [u1; u2];
for i=1:N
    data = struct("x", [vehicle{i}.ctrl_sys.sys.x; vehicle{i}.ctrl_sys.xc], "u", u((i-1)*2+1:(i-1)*2+2, :)); %...
                  %"u", -vehicle{i}.ctrl_sys.Fa(:,1:nx)*vehicle{i}.ctrl_sys.sys.x + vehicle{i}.ctrl_sys.Fa(:,nx+1:end)*vehicle{i}.ctrl_sys.xc);
                  
    vehicle_data = [vehicle_data, data];
end
sim_data = struct('system', vehicle_data, 'time', (1:NT)*Tc, "Tc", Tc, 'd_min', d_min, 'Max_x', [], "Max_y", [], "T_max", T_max, 'g', g_list);

save("main_collision.mat", "sim_data");

figure;
plot(1:NT, dist);
hold on;
plot(1:NT,d_min*ones(1,length(1:NT)));
title('Distance between vehicles');
xlabel('time [s]');
ylabel('distance [m]');

