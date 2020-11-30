clear;
close all;

vehicle_2DOF_model_2

%% Vehicles
N = 2; % number of vehicles

% Vehicle 1
vehicle{1} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{1}.init_position(-1.8062,0.8830);
% Vehicle 2
vehicle{2} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{2}.init_position(-1.1062,0.9570);

%% Planners for Circular Path Following

xSamples = 1.2*[1, 0, -1, 0]';
ySamples = 1.2*[0, 1, 0, -1]';

vehicle{1}.planner = Polar_trajectory_planner(xSamples, ySamples,'recovery', 20,'clockwise',false,'rec_from_collision',true);
vehicle{1}.planner.transform(1, [-1, 0]);

Reference_vehicle2 = [-3,1,3,3]; 

%%% For now no recovery policy for vehicle 2, but can be added
vehicle{2}.planner = LinePlanner(Reference_vehicle2);

%% Net configuration
%   1
%  / \
% 2   3
adj_matrix = [-1  1;
    1 -1];

%% Vehicles constraints
% Vehicles swarm position constraints
% ||(x,y)_i-(x,y)_j||? ? d_max
% ||(x,y)_i-(x,y)_j||? ? d_min
d_max = 1.5; % maximum distance between vehicles - [m]
d_min = 0.3; % minimum distance between vehicles - [m]

% Vehicles input/speed constraints
Vx = 0.5; % max abs of speed along x - [m/s]
Vy = 0.5; % max abs of speed along y - [m/s]
T_max = 50; % max abs of motor thrust - [N]

%% Command Governor parameters
Psi = 1000*eye(2); % vehicle's references weight matrix
k0 = 15; % prediction horizon
for i=1:N
    vehicle{i}.cg = DynamicDistribuitedCommandGovernor(i,Phi,G,Hc,L,Psi,k0, 'gurobi');
    vehicle{i}.cg.add_vehicle_cnstr('speed', [Vx, Vy]', 'thrust',T_max);
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
Tf = 200; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time
NT = ceil(Tf/Tc_cg); % simulation steps number
round = 1;
r{1} = [0,0]';
r{2} = r{1};


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
            
            r{i} = vehicle{i}.planner.compute_reference(vehicle{i},xa);
             
            [g,s] = vehicle{i}.cg.compute_cmd(xa, r{i}, g_n);
                        
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
    
     %%%%%%% live plot %%%%%%%
    figure(1);
    
    axis equal
    plot(vehicle{1}.planner)
    hold on;
    
    for k=1:N
        % Trajectory
        %         axis([0 5 -4 4])
        plot(vehicle{k}.ctrl_sys.sys.x(1,:),vehicle{k}.ctrl_sys.sys.x(2,:), strcat(plot_color(k), '-.'),'LineWidth',0.8);
        plot(vehicle{k}.ctrl_sys.sys.x(1,end),vehicle{k}.ctrl_sys.sys.x(2,end), strcat(plot_color(k), 'o'),'MarkerFaceColor',plot_color(k),'MarkerSize',7);
        %%%% live plot %%%%
        plot(r{k}(1), r{k}(2), strcat(plot_color(k), 'o'));
        plot(vehicle{k}.g(1), vehicle{k}.g(2), strcat(plot_color(k), 'x'));
        %%%%%%%%%%%%
    end
    
    hold off;
    
    if(t==1)
        title('Frontal Collision Simulation');
        xlabel('x [m]');
        ylabel('y [m]');
    end
    
    drawnow;
    %%%%%%%%%%%%%
    
    
%     dist = [dist, norm((vehicle{1}.ctrl_sys.sys.x(1:2,end)-vehicle{2}.ctrl_sys.sys.x(1:2,end)))];
end


figure;
plot(1:NT, dist);
hold on;
plot(1:NT,0.3*ones(1,length(1:NT)));
title('Distance between vehicles');
xlabel('time [s]');
ylabel('distance [m]');

