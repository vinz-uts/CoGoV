clear;
close all;
% Simulation of a potential collision with obstacles in a river 
set(0, 'DefaultLineLineWidth', 3);
%%% Loading vehicle parameters and control variables
addpath('../../../../marine_vehicle');      addpath('../../../../marine_vehicle/omni_vehicle_2DOF');
addpath(genpath('../../../../util'));       addpath('../../../../CG');
addpath(genpath('../../../../tbxmanager'));

vehicle_2DOF_model_2

%% Vehicles
N = 4; % number of vehicles
N1 = [1, 2];
N2 = [2, 3];

% Vehicle 1
vehicle{1} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{1}.init_position(-0.5,0);
% Vehicle 2
vehicle{2} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{2}.init_position(0.5,0);
% Vehicle 3
vehicle{3} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{3}.init_position(-0.5, 20);
% Vehicle 4
vehicle{4} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{4}.init_position(0.5, 20);


%% Net configuration
%   1
%  / \
% 2   3
adj_matrix = [-1  1  1  1;
			   1 -1  1  1;
               1  1  -1  1;
               1  1  1  -1];


%% Vehicles constraints
% Vehicles swarm position constraints
% ||(x,y)_i-(x,y)_j|| > d_min
d_min = 0.5; % minimum distance between vehicles - [m]
d_max = 1;

% Vehicles input/speed constraints
Vx = 0.5; % max abs of speed along x - [m/s]
Vy = 0.5; % max abs of speed along y - [m/s]
T_max = 20; % max abs of motor thrust - [N]
x_max = 3; % max value of x position
y_max = 50; % max value of y position 


%% Command Governor parameters
Psi = 0.01*eye(2); % vehicle's references weight matrix
k0 = 25; % prediction horizon

%% Dynamic Command Governor

for i=1:N
    vehicle{i}.cg = DynamicDistribuitedCommandGovernor(i,Phi,G,Hc,L,Psi,k0, 'gurobi');
    vehicle{i}.cg.add_vehicle_cnstr('position',[x_max,y_max],'thrust',T_max,'speed',[Vx,Vy]);
    for j=1:N
        if adj_matrix(i,j) == 1 % i,j is neighbour       
            if((ismember(i, [1, 2]) && ismember(j, [1, 2])) || ((ismember(i, [3, 4]) && ismember(j, [3, 4]))))
                vehicle{i}.cg.add_swarm_cnstr(j,'anticollision', d_min, 'proximity', d_max);
            else
                vehicle{i}.cg.add_swarm_cnstr(j,'anticollision', d_min, 'proximity', 100);
            end
        end
    end
end

% vehicle{1}.cg.add_swarm_cnstr(2, 'proximity', d_max);
% vehicle{2}.cg.add_swarm_cnstr(1, 'proximity', d_max);
% 
% vehicle{3}.cg.add_swarm_cnstr(4, 'proximity', d_max);
% vehicle{4}.cg.add_swarm_cnstr(3, 'proximity', d_max);
%% Color the net
colors = [0, 1, 2, 3];
vehicle{1}.color = colors(1);
vehicle{2}.color = colors(2);
vehicle{3}.color = colors(3);
vehicle{4}.color = colors(4);

%% Obstacle Initialization
%%% Rectangular orizontal obstacle 
v1a = [1,2]';
v2a = [1,1]';
v3a = [4,1]';
v4a = [4,2]';

%%% Squared obstacle 
v1b = [1,2]';
v2b = [1,1]';
v3b = [2,1]';
v4b = [2,2]';

%%% Rectangular vertical obstacle 
v1c = [1,4]';
v2c = [1,1]';
v3c = [2,1]';
v4c = [2,4]';

%%% Creation of obstacles 

ob1 = Obstacle(v1a,v2a,v3a,v4a);
ob2 = Obstacle(v1b,v2b,v3b,v4b);
ob3 = Obstacle(v1b,v2b,v3b,v4b);
ob4 = Obstacle(v1c,v2c,v3c,v4c);
ob5 = Obstacle(v1b,v2b,v3b,v4b);


%%% Transformation of obstacles (translation) 
ob1.move_obstacle(-1, 16);
ob2.move_obstacle(-1.5, 8);
ob3.move_obstacle(-0.5, 2);
ob4.move_obstacle(-4, 10);
ob5.move_obstacle(-1.5, 12);


% List of all obstacles 
oblist = [ob2, ob3];

%% Simulation 

Tf = 50; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % Recalculation references time

%%% Vehicles References

r{1} = [-0.5,20]'; % position references
r{2} = [0.5,20]'; % position references
r{3} = [-0.5, 0]'; % position references
r{4} = [0.5, 0]'; % position references

NT = ceil(Tf/Tc_cg); % simulation steps number
round = 1;

% Vector used to track distance between the two vehicles
dist = [];

%%% Vision of vehicle in meters 
r_vision = 2.5;

hype = {[], [], [], []};

state_evolution = {vehicle{1}.ctrl_sys.sys.xi, vehicle{2}.ctrl_sys.sys.xi, vehicle{3}.ctrl_sys.sys.xi, vehicle{4}.ctrl_sys.sys.xi};
distance_evolution = {norm(vehicle{1}.ctrl_sys.sys.xi(1:2) - vehicle{2}.ctrl_sys.sys.xi(1:2), inf),...
                      norm(vehicle{1}.ctrl_sys.sys.xi(1:2) - vehicle{3}.ctrl_sys.sys.xi(1:2), inf),...
                      norm(vehicle{1}.ctrl_sys.sys.xi(1:2) - vehicle{4}.ctrl_sys.sys.xi(1:2), inf),...
                      norm(vehicle{2}.ctrl_sys.sys.xi(1:2) - vehicle{3}.ctrl_sys.sys.xi(1:2), inf),...
                      norm(vehicle{2}.ctrl_sys.sys.xi(1:2) - vehicle{4}.ctrl_sys.sys.xi(1:2), inf),...
                      norm(vehicle{3}.ctrl_sys.sys.xi(1:2) - vehicle{4}.ctrl_sys.sys.xi(1:2), inf)};
time = [0];
writerObj = VideoWriter('canal_multi.avi');
open(writerObj);
for t=1:NT
    for i=1:N
        if vehicle{i}.color == colors(round)
            x = vehicle{i}.ctrl_sys.sys.xi; % vehicle current state
            pos = x;
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
            
            %%% Vision Module of the vehicle that identifies the obstacle
            obseen = seen_obstacles(pos(1:2), r_vision, oblist);
            
            
            if(isempty(obseen))
                %%% If no obstacle has been found, then normal CG is been called
                [g,s] = vehicle{i}.cg.compute_cmd(xa,r{i},g_n);
            else
                %%% If an obstacle has been found, it is the closest, and the CG
                %%% takes into consideration the obstacle
                ver = obseen.vertices;
                % Calculation of the closest point
                tmpx = ver(1,:)-pos(1);
                tmpy = ver(2,:)-pos(2);
                tmp = [tmpx; tmpy];
                norm_vect = vecnorm(tmp);
                [m, kk] = min(norm_vect);
%                 if(norm(vehicle{2}.ctrl_sys.sys.x(1:2,end) - ver(:, kk)) <= norm(vehicle{1}.ctrl_sys.sys.x(1:2,end) - ver(:, kk)))
%                     hype{2} = hype{1};
%                     hype{1} = [];
%                 else
%                     hype{1} = hype{2};
%                     hype{2} = [];
%                 end

                if(ismember(i, [1, 2]))
                    if(i == 1)
                        [g, s, hype{1}] = vehicle{i}.cg.compute_cmd(xa,r{i},g_n,ver, []);
                    else
                        [g, s, hype{2}] = vehicle{i}.cg.compute_cmd(xa,r{i},g_n,ver, hype{1});
                    end
                else
                    if(i == 3)
                        [g, s, hype{3}] = vehicle{i}.cg.compute_cmd(xa,r{i},g_n,ver, []);
                    else
                        [g, s, hype{4}] = vehicle{i}.cg.compute_cmd(xa,r{i},g_n,ver, hype{3});
                    end
                end
            end
            
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
    state_evolution = {[state_evolution{1}, vehicle{1}.ctrl_sys.sys.xi],...
                       [state_evolution{2}, vehicle{2}.ctrl_sys.sys.xi],...
                       [state_evolution{3}, vehicle{3}.ctrl_sys.sys.xi],...
                       [state_evolution{4}, vehicle{4}.ctrl_sys.sys.xi]};
                   
    distance_evolution{1} = [distance_evolution{1}, norm(vehicle{1}.ctrl_sys.sys.xi(1:2) - vehicle{2}.ctrl_sys.sys.xi(1:2), inf)];
    distance_evolution{2} = [distance_evolution{2}, norm(vehicle{1}.ctrl_sys.sys.xi(1:2) - vehicle{3}.ctrl_sys.sys.xi(1:2), inf)];
    distance_evolution{3} = [distance_evolution{3}, norm(vehicle{1}.ctrl_sys.sys.xi(1:2) - vehicle{4}.ctrl_sys.sys.xi(1:2), inf)];
    distance_evolution{4} = [distance_evolution{4}, norm(vehicle{2}.ctrl_sys.sys.xi(1:2) - vehicle{3}.ctrl_sys.sys.xi(1:2), inf)];
    distance_evolution{5} = [distance_evolution{5}, norm(vehicle{2}.ctrl_sys.sys.xi(1:2) - vehicle{4}.ctrl_sys.sys.xi(1:2), inf)];
    distance_evolution{6} = [distance_evolution{6}, norm(vehicle{3}.ctrl_sys.sys.xi(1:2) - vehicle{4}.ctrl_sys.sys.xi(1:2), inf)];
    time = [time, time(end)+Tc];
    round = rem(round,length(colors))+1;
    
     %%% Plotting section
%     figure(1)
    clf;
    plot_struct = [];
    color_plot = ['k', 'k', 'b', 'b'];
    title('Canal Simulation');
    xlabel('x[m]');
    ylabel('y[m]');
    grid on;
    canal = polyshape([-3 -3 3 3], [20, 0, 0, 20]);
    hold on
    plot(canal, 'FaceColor', 'cyan')
    for i=1:N
        plot_struct = [plot_struct, plot_2Dof_vehicle(vehicle{i}, r, r_vision, 'RangeAxis', [-10 10 0 20], 'LineWidth', 3, 'D_min_style', '--', 'Color', color_plot(i))];
        plot(hype{i}, color_plot(i));
    end
    
    % Plotting Command Governor Reference
    plot_struct = [plot_struct, plot(g(1),g(2),'xk')];
    % Plotting of the obstacles (black)
    for ii = 1:length(oblist)
        plot(oblist(ii), '.-b');
    end
    
%     plot(ones(1,21)*-(x_max),0:20,'b');
%     plot(ones(1,21)*x_max,0:20,'b');
%    Plotting of the closest obstacle found (red)
    if(not(isempty(obseen)))
        plot(obseen,'-r');
    end
    
    legend(plot_struct([1,3]), ["group 1", "group 2"]);
    drawnow;
    
writeVideo(writerObj,im2frame(print('-RGBImage', strcat('-r', '300'))));
%     writeVideo(writerObj, getframe(gcf));
    hold off;
    
%     titles_1 = ["distance between vehicles 1 and 2",...
%             "distance between vehicles 1 and 3",...
%             "distance between vehicles 1 and 4"];
%     titles_2 = ["distance between vehicles 2 and 3",...
%             "distance between vehicles 2 and 4",...
%             "distance between vehicles 3 and 4"];
%     figure(2)
%     clf;
%     for jj=1:3
%         subplot(3, 1, jj)
%         grid on;
%         hold on;
%         plot(time, distance_evolution{jj})
%         plot(time, d_min*ones(length(time)), 'r--')
%         if(jj == 1)
%             plot(time, d_max*ones(length(time)), 'r--')
%         end
%         title(titles_1(jj));
%         legend(["distance", "minimum distance", "maximum distance"])
%     end
%     drawnow;
%     
%     figure(3)
%     clf;
%     for jj=1:3
%         subplot(3, 1, jj)
%         grid on;
%         hold on;
%         plot(time, distance_evolution{jj+3})
%         plot(time, d_min*ones(length(time)), 'r--')
%         if(jj == 3)
%             plot(time, d_max*ones(length(time)), 'r--')
%         end
%         legend(["distance", "minimum distance", "maximum distance"])
%          title(titles_2(jj));
%     end
%     drawnow;
    %%%%%%%%%%%%%
     
    dist = [dist, norm((vehicle{1}.ctrl_sys.sys.x(1:2,end)-vehicle{2}.ctrl_sys.sys.x(1:2,end)))];
end
close(writerObj);

figure;
plot(1:NT, dist);
hold on;
plot(1:NT,d_min*ones(1,length(1:NT)));
title('Distance between vehicles');
xlabel('time [s]');
ylabel('distance [m]');



