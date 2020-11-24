%% Clear workspace
clear;  close all;

%% Load vehicles' model matrices
addpath('../../../../marine_vehicle');      addpath('../../../../marine_vehicle/omni_vehicle_2DOF');
addpath(genpath('../../../../util'));       addpath('../../../../CG');
addpath(genpath('../../../../tbxmanager'));

vehicle_2DOF_model_2

%% Init vehicles
N = 5; % number of vehicles
vehicle = cell(1, N);
for i=1:N
    vehicle{i} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
    vehicle{i}.init_position(10*sin(2*pi/N*i) + 20 ,10*cos(2*pi/N*i)+20);
end


%% Net configuration

adj_matrix = [-1  1  0  0  1;
    1 -1  1  0  0;
    0  1 -1  1  0;
    0  0  1 -1  1;
    1  0  0  1 -1];

spanning_tree =[-1  1  0  0  0;
    1 -1  1  0  0;
    0  1 -1  1  0;
    0  0  1 -1  1;
    0  0  0  1 -1];

vehicle{1}.parent = 0;

for i=2:N
    vehicle{i}.parent = i-1;
end


%% Vehicles constraints
% Vehicles swarm position constraints
% ||(x,y)_i-(x,y)_j||∞ ≤ d_max
% ||(x,y)_i-(x,y)_j||∞ ≥ d_min
d_max = 13;  % maximum distance between vehicles - [m]
d_min = 3; % minimum distance between vehicles - [m]

% Vehicles input/speed constraints
Vx = 0.5; % max abs of speed along x - [m/s]
Vy = 0.5; % max abs of speed along y - [m/s]
T_max = 20; % max abs of motor thrust - [N]

%% Command Governor parameters
Psi = 1000*eye(2); % vehicle's references weight matrix
k0 = 10; % prediction horizon

%%% Vector for distance for plotting purposes
dist = [];
dist2 = [];
dist3 = [];
dist4 = [];


%% Dynamic Command Governor
for i=1:N
    vehicle{i}.cg = DynamicDistribuitedCommandGovernor(i,Phi,G,Hc,L,Psi,k0,'gurobi');
    vehicle{i}.cg.add_vehicle_cnstr('speed',[Vx,Vy],'thrust',T_max);
    for j=1:N
        if(not (i==j))
            if(spanning_tree(i,j) == 1) % i,j is neighbour
                vehicle{i}.cg.add_swarm_cnstr(j,'anticollision',d_min,'proximity',d_max);
            else
                vehicle{i}.cg.add_swarm_cnstr(j,'anticollision',d_min);
            end
        end
    end
end

%% Color the net
colors = 1:N;

for i=1:N
    vehicle{i}.color = colors(i);
end



%% Simulation Colored Round CG
Tf = 20; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time
NT = ceil(Tf/Tc_cg); % simulation steps number

plot_color = ['b', 'g', 'k', 'r', 'm'];



% References

r{1} = vehicle{1}.ctrl_sys.sys.xi(1:2);
r{2} = vehicle{2}.ctrl_sys.sys.xi(1:2);
r{3} = vehicle{3}.ctrl_sys.sys.xi(1:2);
r{4} = vehicle{1}.ctrl_sys.sys.xi(1:2);
r{5} = vehicle{1}.ctrl_sys.sys.xi(1:2);

for i=1:N
    vehicle{i}.planner = LinePlanner(r{i}, 'radius', 1.2);
end

round = 1;
for t=1:NT
    for i=1:N
        
        if vehicle{i}.color == colors(round)
            index_min = -1;
            d_ijmin = 100;
            for j=1:N
                if i~=j && not(vehicle{i}.parent==j) && not(vehicle{i}.parent==0)
                    d_ij = norm(vehicle{i}.ctrl_sys.sys.xi(1:2)-vehicle{j}.ctrl_sys.sys.xi(1:2));
                    if(d_ij < d_ijmin)
                        index_min = j;
                        d_ijmin = d_ij;
                    end
                end
            end
            if(not(vehicle{i}.parent==0))
                if(d_ijmin < norm(vehicle{i}.ctrl_sys.sys.xi(1:2)-vehicle{vehicle{i}.parent}.ctrl_sys.sys.xi(1:2)))
                    spanning_tree_tmp = spanning_tree;
                    spanning_tree_tmp(i,vehicle{i}.parent) = 0;
                    spanning_tree_tmp(vehicle{i}.parent,i) = 0;
                    spanning_tree_tmp(i,index_min) = 1;
                    spanning_tree_tmp(index_min,i) = 1;
                    if(isConnected(spanning_tree_tmp))
                        
                        % Remove the old parent proximity constraints
                        vehicle{vehicle{i}.parent}.cg.remove_swarm_cnstr(i);
                        vehicle{i}.cg.remove_swarm_cnstr(vehicle{i}.parent);
                        vehicle{vehicle{i}.parent}.cg.add_swarm_cnstr(i,'anticollision',d_min);
                        vehicle{i}.cg.add_swarm_cnstr(vehicle{i}.parent,'anticollision',d_min);
                        
                        % Change parent
                        vehicle{i}.parent = index_min;
                        
                        fprintf('***************Parent changed (%d ,%d) **************** \n ',i,index_min);
                        
                        % Add proximity constraints with new parent
                        vehicle{vehicle{i}.parent}.cg.remove_swarm_cnstr(i);
                        vehicle{i}.cg.remove_swarm_cnstr(vehicle{i}.parent);
                        
                        vehicle{vehicle{i}.parent}.cg.add_swarm_cnstr(i,'proximity',d_max,'anticollision',d_min);
                        vehicle{i}.cg.add_swarm_cnstr(vehicle{i}.parent,'proximity',d_max,'anticollision',d_min);
                        
                        spanning_tree = spanning_tree_tmp;
                    else
                        fprintf('************Change parent request denied (%d ,%d) **************** \n ',i,index_min);
                    end
                end
            end
            
            x = vehicle{i}.ctrl_sys.sys.xi; % vehicle current state
            xc = vehicle{i}.ctrl_sys.xci; % controller current state
            xa = [x;xc];
            g_n = [];
            for k = vehicle{i}.cg.neigh
                g_n = [g_n;vehicle{k}.g];
                x = vehicle{k}.ctrl_sys.sys.xi; % vehicle{j} current state
                xc = vehicle{k}.ctrl_sys.xci; % controller{j} current state
                xa = [xa;x;xc];
            end
            
            r_tmp = vehicle{i}.planner.compute_reference(vehicle{i},xa); 
            
            [g,s] = vehicle{i}.cg.compute_cmd(xa,r_tmp,g_n);
            
            if ~isempty(g)
                vehicle{i}.g = g;
            else
                disp('WARN: old references');
                t,i
            end
        end
    end
    % Simulate for Tc_cg
    for i=1:N
        vehicle{i}.ctrl_sys.sim(vehicle{i}.g,Tc_cg);
    end
    round = rem(round,length(colors))+1;
    
    figure(1);
    
    axis equal
    for k=1:N
        % Trajectory
        %         axis([0 5 -4 4])
        plot(vehicle{k}.ctrl_sys.sys.x(1,:),vehicle{k}.ctrl_sys.sys.x(2,:), strcat(plot_color(k), '-.'),'LineWidth',0.8);
        hold on;
        plot(vehicle{k}.ctrl_sys.sys.x(1,end),vehicle{k}.ctrl_sys.sys.x(2,end), strcat(plot_color(k), 'o'),'MarkerFaceColor',plot_color(k),'MarkerSize',7);
        %%%% live plot %%%%
        plot(r{k}(1), r{k}(2), strcat(plot_color(k), 'o'));
        plot(vehicle{k}.g(1), vehicle{k}.g(2), strcat(plot_color(k), 'x'));
        %%%%%%%%%%%%
        if(not(vehicle{k}.parent==0))
            v = vehicle{k}.ctrl_sys.sys.xi(1:2);
            p =  vehicle{vehicle{k}.parent}.ctrl_sys.sys.xi(1:2);
            
            if((v(1) - p(1)) == 0)
                slope = inf;
                intercept = p(1);
            else
                slope = (v(2) - p(2))/(v(1) - p(1));
                intercept = -slope*p(1) + p(2);
            end
            
            if(slope == inf)
                line = @(x) (intercept);
            else
                line = @(x) (x*slope + intercept);
            end
            

            if(v(1) < p(1))
                x_ax = v(1):0.01:p(1);
            else
                x_ax = p(1):0.01:v(1) ;
            end
            
            if(slope == inf)
                plot(ones(20,1)*p(1), ones(20,1)*line(p(2)), strcat(plot_color(k), ':'),'LineWidth',1);
            else
                plot(x_ax, line(x_ax), strcat(plot_color(k), ':'),'LineWidth',1);
            end
        end
        
    end
    
    hold off;
    dist = [dist, norm((vehicle{1}.ctrl_sys.sys.x(1:2,end)-vehicle{2}.ctrl_sys.sys.x(1:2,end)),inf)];
    dist2 = [dist2, norm((vehicle{2}.ctrl_sys.sys.x(1:2,end)-vehicle{3}.ctrl_sys.sys.x(1:2,end)),inf)];
    dist3 = [dist3, norm((vehicle{3}.ctrl_sys.sys.x(1:2,end)-vehicle{4}.ctrl_sys.sys.x(1:2,end)),inf)];
    dist4 = [dist3, norm((vehicle{4}.ctrl_sys.sys.x(1:2,end)-vehicle{5}.ctrl_sys.sys.x(1:2,end)),inf)];
    drawnow;
end


figure;
subplot(4, 1, 1);
plot(1:NT, dist);
hold on;
plot(1:NT,d_max*ones(1,length(1:NT)));
title('Distance between vehicles 1 and vehicle 2');
xlabel('time [s]');
ylabel('distance [m]');

%%%%%%%%%%%%
subplot(4, 1, 2);
plot(1:NT, dist2);
hold on;
plot(1:NT,d_max*ones(1,length(1:NT)));
title('Distance between vehicles 2 and vehicle 3');
xlabel('time [s]');
ylabel('distance [m]');

%%%%%%%%%
subplot(4, 1, 3);
plot(1:NT, dist3);
hold on;
plot(1:NT,d_max*ones(1,length(1:NT)));
title('Distance between vehicles 3 and vehicle 4');
xlabel('time [s]');
ylabel('distance [m]');
%

%%%%%%%%%
subplot(4, 1, 4);
plot(1:NT, dist3);
hold on;
plot(1:NT,d_max*ones(1,length(1:NT)));
title('Distance between vehicles 4 and vehicle 5');
xlabel('time [s]');
ylabel('distance [m]');
%