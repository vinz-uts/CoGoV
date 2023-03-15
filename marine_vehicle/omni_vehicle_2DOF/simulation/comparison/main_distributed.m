%% Clear workspace
clear;  close all;

%%%%%%%%%%%%%%%%%%%%% configurazion parameters %%%%%%%%%%%%%%
N = 4; % number of vehicles
init_radius = 7;
random_reference = true;
parent_change_enabled = false;

%%%%%%%%%%%%%%%%%%%% computed parameters %%%%%%%%%%%%%
theta_step = 2*pi/N;

%%%%%%%%%%%%%%%
%% Vehicles constraints
% Vehicles swarm position constraints
% ||(x,y)_i-(x,y)_j|| < d_max
% ||(x,y)_i-(x,y)_j|| > d_min
d_max = 20;  % maximum distance between vehicles - [m]
d_min = 0.1; % minimum distance between vehicles - [m]

% Vehicles input/speed constraints
Vx = 0.7; % max abs of speed along x - [m/s]
Vy = 0.7; % max abs of speed along y - [m/s]
T_max = 30; % max abs of motor thrust - [N]

%% Load vehicles' model matrices
addpath('../../../../marine_vehicle');      addpath('../../../../marine_vehicle/omni_vehicle_2DOF');
addpath(genpath('../../../../util'));       addpath('../../../../CG');
addpath(genpath('../../../../tbxmanager'));

vehicle_2DOF_model_2

%%%% Simulation Settings
%% Init vehicles

vehicle = cell(1, N);
r = cell(1, N);

for i=1:N
    vehicle{i} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
%     vehicle{N-i+1} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
    
    vehicle{i}.init_position(init_radius*cos(theta_step*(i-1)), init_radius*sin(theta_step*(i-1)));
%     vehicle{N-i+1}.init_position(init_radius*cos(theta_step*(i-1) + pi), init_radius*sin(theta_step*(i-1) + pi));
    
    if(not(random_reference))
        r{i} = [init_radius*cos(theta_step*(i-1) + pi); init_radius*sin(theta_step*(i-1) + pi)];
        vehicle{i}.planner = LinePlanner(r{i}, 'radius', 1.2);
    end
end

if(random_reference)
    refs = zeros(2, N);
    for i = 1:N
        refs(:, i) = vehicle{i}.ctrl_sys.sys.xi(1:2);
    end
    
    for i = 1:N
        index = round((rand()*(length(refs(1, :))-1))) + 1;
        r{i} = refs(:, index);
        refs(:, index) = [];
        
        vehicle{i}.planner = LinePlanner(r{i}, 'radius', 1.2);
    end
end
%% Net configuration
spanning_tree = zeros(N, N);

spanning_tree(1, 1) = -1;
spanning_tree(1, 2) = 1;
for i = 2:N-1
    spanning_tree(i, i) = -1;
    spanning_tree(i, i-1) = 1;
    spanning_tree(i, i+1) = 1;
end
spanning_tree(N, N) = -1;
spanning_tree(N, N-1) = 1;

%%%%%%%%%%%%%

%%%%%%%%% parent change algorithm %%%%%%%%%%
% It is assumed that vehicle 1 is the root of the spanning tree, and cant
% change
vehicle{1}.parent = 0;

for i=2:N
    vehicle{i}.parent = i-1;
end

%%%%%%%%%%%%%%%%%

%% Command Governor parameters
Psi = 1000*eye(2); % vehicle's references weight matrix
k0 = 10; % prediction horizon

%% Dynamic Command Governor
for i=1:N
    vehicle{i}.cg = DynamicDistribuitedCommandGovernor(i,Phi,G,Hc,L,Psi,k0,'gurobi');
    vehicle{i}.cg.add_vehicle_cnstr('speed',[Vx,Vy],'thrust',T_max);
    for j=1:N
        if(not (i==j))
            if(spanning_tree(i,j) == 1) % i,j is neighbour
                vehicle{i}.cg.add_swarm_cnstr(j,'anticollision',d_min,'proximity',d_max);
            end
        end
    end
end

%% Color the net
colors = 1:N;

for i=1:N
    vehicle{i}.color = colors(i);
end

% plot_color = ['b', 'g', 'k', 'r', 'm'];

%% Communication constraints
R   = d_max+5; % maximum distance of communications - [m]
R__ = d_max; % maximum distance of connectivity - [m]
R_  = 5; % minimum distance for cooperation - [m]
R_dmin = R_-1;


%% Simulation Colored Round CG
Tf = 10; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time
NT = ceil(Tf/Tc_cg); % simulation steps number

%% Communication Layer Initilization
pl = Phisical_Layer(N, 200, 0, 1);
pl.update(vehicle, 0);

% Vector used to consider eventual virtual references coming from the plug
% in operation 
virtual = zeros(N,1);

% Vectors needed to analyze computational aspects
cputime= [];
yalmiptime=[];

round = 1;
for t=1:NT
    for i=1:N
        
        if vehicle{i}.color == colors(round)
           
            index_min = -1;
            d_ijmin = 100;
            if(parent_change_enabled)
                % Search for the closest vehicle
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
                    % Check if the distance between the actual parent is larger
                    % than the distance with the closest vehicle
                    
                    if(d_ijmin < norm(vehicle{i}.ctrl_sys.sys.xi(1:2)-vehicle{vehicle{i}.parent}.ctrl_sys.sys.xi(1:2)))
                        spanning_tree_tmp = spanning_tree;
                        spanning_tree_tmp(i,vehicle{i}.parent) = 0;
                        spanning_tree_tmp(vehicle{i}.parent,i) = 0;
                        spanning_tree_tmp(i,index_min) = 1;
                        spanning_tree_tmp(index_min,i) = 1;
                        if(isConnected(spanning_tree_tmp))
                            
                            % Remove the old parent proximity constraints
                            pl.send_packet(i, index_min, [1,2]);
                            vehicle{vehicle{i}.parent}.cg.remove_swarm_cnstr(i);
                            vehicle{i}.cg.remove_swarm_cnstr(vehicle{i}.parent);
                            vehicle{vehicle{i}.parent}.cg.add_swarm_cnstr(i,'anticollision',d_min);
                            vehicle{i}.cg.add_swarm_cnstr(vehicle{i}.parent,'anticollision',d_min);
                            
                            % Change parent
                            vehicle{i}.parent = index_min;
                            
                            fprintf('***************Parent changed (%d ,%d) **************** \n ',i,index_min);
                            
                            % Add proximity constraints with new parent
                            if(pl.look_for_packets(vehicle{i}.parent))
                                pl.get_packet(vehicle{i}.parent);
                                vehicle{vehicle{i}.parent}.cg.remove_swarm_cnstr(i);
                                vehicle{i}.cg.remove_swarm_cnstr(vehicle{i}.parent);
                                
                                vehicle{vehicle{i}.parent}.cg.add_swarm_cnstr(i,'proximity',d_max,'anticollision',d_min);
                                vehicle{i}.cg.add_swarm_cnstr(vehicle{i}.parent,'proximity',d_max,'anticollision',d_min);
                                
                                spanning_tree = spanning_tree_tmp;
                            end
                        else
                            fprintf('************Change parent request denied (%d ,%d) **************** \n ',i,index_min);
                        end
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
            
            r{i} = vehicle{i}.planner.compute_reference(vehicle{i},xa);
            
            [g,s] = vehicle{i}.cg.compute_cmd(xa,r{i},g_n);
            
            if ~isempty(g)
                vehicle{i}.g = g;
                cputime= [cputime,s.solvertime];
                yalmiptime=[yalmiptime,s.yalmiptime];
            else
                disp('WARN: old references');
                t,i
            end
            % Position Update through communication
            for j=1:N
                if i~=j
                   pl.send_packet(i, j, []);
                   if(pl.look_for_packets(j))
                       pl.get_packet(j);
                   end
                end
            end
        end
        pl.update(vehicle, t*Tc_cg); 
      
    end
    % Simulate for Tc_cg
    for i=1:N
        vehicle{i}.ctrl_sys.sim(vehicle{i}.g,Tc_cg);
    end
    round = rem(round,length(colors))+1;
       
    figure(1);
    
%     axis([0 40 0 40]);
    for k=1:N
        % Trajectory
        %         axis([0 5 -4 4])
        plot(vehicle{k}.ctrl_sys.sys.x(1,:),vehicle{k}.ctrl_sys.sys.x(2,:), strcat('k', '-.'),'LineWidth',0.8);
        hold on;
        axis([-10 10 -10 10]);
        plot(vehicle{k}.ctrl_sys.sys.x(1,end),vehicle{k}.ctrl_sys.sys.x(2,end), strcat('k', 'o'),'MarkerFaceColor','k','MarkerSize',7);
        %%%% live plot %%%%
        plot(r{k}(1), r{k}(2), strcat('k', 'o'));
        plot(vehicle{k}.g(1), vehicle{k}.g(2), strcat('k', 'x'));
        %%%%%%%%%%%% Plotting of the parent connection 
        if(not(vehicle{k}.parent==0))
            v = vehicle{k}.ctrl_sys.sys.xi(1:2);
            p =  vehicle{vehicle{k}.parent}.ctrl_sys.sys.xi(1:2);
            plot([v(1), p(1)], [v(2), p(2)], strcat('k', ':'),'LineWidth',1);
        end
    end
    plot(pl);
    hold off;
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

