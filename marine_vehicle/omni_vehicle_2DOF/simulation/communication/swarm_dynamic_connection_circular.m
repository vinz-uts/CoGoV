% Simulation implemented to show only the Change Parent strategy 
%% Clear workspace
clear;  close all;

%% Simulation Settings
realtime_plot = true;
show_packets = false;

record_video = true;
video_name = 'communication';
resolution = '300'; % ppi pixel per inch (300, 120)

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


%% Planners Initialization 
xSamples = [1, 0, -1, 0]; % x Samples for circular_trajectory
ySamples = [0, 1, 0, -1]; % y Samples for circular_trajectory

% Vehicles 2,3 and 5 need to follow a circular trajectory 
vehicle{2}.planner = Polar_trajectory_planner(xSamples, ySamples, 'step', 0.4 , 'tolerance', 0.7);
vehicle{2}.planner.transform(4.5, vehicle{2}.ctrl_sys.sys.xi(1:2));
vehicle{3}.planner = Polar_trajectory_planner(xSamples, ySamples, 'clockwise', false , 'step', 0.4 , 'tolerance', 0.7);
vehicle{3}.planner.transform(3, vehicle{3}.ctrl_sys.sys.xi(1:2));
vehicle{5}.planner = Polar_trajectory_planner(xSamples, ySamples, 'step', 0.4, 'clockwise', false, 'tolerance', 0.1);
vehicle{5}.planner.transform(6, vehicle{5}.ctrl_sys.sys.xi(1:2));

% Vehicles 1 and 4 need to stand still 
vehicle{1}.planner = LinePlanner(vehicle{1}.ctrl_sys.sys.xi(1:2));
vehicle{4}.planner = LinePlanner(vehicle{4}.ctrl_sys.sys.xi(1:2));

%% Net configuration
spanning_tree =[-1  1  0  0  0;
                 1 -1  1  0  0;
                 0  1 -1  1  0;
                 0  0  1 -1  1;
                 0  0  0  1 -1];

% It is assumed that vehicle 1 is the fixed root of the spanning tree
vehicle{1}.parent = 0;
% Parent initialization 
for i=2:N
    vehicle{i}.parent = i-1;
end


%% Vehicles constraints
% Vehicles swarm position constraints
% ||(x,y)_i-(x,y)_j|| < d_max
% ||(x,y)_i-(x,y)_j|| > d_min
d_max = 15;  % maximum distance between vehicles - [m]
d_min = 1; % minimum distance between vehicles - [m]

% Vehicles input/speed constraints
Vx = 0.7; % max abs of speed along x - [m/s]
Vy = 0.7; % max abs of speed along y - [m/s]
T_max = 30; % max abs of motor thrust - [N]

%% Command Governor parameters
Psi = 1000*eye(2); % vehicle's references weight matrix
k0 = 20; % prediction horizon

%% Dynamic Command Governor
for i=1:N
    vehicle{i}.cg = DynamicDistribuitedCommandGovernor(i,Phi,G,Hc,L,Psi,k0,'gurobi');
    vehicle{i}.cg.add_vehicle_cnstr('speed', [Vx,Vy], 'thrust', T_max);
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

%% Colors
colors = 1:N;
for i=1:N
    vehicle{i}.color = colors(i);
end
plot_color = ['b', 'g', 'c', 'r', 'm'];

%% Simulation Colored Round CG
Tf = 340000; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time
NT = ceil(Tf/Tc_cg); % simulation steps number

%% Communication Layer Initilization (packets counting)
pl = Phisical_Layer(N, 200, 0, 1);
pl.update(vehicle, 0);
if(record_video)
    writerObj = VideoWriter(strcat(video_name, '.avi'));
    writerObj.FrameRate = 20;
    open(writerObj);
end

%% Data struct
data = struct('cpu_time', [], 'yalmip_time', []);

%% Simulation cycle
round = 1;
for t=1:NT
    for i=1:N
        
        if vehicle{i}.color == colors(round)
            index_min = -1;
            d_ijmin = 100;
            
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
            
%             if(not(vehicle{i}.parent == 0))
%                 % Check if the distance between the actual parent is larger
%                 % than the distance with the closest vehicle 
%                 
%                 if(d_ijmin < norm(vehicle{i}.ctrl_sys.sys.xi(1:2)-vehicle{vehicle{i}.parent}.ctrl_sys.sys.xi(1:2)))
%                     spanning_tree_tmp = spanning_tree;
%                     spanning_tree_tmp(i,vehicle{i}.parent) = 0;
%                     spanning_tree_tmp(vehicle{i}.parent,i) = 0;
%                     spanning_tree_tmp(i,index_min) = 1;
%                     spanning_tree_tmp(index_min,i) = 1;
%                     if(isConnected(spanning_tree_tmp))
%                         
%                         % Remove the old parent proximity constraints
%                         pl.send_packet(i, index_min, [1,2]);
%                         vehicle{vehicle{i}.parent}.cg.remove_swarm_cnstr(i);
%                         vehicle{i}.cg.remove_swarm_cnstr(vehicle{i}.parent);
%                         vehicle{vehicle{i}.parent}.cg.add_swarm_cnstr(i,'anticollision',d_min);
%                         vehicle{i}.cg.add_swarm_cnstr(vehicle{i}.parent,'anticollision',d_min);
%                         
%                         % Change parent
%                         vehicle{i}.parent = index_min;
%                         
%                         fprintf('***************Parent changed (%d ,%d) **************** \n ',i,index_min);
%                         
%                         % Add proximity constraints with new parent
%                         if(pl.look_for_packets(vehicle{i}.parent))
%                             pl.get_packet(vehicle{i}.parent);
%                             vehicle{vehicle{i}.parent}.cg.remove_swarm_cnstr(i);
%                             vehicle{i}.cg.remove_swarm_cnstr(vehicle{i}.parent);
%                             
%                             vehicle{vehicle{i}.parent}.cg.add_swarm_cnstr(i,'proximity',d_max,'anticollision',d_min);
%                             vehicle{i}.cg.add_swarm_cnstr(vehicle{i}.parent,'proximity',d_max,'anticollision',d_min);
%                             
%                             spanning_tree = spanning_tree_tmp;
%                         end
%                     else
%                         fprintf('************Change parent request denied (%d ,%d) **************** \n ',i,index_min);
%                     end
%                 end
%             end
            
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
            
            r = vehicle{i}.planner.compute_reference(vehicle{i},xa); 
            
            [g, diagnostic] = vehicle{i}.cg.compute_cmd(xa, r, g_n);
%             g = r;
            if ~isempty(g)
                vehicle{i}.g = g;
                data.cpu_time = [data.cpu_time, diagnostic.solvertime];
                data.yalmip_time = [data.cpu_time, diagnostic.yalmiptime];
            else
                disp('WARN: old references');
                t,i
            end
            % Position Update through communication
             for j=1:N
                if i~=j
                   pl.send_packet(i, j, '$C 00.00,00.00,00.00,00.00,00.00,00.00,00.00,00.00,00');
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
    
    %%%% live plot %%%%
    if(realtime_plot)
        plot_struct = []; % Legend data
        legend_list = []; % Legend data
        
        figure(1);
        clf;
        hold on;

        plot(vehicle{2}.planner);
        plot(vehicle{3}.planner);
        plot(vehicle{5}.planner);
        %%%%%%%%%%%% Plotting of the parent connection 
        for k=1:N
            if(not(vehicle{k}.parent==0))
                v = vehicle{k}.ctrl_sys.sys.xi(1:2);
                p =  vehicle{vehicle{k}.parent}.ctrl_sys.sys.xi(1:2);
                plot([v(1), p(1)], [v(2), p(2)], strcat(plot_color(k), ':'), 'LineWidth', 2);
            end
        end
        for k=1:N
            % Trajectory
            plot(vehicle{k}.ctrl_sys.sys.x(1,:),vehicle{k}.ctrl_sys.sys.x(2,:), strcat(plot_color(k), '-.'), 'LineWidth', 3);
            axis([0 40 0 40]);
            plot_struct = [plot_struct, plot(vehicle{k}.ctrl_sys.sys.x(1,end),vehicle{k}.ctrl_sys.sys.x(2,end), strcat(plot_color(k), 'o'), ...
                'MarkerFaceColor', plot_color(k), 'MarkerSize', 8)];
            legend_list = [legend_list, sprintf("vehicle %d", k)];
            %plot(r{k}(1), r{k}(2), strcat(plot_color(k), 'o'));
            plot(vehicle{k}.g(1), vehicle{k}.g(2), strcat(plot_color(k), 'x'), 'LineWidth', 1.5);
            %legend_list = [legend_list, sprintf("vehicle %d g", k)];
        end
        if(show_packets)
            plot(pl);
        end
        hold off;
        title('Connectivity Simulation');
        xlabel('x [m]');
        ylabel('y [m]');
        legend(plot_struct, legend_list, 'Location', 'northwest');
        grid on;
        drawnow;
        if(record_video)
            writeVideo(writerObj, im2frame(print('-RGBImage', strcat('-r', resolution))));
        end
    end
end
if(record_video)
    close(writerObj);
end
