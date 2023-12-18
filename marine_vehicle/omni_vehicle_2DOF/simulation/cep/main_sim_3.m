%% Clear workspace
clear; close all;

%% Load vehicles' model matrices 
vehicle_2DOF_model_2

%% Vehicles
N = 6;
vehicle = cell(1, N);
init_radius = 6;
theta_step = 2*pi/N;
for i=1:N
    vehicle{i} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
    vehicle{i}.init_position(init_radius*cos(theta_step*(i-1)), init_radius*sin(theta_step*(i-1)));
end

%% Net configuration 
% Dynamic configuration (no connection at the beginning)
adj_matrix = zeros(N, N);
for i = 1:N
    adj_matrix(i,i) = -1;
end

%% Vehicles constraints
% Vehicles torque constraints
T_max = 350; % max abs of motor thrust - [N]
% Vehicles position (remain inside the pool)
Max_x = 20; % max position value along x - [m]
Max_y = 20; % max position value along y - [m]
% Vehicles swarm position constraints
% ||p_i - p_j|| > d_min
d_min = 1; % minimum distance between vehicles - [m] 



%% Dynamic Command Governor
Psi = eye(2); % vehicle's references weight matrix
k0 = 20; % prediction horizon
for i=1:N
    vehicle{i}.cg = DynamicDistribuitedCommandGovernor(i,Phi,G,Hc,L,Psi,k0, 'GUROBI');
    vehicle{i}.cg.add_vehicle_cnstr('position', [Max_x, Max_y], 'thrust', T_max);
    for j=1:N
        if adj_matrix(i,j) == 1 % i,j is neighbour
            vehicle{i}.cg.add_swarm_cnstr(j, 'anticollision', d_min);
        end
    end
end

% P&P parameters
R   = 6; % maximum distance of communications - [m]
R__ = 5.5; % maximum distance of connectivity - [m]
R_  = 5; % minimum distance for collision avoidance - [m]

%% Colors (Dynamic)
colors = 1:N;
for i=1:length(colors)
    vehicle{i}.color = colors(i);
end
possible_colors = ['r', 'g', 'b', 'm', 'c', 'y'];
plot_color = ['k', 'k', 'k', 'k'];

%% Obstacles 
obstacle_res = 20;
for i=1:obstacle_res
    cloud_points1(:,i) = [3*sin(2*pi/obstacle_res*i) ,3*cos(2*pi/obstacle_res*i)]';
    cloud_points2(:,i) = [1.8*sin(2*pi/obstacle_res*i) ,1.8*cos(2*pi/obstacle_res*i)]';
    cloud_points3(:,i) = [0.6*sin(2*pi/obstacle_res*i) ,0.6*cos(2*pi/obstacle_res*i)]';
end
ob1 = Obstacle(cloud_points1);
ob2 = Obstacle(cloud_points2);
ob3 = Obstacle(cloud_points3);

%%% Transformation of obstacles (translation)
ob2.move_obstacle(3, -10);
ob3.move_obstacle(-7, 13);
% List of all obstacles 
oblist = [ob1, ob2, ob3];
% Vision of vehicle in meters 
r_vision = 8;

%% Simulation Colored Round CG
Tf = 600; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time
NT = ceil(Tf/Tc_cg); % simulation steps number
round = 1;

% Variables for PnP operation
ask_to_freeze = zeros(N, 1);
ask_to_plug = zeros(N, 1);
pluggable = false;
virtual = zeros(N, 1);

% References
r = cell(1, N);
% Planner
slope_list = [0.1 0.3 0.35 -0.1 -0.7 -0.2];
for i=1:N
    vehicle{i}.planner =  Border_Planner([Max_x, Max_y], slope_list(i), 'radius', 2, 'tolerance', 0.7);
end

% Pool perimeter
x_plot = -Max_x:0.1:Max_x;
y_plot = -Max_y:0.1:Max_y;

% Simulation
for t=1:NT
    for i=1:N
        if vehicle{i}.color == colors(round)
            % Receive msg from v{j} : ||v{i}-v{j}||∞ ≤ R
            for j=1:N
                if i~=j
                    d_ij = norm(vehicle{i}.ctrl_sys.sys.xi(1:2)-vehicle{j}.ctrl_sys.sys.xi(1:2));
                    if d_ij <= R
                        if d_ij > R__ && ~isempty(find(vehicle{j}.cg.id==vehicle{i}.cg.neigh, 1))
                            fprintf('Unplugged: %d - %d \n',i,j);
                            % Remove constraints and send plug-out message to v{j}
                            vehicle{i}.cg.remove_swarm_cnstr(j);
                            vehicle{j}.cg.remove_swarm_cnstr(i);
                            adj_matrix(i, j) = 0;
                            adj_matrix(j, i) = 0;
                        end
                        
                        if d_ij <= R_ && ... 
                                (vehicle{i}.pending_plugin == -1 || vehicle{i}.pending_plugin == 0)  && ...
                                vehicle{j}.pending_plugin == -1 && ... %%% No plugin pending
                                isempty(find(vehicle{j}.cg.id==vehicle{i}.cg.neigh, 1)) %  ||v{i}-v{j}||∞ ≤ R && not already pending plug-in request
                            fprintf('Plug-in request from %d to %d \n',i,j);
                            
                            % Plug-in request sent                           
                            vehicle{i}.pending_plugin = j;
                            % Send plug-in request message to v{j}
                            vehicle{j}.pending_plugin = i;
                            pluggable_status = vehicle{j}.cg.check([vehicle{j}.ctrl_sys.sys.xi; vehicle{j}.ctrl_sys.xci],...
                                [vehicle{i}.ctrl_sys.sys.xi; vehicle{i}.ctrl_sys.xci],vehicle{j}.g,vehicle{i}.g,[],d_min);
                            if pluggable_status %== 'success'                                
                                fprintf('Plug-in success: %d - %d \n',i,j);
                                vehicle{j}.pending_plugin = -1;
                                vehicle{i}.pending_plugin = -1;
                                vehicle{j}.cg.add_swarm_cnstr(i,'anticollision',d_min); % Setted from v{j} after check
                                vehicle{i}.cg.add_swarm_cnstr(j,'anticollision',d_min);
                                adj_matrix(i, j) = 1;
                                adj_matrix(j, i) = 1;
                            else
                                fprintf('Unpluggable: %d - %d. Need a virtual command \n',i,j);
                                pluggable_status = 'virtual_cmd';
                                
                                g_n = [];
                                for k = vehicle{j}.cg.neigh
                                    g_n = [g_n; vehicle{k}.g];
                                end    
                                
                                [g_j,g_ii] = vehicle{j}.cg.compute_virtual_cmd(vehicle{j}.g,vehicle{i}.ctrl_sys.sys.xi(1:2),g_n,[],d_min+0.1*d_min);
                                
                                g_n = [];
                                for k = vehicle{i}.cg.neigh
                                    g_n = [g_n; vehicle{k}.g];
                                end
                                
                                [g_i] = vehicle{i}.cg.compute_virtual_cmd_fixed(vehicle{i}.ctrl_sys.sys.xi(1:2),g_j,g_n,[],d_min+0.1*d_min);
                                
                                r{j} = g_j;
                                r{i} = g_i;
                                
                                % Send freeze request to v{j} neighbours
                                for k = vehicle{j}.cg.neigh
                                    vehicle{k}.pending_plugin = j;
                                    vehicle{k}.freeze = 1;  
                                end
                                for k = vehicle{i}.cg.neigh
                                    vehicle{k}.pending_plugin = i;
                                    vehicle{k}.freeze = 1;
                                end
                            end
                        elseif(d_ij <= R_ && not(vehicle{j}.pending_plugin == i) && not(vehicle{i}.pending_plugin == 0) &&...
                                not(vehicle{j}.pending_plugin == -1) && ...
                                vehicle{i}.pending_plugin == -1 && isempty(find(vehicle{j}.cg.id==vehicle{i}.cg.neigh, 1)))
                            fprintf('Vehicle %d already in plug-in operations. Computing proper virtual reference(%d) \n',j,i);
                            g_n = [];
                            for k = vehicle{i}.cg.neigh
                                g_n = [g_n; vehicle{k}.g];
                            end
                            [g_i,g_j] = vehicle{i}.cg.compute_virtual_cmd(vehicle{i}.g,r{j},g_n,[],d_min+0.1*d_min);
                            r{i} = g_i;
                            vehicle{i}.pending_plugin = 0;
                        end
                        
                         if d_ij <= R_ && ... 
                                vehicle{i}.pending_plugin == j && vehicle{i}.freeze == 0 && ...
                                isempty(find(vehicle{j}.cg.id==vehicle{i}.cg.neigh, 1)) 
                                
                                pluggable_status = vehicle{j}.cg.check([vehicle{j}.ctrl_sys.sys.xi; vehicle{j}.ctrl_sys.xci],...
                                    [vehicle{i}.ctrl_sys.sys.xi; vehicle{i}.ctrl_sys.xci],vehicle{j}.g,vehicle{i}.g,[],d_min);
                                
                                if pluggable_status %== 'success'
                                    fprintf('Plug-in success: %d - %d \n',i,j);
                                    vehicle{j}.pending_plugin = -1;
                                    vehicle{i}.pending_plugin = -1;
                                    vehicle{j}.cg.add_swarm_cnstr(i,'anticollision',d_min); % Setted from v{j} after check
                                    vehicle{i}.cg.add_swarm_cnstr(j,'anticollision',d_min);
                                    adj_matrix(i, j) = 1;
                                    adj_matrix(j, i) = 1;
                                    r{j} = r_{j};
                                    r{i} = r_{i};
                                    % Send unfreeze request to v{j} neighbours
                                    for k = vehicle{j}.cg.neigh
                                        vehicle{k}.pending_plugin = -1;
                                        vehicle{k}.freeze = 0;
                                    end
                                    for k = vehicle{i}.cg.neigh
                                        vehicle{k}.pending_plugin = -1;
                                        vehicle{k}.freeze = 0;
                                    end
                                end                        
                         end
                        if((vehicle{i}.pending_plugin==0) && norm(r{i}-vehicle{i}.ctrl_sys.sys.xi(1:2))< 0.1)
                            vehicle{i}.pending_plugin = -1;
                            r{i}=r_{i};
                        end
                    end
                end
            end
            
            if vehicle{i}.freeze == 0
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

                r{i} = vehicle{i}.planner.compute_reference(vehicle{i}, xa);
                r_{i} = r{i};

                %Vision Module of the vehicle that identifies the obstacle
                obseen = seen_obstacles(x(1:2), r_vision, oblist);  %%% MODIFY HERE

                if(isempty(obseen))
                    %If no obstacle has been found, then normal CG is been called
                    [g,yalmip_diag] = vehicle{i}.cg.compute_cmd(xa,r{i},g_n);
                else
                    % If an obstacle has been found the CG OA constraints
                    % are enables
                    ver = obseen.vertices;
                    [g,yalmip_diag] = vehicle{i}.cg.compute_cmd(xa, r{i}, g_n, ver);
                end

                if ~isempty(g)
                    vehicle{i}.g = g;
                else
                    disp('WARN: old references');
                    t,i
                end
            else
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
                            
                [g, yalmip_diag] = vehicle{i}.cg.compute_cmd(xa, vehicle{i}.ctrl_sys.sys.xi(1:2), g_n);
                if ~isempty(g)
                    vehicle{i}.g = g;
                else
                    disp('WARN: old references');
                    t,i
                end
            end
        end
    end
    for i=1:N
        vehicle{i}.ctrl_sys.sim(vehicle{i}.g,Tc_cg);
    end
    round = rem(round,length(colors))+1;

    % Live plot 
        f = figure(1);  clf;
        set(0, 'DefaultLineLineWidth', 3);
        hold on;
        grid on;
        axis equal

        % Plot borders
        % Left side plot
        plot(ones(size(y_plot))*x_plot(1), y_plot,'k');
        % Lower side plot
        plot(x_plot, ones(size(x_plot))*y_plot(1), 'k');
        % Right side plot
        plot(ones(size(y_plot))*x_plot(end), y_plot, 'k');
        % Upper side plot
        plot(x_plot, ones(size(x_plot))*y_plot(end), 'k');

        % Plot obstacles
        plot(ob1,'.-b'); plot(ob2,'.-b'); plot(ob3,'.-b');

        plot_color = update_color(vehicle, possible_colors, adj_matrix);
        for k=1:N
            if(isempty(vehicle{k}.cg.neigh))
                plot_color(k) = 'k';
            end
            for kk=vehicle{k}.cg.neigh
                plot([vehicle{k}.ctrl_sys.sys.x(1,end), vehicle{kk}.ctrl_sys.sys.x(1,end)],...
                    [vehicle{k}.ctrl_sys.sys.x(2,end), vehicle{kk}.ctrl_sys.sys.x(2,end)],  '--',...
                    'Color', plot_color(k), 'Linewidth', 2);
            end
        end
    
        plot_struct = [];
        legend_list = [];
        
        for k=1:N
            l_max = 13516;
            if(length(vehicle{k}.ctrl_sys.sys.x(1,:)) <= l_max)
                scia = 1:length(vehicle{k}.ctrl_sys.sys.x(1,:));
            else
                scia = length(vehicle{k}.ctrl_sys.sys.x(1,:))-13516:length(vehicle{k}.ctrl_sys.sys.x(1,:));
            end
            % Plot vehicles trajectory
            plot(vehicle{k}.ctrl_sys.sys.x(1, scia), vehicle{k}.ctrl_sys.sys.x(2, scia), '-.',...
                'Color', uint8([113 113 113]), 'LineWidth', 1.5);
            % Plot vehicles position 
            plot_struct = [plot_struct, plot(vehicle{k}.ctrl_sys.sys.x(1,end),vehicle{k}.ctrl_sys.sys.x(2,end), 'o',...
                'Color', plot_color(k), 'MarkerFaceColor', plot_color(k), 'MarkerSize', 4.5)];
            legend_list = [legend_list, sprintf("vehicle %d", k)];
            % Plot vehicles CG references
            plot(vehicle{k}.g(1), vehicle{k}.g(2), strcat(plot_color(k), 'x'), 'LineWidth', 1.2);
        end

        title('Crowded Patroling Simulation');
        xlabel('x [m]');
        ylabel('y [m]');
        legend(plot_struct, legend_list, 'NumColumns', ceil(N/2),'Location','northoutside');
        axis([-Max_x - 4, Max_x + 4, -Max_y - 4, Max_y + 4]);
        drawnow;
end
