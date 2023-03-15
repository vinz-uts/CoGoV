%%% Pool scenario

% We consider 3 vehicles moving in a pool starting from different initial positions and moving
% along straight line. When hitting the boundaries of the pool the new vehicle's trajectory starts
% with the same strike angle. Vehicles have to avoid collision among them.
% The topology is described by an Incidence matrix introduced below.


%% Clear workspace
clear;  close all;
set(0, 'DefaultLineLineWidth', 3);
%%%%%%%%%%%%%%%%%%%%% configurazion parameters %%%%%%%%%%%%%%
N = 10; % number of vehicles
init_radius = 11;

%%%%%%%%%%%%%%%%%%%% computed parameters %%%%%%%%%%%%%
theta_step = 2*pi/N;

%% Load vehicles' model matrices
addpath('../../../../marine_vehicle');      addpath('../../../../marine_vehicle/omni_vehicle_2DOF');
addpath(genpath('../../../../util'));       addpath('../../../../CG');
addpath(genpath('../../../../tbxmanager'));

vehicle_2DOF_model_2

%%%% Simulation Settings
%% Vehicles constraints
% Vehicles swarm position constraints
% ||(x,y)_i-(x,y)_j|| > d_min
d_min = 1; % minimum distance between vehicles - [m] 
d_max = 20;

% Vehicles position and torque constraints
Max_x = 15; % max position value along x - [m]
Max_y = 15; % max position value along y - [m]
T_max = 50; % max abs of motor thrust - [N]
%% Init vehicles
vehicle = cell(1, N);
r = cell(1, N);

% slope_list = [0.5610 0.5520 0.7623];
% slope_list = rand(1, N);
slope_list = [0.4, -0.3000, 0.4270, 0.5806, 0.5533, 0.3336, 0.2561, 0.3573, 0.4885, 0.4806];
slope_list(2) = -0.3;
% slope_list(10:12) = -0.56;
for i=1:N
    vehicle{i} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
    vehicle{i}.init_position(init_radius*cos(theta_step*(i-1)), init_radius*sin(theta_step*(i-1)));
    vehicle{i}.planner =  Border_Planner([Max_x, Max_y], slope_list(i), 'radius', 1, 'tolerance', 0.5);
    cloud_points(:,i) = [3*sin(2*pi/N*i) ,3*cos(2*pi/N*i)]';
end

pentagon = polyshape( cloud_points(1,:),cloud_points(2,:));
hypeblack = [];

% plot(pentagon); 
% return;
%% Net configuration
adj_matrix = zeros(N,N);

for i = 1:N
    adj_matrix(i,i) = -1;
end


%% Command Governor parameters
Psi = eye(2); % vehicle's references weight matrix
k0 = 10; % prediction horizon

%% Dynamic Command Governor
for i=1:N
    vehicle{i}.cg = DynamicDistribuitedCommandGovernor(i,Phi,G,Hc,L,Psi,k0, 'GUROBI');
    % Setting local constraints
    vehicle{i}.cg.add_vehicle_cnstr('position', [Max_x, Max_y],'thrust',T_max-0.074);
    for j=1:N
        if adj_matrix(i,j) == 1 % i,j is neighbour
            % Setting swarm constraints
            vehicle{i}.cg.add_swarm_cnstr(j,'anticollision',d_min);
        end
    end
end

% Color the net
colors = 1:N;
for i=1:length(colors)
    vehicle{i}.color = colors(i);
end


plot_color = ['b', 'g', 'k', 'r'];

%% Simulation Colored Round CG
Tf = 600; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time
NT = ceil(Tf/Tc_cg); % simulation steps number


% Perimeter plotting
x_plot = -Max_x:0.1:Max_x;
y_plot = -Max_y:0.1:Max_y;
axis([-Max_x-1, Max_x + 1, -Max_y - 1, Max_y + 1]);

%%%%% Binary variables usefull to PnP operation
ask_to_freeze = zeros(N, 1);
ask_to_plug = zeros(N, 1);
pluggable = false;
virtual = zeros(N, 1);

% Vector needed to track distance between vehicles
yalmiptime = [];
cputime = [];

pl = Phisical_Layer(N, 200, 0, 1);
pl.update(vehicle, 0);

u_list = {[], [], [], []};
time_to_solve = [];
g_list = [];

%% Communication constraints
R   = 7; % maximum distance of communications - [m]
R__ = 6; % maximum distance of connectivity - [m]
R_  = 4; % minimum distance for cooperation - [m]

% writerObj = VideoWriter('Final_Simulation_Obstacle3.avi');
% writerObj.FrameRate = 10;
% open(writerObj);

round = 1;
%%%%%% Sim cycle %%%%%%%%%
for t=1:NT
    for i=1:N
        if vehicle{i}.color == colors(round)
            % Receive msg from v{j} : ||v{i}-v{j}||∞ ≤ R
           
            for j=1:N
                if i~=j
                    d_ij = norm(vehicle{i}.ctrl_sys.sys.xi(1:2)-vehicle{j}.ctrl_sys.sys.xi(1:2));
                    if d_ij <= R
                        if d_ij > R__ && ~isempty(find(vehicle{j}.cg.id==vehicle{i}.cg.neigh))
                            fprintf('Unplugged: %d - %d \n',i,j);
                            % Remove constraints and send plug-out message to v{j}
                            vehicle{i}.cg.remove_swarm_cnstr(j);
                            vehicle{j}.cg.remove_swarm_cnstr(i);
                            % Plug-out request sent
                            pl.send_packet(i, j, '$C 00.00,00.00,00.00,00.00,00.00,00.00,00.00,00.00,00');
                            if(pl.look_for_packets(j))
                                 pl.get_packet(j); % Plug-out request received
                            end
                        end
                        
                        if d_ij <= R_ && ... 
                                (vehicle{i}.pending_plugin == -1 || vehicle{i}.pending_plugin == 0)  && ...
                                vehicle{j}.pending_plugin == -1 && ... %%% No plugin pending
                                isempty(find(vehicle{j}.cg.id==vehicle{i}.cg.neigh)) %  ||v{i}-v{j}||∞ ≤ R && not already pending plug-in request
                            fprintf('Plug-in request from %d to %d \n',i,j);
                            
                            % Plug-in request sent
                            pl.send_packet(i, j, '$C 00.00,00.00,00.00,00.00,00.00,00.00,00.00,00.00,00');
                            if(pl.look_for_packets(j))
                                 pl.get_packet(j); % Plug-in request received
                            end
                            vehicle{i}.pending_plugin = j;
                            % Send plug-in request message to v{j}
                            vehicle{j}.pending_plugin = i;
                            pluggable_status = vehicle{j}.cg.check([vehicle{j}.ctrl_sys.sys.xi; vehicle{j}.ctrl_sys.xci],...
                                [vehicle{i}.ctrl_sys.sys.xi; vehicle{i}.ctrl_sys.xci],vehicle{j}.g,vehicle{i}.g,[],d_min);
                            % Answer to plug-in request
                            pl.send_packet(j, i, '$C 00.00,00.00,00.00,00.00,00.00,00.00,00.00,00.00,00');
                            if(pl.look_for_packets(i))
                                 pl.get_packet(i); % Answer reveived
                            end
                            if pluggable_status %== 'success'                                
                                fprintf('Plug-in success: %d - %d \n',i,j);
                                vehicle{j}.pending_plugin = -1;
                                vehicle{i}.pending_plugin = -1;
                                vehicle{j}.cg.add_swarm_cnstr(i,'anticollision',d_min); % Setted from v{j} after check
                                vehicle{i}.cg.add_swarm_cnstr(j,'anticollision',d_min);
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
                                    pl.send_packet(j, k, '$C 00.00,00.00,00.00,00.00,00.00,00.00,00.00,00.00,00');
                                    if(pl.look_for_packets(k))
                                         pl.get_packet(k);
                                    end
                                end
                                for k = vehicle{i}.cg.neigh
                                    vehicle{k}.pending_plugin = i;
                                    vehicle{k}.freeze = 1;
                                    pl.send_packet(i, k, '$C 00.00,00.00,00.00,00.00,00.00,00.00,00.00,00.00,00');
                                    if(pl.look_for_packets(k))
                                         pl.get_packet(k);
                                    end
                                end
                            end
                        elseif(d_ij <= R_ && not(vehicle{j}.pending_plugin == i) && not(vehicle{i}.pending_plugin == 0) &&...
                                not(vehicle{j}.pending_plugin == -1) && ...
                                vehicle{i}.pending_plugin == -1 && isempty(find(vehicle{j}.cg.id==vehicle{i}.cg.neigh)))
                            fprintf('Vehicle %d already in plug-in operations. Computing proper virtual reference(%d) \n',j,i);
                            
                            % Request plug-in from i to j
                            pl.send_packet(i, j, '$C 00.00,00.00,00.00,00.00,00.00,00.00,00.00,00.00,00');
                            if(pl.look_for_packets(j))
                                 pl.get_packet(j);
                            end
                            % Request denied from j to i
                            pl.send_packet(j, i, '$C 00.00,00.00,00.00,00.00,00.00,00.00,00.00,00.00,00');
                            if(pl.look_for_packets(i))
                                 pl.get_packet(i);
                            end
                            
                            g_n = [];
                            for k = vehicle{i}.cg.neigh
                                g_n = [g_n; vehicle{k}.g];
                            end
                            
                            % Should consider also
                            % index=vehicle{j}.plug_in_request
                            [g_i,g_j] = vehicle{i}.cg.compute_virtual_cmd(vehicle{i}.g,r{j},g_n,[],d_min+0.1*d_min);
                            r{i} = g_i;
                            vehicle{i}.pending_plugin = 0;
                        end
                        
                         if d_ij <= R_ && ... 
                                vehicle{i}.pending_plugin == j && vehicle{i}.freeze == 0 && ...
                                isempty(find(vehicle{j}.cg.id==vehicle{i}.cg.neigh)) 
                                
                                pl.send_packet(i, j, '$C 00.00,00.00,00.00,00.00,00.00,00.00,00.00,00.00,00');
                                if(pl.look_for_packets(j))
                                     pl.get_packet(j);
                                end
                                pluggable_status = vehicle{j}.cg.check([vehicle{j}.ctrl_sys.sys.xi; vehicle{j}.ctrl_sys.xci],...
                                    [vehicle{i}.ctrl_sys.sys.xi; vehicle{i}.ctrl_sys.xci],vehicle{j}.g,vehicle{i}.g,[],d_min);
                                
                                if pluggable_status %== 'success'
                                    fprintf('Plug-in success: %d - %d \n',i,j);
                                    vehicle{j}.pending_plugin = -1;
                                    vehicle{i}.pending_plugin = -1;
                                    vehicle{j}.cg.add_swarm_cnstr(i,'anticollision',d_min); % Setted from v{j} after check
                                    vehicle{i}.cg.add_swarm_cnstr(j,'anticollision',d_min);
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
%                 [g,s] = vehicle{i}.cg.compute_cmd(xa,r{i},g_n,cloud_points,[]);
                [g,s] = vehicle{i}.cg.compute_cmd(xa,r{i},g_n);
                if ~isempty(g)
                    vehicle{i}.g = g;
                    cputime= [cputime,s.solvertime];
                    yalmiptime=[yalmiptime,s.yalmiptime];
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
                
                [g,s] = vehicle{i}.cg.compute_cmd(xa,vehicle{i}.ctrl_sys.sys.xi(1:2),g_n);
                if ~isempty(g)
                    vehicle{i}.g = g;
                    time_to_solve = [time_to_solve, [s.solvertime; s.yalmiptime]];
                else
                    disp('WARN: old references');
                    t,i
                end
            end
        end
    end
    % Simulate for Tc_cg
    for i=1:N
%          u_list{i} = [u_list{i}, vehicle{i}.ctrl_sys.sim(vehicle{i}.g,Tc_cg)];
        vehicle{i}.ctrl_sys.sim(vehicle{i}.g,Tc_cg);
        for j = vehicle{i}.cg.neigh
           pl.send_packet(i, j, '$C 00.00,00.00,00.00,00.00,00.00,00.00,00.00,00.00,00');
           if(pl.look_for_packets(j))
               pl.get_packet(j);
           end
        end
    end
    round = rem(round,length(colors))+1;
    pl.update(vehicle, t*Tc_cg);
    
   % Live plot
    figure(1);  clf;
    title('Pool PnP Simulation');
    xlabel('x [m]');
    ylabel('y [m]');
    grid on
    legend_list = {'vehicle 1 trajectory', 'vehicle 1 g', ...
                    'vehicle 2 trajectory', 'vehicle 2 g', ...
                    'vehicle 3 trajectory', 'vehicle 3 g', ...
                    'vehicle 4 trajectory', 'vehicle 4 g'};
    
    hold on;
    %plot(pentagon); 
    hold on;
    for k=1:N
        for kk=vehicle{k}.cg.neigh
            plot([vehicle{k}.ctrl_sys.sys.x(1,end), vehicle{kk}.ctrl_sys.sys.x(1,end)],...
                [vehicle{k}.ctrl_sys.sys.x(2,end), vehicle{kk}.ctrl_sys.sys.x(2,end)],strcat(plot_color(mod(kk, 4)+1), '--'), 'Linewidth', 2);
        end
    end
    
    % Left side plot
    plot(ones(size(y_plot))*x_plot(1), y_plot,'k');
    % Lower side plot
    plot(x_plot, ones(size(x_plot))*y_plot(1), 'k');
    % Right side plot
    plot(ones(size(y_plot))*x_plot(end), y_plot, 'k');
    % Upper side plot
    plot(x_plot, ones(size(x_plot))*y_plot(end), 'k');
    
    plot_struct = [];
    len_scia = 7000;
    for k=1:N
        if(length(vehicle{k}.ctrl_sys.sys.x(1,:)) < len_scia)
            scia = 1:length(vehicle{k}.ctrl_sys.sys.x(1,:));
        else
            scia = length(vehicle{k}.ctrl_sys.sys.x(1,:)) - len_scia:length(vehicle{k}.ctrl_sys.sys.x(1,:));
        end
        % Plot vehicles trajectory
        plot_struct = [plot_struct, plot(vehicle{k}.ctrl_sys.sys.x(1,scia),vehicle{k}.ctrl_sys.sys.x(2,scia), strcat(plot_color(mod(k, 4)+1), '-.'))];
        
        axis([-Max_x-1, Max_x + 1, -Max_y - 1, Max_y + 1]);
        
        % Plot vehicles position 
        plot(vehicle{k}.ctrl_sys.sys.x(1,end),vehicle{k}.ctrl_sys.sys.x(2,end), strcat(plot_color(mod(k, 4)+1), 'o'),'MarkerFaceColor',plot_color(mod(k, 4)+1),'MarkerSize', 7);
        
        % Plot vehicles CG references
        plot_struct = [plot_struct, plot(vehicle{k}.g(1), vehicle{k}.g(2), strcat(plot_color(mod(k, 4)+1), 'x'))];
    end
    %plot(pl);
    %legend(plot_struct, legend_list);
    drawnow;
%     writeVideo(writerObj, getframe(gcf));
end
% close(writerObj);
% -0.6567 -0.2902 0.7545
% 0.5610 0.5520 0.7423

vehicle_data = [];
nx = 4;

for i=1:N
    data = struct("x", [vehicle{i}.ctrl_sys.sys.x; vehicle{i}.ctrl_sys.xc], "u", u_list{i}); %...
                  %"u", -vehicle{i}.ctrl_sys.Fa(:,1:nx)*vehicle{i}.ctrl_sys.sys.x + vehicle{i}.ctrl_sys.Fa(:,nx+1:end)*vehicle{i}.ctrl_sys.xc);
                  
    vehicle_data = [vehicle_data, data];
end
sim_data = struct('system', vehicle_data, 'time', (1:NT)*Tc, "Tc", Tc, 'd_min', d_min, "T_max", T_max);

virtual_time = 0:sim_data.time(end)/length((sim_data.system(1).x(1, :))):sim_data.time(end);
virtual_time(end) = [];
plot_window = 1:length(virtual_time)-1;
save("pool_dyn.mat", "sim_data");

full_time = time_to_solve(1, :);
fprintf("Minimum time: %.3f\t Maximum time: %.3f, Mean time: %.3f\n", ...
         min(full_time),  max(full_time),  mean(full_time));

% figure
% virtual_time = vehicle{1}.ctrl_sys.sys.t;
% len = length(virtual_time)-1;
% l1 = sim_data.system(1).x(1:2, 1:len) - sim_data.system(2).x(1:2, 1:len);
% l2 = sim_data.system(1).x(1:2, 1:len) - sim_data.system(3).x(1:2, 1:len);
% l4 = sim_data.system(2).x(1:2, 1:len) - sim_data.system(3).x(1:2, 1:len);
% distance = zeros(3, size(l1, 2));
% for i=1:length(l1)
%     distance(1, i) = norm(l1(:, i), inf);
%     distance(2, i) = norm(l2(:, i), inf);
%     distance(3, i) = norm(l4(:, i), inf);
% end
% hold on
% 
% plot(virtual_time(plot_window), distance(1, plot_window));
% plot(virtual_time(plot_window), distance(2, plot_window));
% plot(virtual_time(plot_window), distance(3, plot_window));
% if(~isempty(sim_data.d_min))
%     hold on
%     plot(virtual_time(plot_window), ones(length(virtual_time(plot_window)), 1)*sim_data.d_min-0.04, 'r--');
%     hold off
% end
% ylabel('distance [m]');
% xlabel('time [s]');
% legend('1-2 distance', '1-3 distance', '2-3 distance', 'minimum admissible distance');

figure

vect_len = [];
for dd = 1:N
    vect_len = [vect_len; length(vehicle{dd}.ctrl_sys.sys.x(1:2, :))];
end
len = min(vect_len);
dd_i = 2
l{1} = vehicle{6}.ctrl_sys.sys.x(1:2, 1:len) - vehicle{4}.ctrl_sys.sys.x(1:2, 1:len);
l{2} = vehicle{6}.ctrl_sys.sys.x(1:2, 1:len) - vehicle{5}.ctrl_sys.sys.x(1:2, 1:len);
l{3} = vehicle{6}.ctrl_sys.sys.x(1:2, 1:len) - vehicle{1}.ctrl_sys.sys.x(1:2, 1:len);
% l{10} = vehicle{1}.ctrl_sys.sys.x(1:2, 1:len) - vehicle{11}.ctrl_sys.sys.x(1:2, 1:len);
% l{11} = vehicle{1}.ctrl_sys.sys.x(1:2, 1:len) - vehicle{12}.ctrl_sys.sys.x(1:2, 1:len);
distance = zeros(11, size(l{1}, 2));      
hold on
for dd = 1:3
    for i=1:length(l{dd})
        vect = l{dd};
        distance(dd, i) = norm(vect(:, i), inf);
    end
end
plot_struct = [];
plot_struct = [plot_struct, plot(vehicle{dd_i(1)}.ctrl_sys.sys.t(1:end), distance(1, 1:end))];
plot_struct = [plot_struct, plot(vehicle{dd_i(1)}.ctrl_sys.sys.t(1:end), distance(2, 1:end))];
plot_struct = [plot_struct, plot(vehicle{dd_i(1)}.ctrl_sys.sys.t(1:end), distance(3, 1:end))];
plot_struct = [plot_struct, plot(vehicle{1}.ctrl_sys.sys.t, ones(length(vehicle{1}.ctrl_sys.sys.t), 1)*d_min, 'r--')];
hold off;
legend_list = ["distance 6-4", "distance 6-5", "distance 6-1", "minimum safe distance"];

    
grid on
legend(plot_struct, legend_list);
title("Relative distances")
ylabel('distances [m]');
xlabel('time [s]');
%legend('1-2 distance', '1-3 distance', '2-3 distance', 'minimum admissible distance');
