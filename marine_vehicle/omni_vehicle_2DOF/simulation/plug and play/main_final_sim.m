clear;
close all;

set(0, 'DefaultLineLineWidth', 3);

%% Load vehicles' model matrices
addpath('../../../../marine_vehicle');      addpath('../../../../marine_vehicle/omni_vehicle_2DOF');
addpath(genpath('../../../../util'));       addpath('../../../../CG');
addpath(genpath('../../../../tbxmanager'));


vehicle_2DOF_model_2

%% Vehicles
N = 4; % number of vehicles
% Vehicle 1
vehicle{1} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{1}.init_position(-0.2,-0.1);
% Vehicle 2
vehicle{2} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{2}.init_position(-0.2404,0.9290);

vehicle{3} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{3}.init_position(-0.9982,0.334);

vehicle{4} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{4}.init_position(3, 3);
% end

%% Planners for Circular Path Following

xSamples = 1.2*[1, 0, -1, 0]';
ySamples = 1.2*[0, 1, 0, -1]';

vehicle{1}.planner = Polar_trajectory_planner(xSamples, ySamples, 'radius', 0.5);
vehicle{2}.planner = Polar_trajectory_planner(xSamples, ySamples,'recovery',20,'clockwise',false,'rec_from_collision',true, 'radius', 0.5);
vehicle{3}.planner = Polar_trajectory_planner(xSamples, ySamples, 'radius', 0.5);

r_vect = [-2,-2,3,0.5,-3,1,3,3];
%r_vect = [-2,-2];

vehicle{4}.planner = LinePlanner(r_vect, 'recovery', 20);

vehicle{1}.planner.transform(1, [1, 0]);
vehicle{2}.planner.transform(1, [-1, 0]);
vehicle{3}.planner.transform(1, [0, 1]);

%% Net configuration

adj_matrix = [-1  1  1  0;
    1 -1  1  0;
    1  1 -1  0;
    0  0  0 -1];

%% Vehicles constraints
% Vehicles swarm position constraints
% ||(x,y)_i-(x,y)_j|| < d_max
% ||(x,y)_i-(x,y)_j|| > d_min
d_max = 1.5; % maximum distance between vehicles - [m]
d_min = 0.6; % minimum distance between vehicles - [m]

% Vehicles input/speed constraints
Vx = 0.5; % max abs of speed along x - [m/s]
Vy = 0.5; % max abs of speed along y - [m/s]
T_max = 50; % max abs of motor thrust - [N]

%% Command Governor parameters
Psi = 1000*eye(2); % vehicle's references weight matrix
k0 = 15; % prediction horizon

%% Dynamic Command Governor
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
colors = [0,1,2];
vehicle{1}.color = colors(1);
vehicle{2}.color = colors(2);
vehicle{3}.color = colors(2);
%%% The new vehicle does not follow the Turn-Based policy at the beginning
% vehicle{4}.color = colors(3);

%% Simulation Colored Round CG
Tf = 200; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time
NT = ceil(Tf/Tc_cg); % simulation steps number
round = 1;

%%%%% Data collection about optimization time %%%%%%%%
% names are self-explanatory
cputime =[];
yalmiptime = [];

plot_color = ['b', 'g', 'k', 'r'];
plaggable = false;

%%%%%%%%%% Frontal Collision References
r{1} = vehicle{1}.ctrl_sys.sys.xi(1:2) + [2, 0]';
r{2} = vehicle{2}.ctrl_sys.sys.xi(1:2) + [2, 0]';
r{3} =  vehicle{3}.ctrl_sys.sys.xi(1:2) + [2, 0]';
r{4} = r{3};
% rN_tmp = r{4};
%%%%% Binary variables usefull to PnP operation
ask_to_freeze = [false, false, false, false]';
ask_to_plug = [false, false, false, false];
pluggable = false;
virtual = [false, false, false, false]; 

%%%%% Virtual references
r_stari = [];
r_starn1 = [];

%%% Vector for distance between i and N+1 for plotting purposes
u_list = {[], [], [], []};
time_to_solve = [];
g_list = [];

writerObj = VideoWriter('Plugin_Video.avi');
writerObj.FrameRate = 10;
open(writerObj);
%%%%%% Sim cycle %%%%%%%%%
for t=1:NT
    for i=1:N
        
        %%% The vehicle doesnt make part of the formation, hence it
        %%% computes its reference outside the Turn based approach 
        if(isempty(vehicle{4}.color) && i == 4)
            for ii = 1:N-1
                if(not(isempty(find(ask_to_plug == 1, 1)))) % one plug at time
                    break;
                end     
                
                % Plug IN
                if(norm(vehicle{ii}.ctrl_sys.sys.xi(1:2) - vehicle{N}.ctrl_sys.sys.xi(1:2)) < 4*d_min ... % close enough
                        && not(ask_to_plug(ii)) ... % pug-in request not already sent
                        && isempty(find(vehicle{ii}.cg.neigh == N, 1))) % not already coupled                
                    ask_to_plug(ii) = true;
                    fprintf('Plug-in request sent to vehicle %d.\n', ii);
                    break;
                end
                
                % Plug OUT 
                if(norm(vehicle{ii}.ctrl_sys.sys.xi(1:2) - vehicle{N}.ctrl_sys.sys.xi(1:2)) > 4*d_min ... % far enough
                        && not(isempty(find(vehicle{ii}.cg.neigh == N, 1))))
                    
                    fprintf('Plug-out request sent to vehicle %d.\n', ii);
                    vehicle{ii}.cg.remove_swarm_cnstr(N);
                    vehicle{N}.cg.remove_swarm_cnstr(ii);
                    adj_matrix(N, ii) = 0;
                    adj_matrix(ii, N) = 0;
                    
                    fprintf('Plug-out request satisfied for vehicle %d.\n', ii);
                end
            end
            
            x = vehicle{4}.ctrl_sys.sys.xi; % vehicle current state
            xc = vehicle{4}.ctrl_sys.xci; % controller current state
            xa = [x;xc];
            g_n = [];
            for j=1:N
                if adj_matrix(4,j) == 1 % i,j is neighbour
                    g_n = [g_n;vehicle{j}.g];
                    x = vehicle{j}.ctrl_sys.sys.xi; % vehicle current state
                    xc = vehicle{j}.ctrl_sys.xci; % controller current state
                    xa = [xa;x;xc];
                end
            end
            
            if(not(virtual(4)))
                 r{4} = vehicle{4}.planner.compute_reference(vehicle{4},xa);
            end
            
            vehicle{4}.g = vehicle{4}.cg.compute_cmd(xa, r{4}, g_n);
         
        end
        
        if vehicle{i}.color == colors(round)
            
            %%%% Condition to plug verified by vehicle N+1
            if(colors(round) == vehicle{N}.color)
                for ii = 1:N-1
                    if(not(isempty(find(ask_to_plug == 1, 1)))) % one plug at time
                        break;
                    end
                    
                    % Plug IN
                    if(norm(vehicle{ii}.ctrl_sys.sys.xi(1:2) - vehicle{N}.ctrl_sys.sys.xi(1:2)) < 3*d_min + 0.3 ... % close enough
                            && not(ask_to_plug(ii)) ... % pug-in request not already sent
                            && isempty(find(vehicle{ii}.cg.neigh == N, 1))) % not already coupled
                        
                        ask_to_plug(ii) = true;
                        fprintf('Plug-in request sent to vehicle %d.\n', ii);
                        break;
                    end
                    % Plug OUT
                    if(norm(vehicle{ii}.ctrl_sys.sys.xi(1:2) - vehicle{N}.ctrl_sys.sys.xi(1:2)) > 4*d_min ... % far enough
                            && not(isempty(find(vehicle{ii}.cg.neigh == N, 1))))                       
                        fprintf('Plug-out request sent to vehicle %d.\n', ii);
                        vehicle{ii}.cg.remove_swarm_cnstr(N);
                        vehicle{N}.cg.remove_swarm_cnstr(ii);
                        adj_matrix(N, ii) = 0;
                        adj_matrix(ii, N) = 0;                        
                        fprintf('Plug-out request satisfied for vehicle %d.\n', ii);
                    end              
                end
            end
            
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
            
            if(not(virtual(i)))
                r{i} = vehicle{i}.planner.compute_reference(vehicle{i},xa);
            end

            if(norm(vehicle{i}.ctrl_sys.sys.xi(1:2) - vehicle{N}.ctrl_sys.sys.xi(1:2)) < d_min && not(i == 4)) % validity check
                warning('Collided');
            end
            
            %%%% Pluggability condition verified by vehicle i
            if(ask_to_plug(i)) % someone asked for a plug-in operation
                pluggable = vehicle{i}.cg.check([vehicle{i}.ctrl_sys.sys.xi; vehicle{i}.ctrl_sys.xci], ...
                    [vehicle{N}.ctrl_sys.sys.xi; vehicle{N}.ctrl_sys.xci], vehicle{i}.g, vehicle{N}.g, [], d_min);
                if(pluggable)
                    fprintf('Plug-in request accepted (vehicle %d and %d).\n', i, N);
                else
                    fprintf('Plug-in request denied (vehicle %d and %d).\n', i, N);
                end
            end
            
            %%%% Now that we have pluggability result we procede attaching
            %%%% the vehicles
            if(pluggable && ask_to_plug(i))
                adj_matrix(N, i) = 1;
                adj_matrix(i, N) = 1;
                vehicle{i}.cg.add_swarm_cnstr(N, 'anticollision', d_min);
                vehicle{N}.cg.add_swarm_cnstr(i, 'anticollision', d_min);
                fprintf('Communication between vehicle %d and %d added.\n', i, N);

                ask_to_plug(i) = false;
                pluggable = false;
                r_stari = [];
                r_starn1 = [];
                ask_to_freeze(vehicle{i}.cg.neigh) = false;
                vehicle{4}.color = colors(3);
                %%% Since adj_matrix is modified, we need to update state
                %%% variables with new neighboor
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
                
                r{i} =  vehicle{i}.planner.compute_reference(vehicle{i},xa);
                r{4} =  vehicle{4}.planner.compute_reference(vehicle{4},[]);
                
                virtual(4)= false;
                virtual(i)= false;   
                %%% If they are not pluggable we procede finding a virtual
                %%% reference and freezing references for neig of
                
            elseif(ask_to_plug(i) && isempty(find(vehicle{i}.cg.neigh == N, 1)))
                %%% If virtual references were not computed then compute
                %%% (ONLY ONCE)
                if(isempty(r_stari) && isempty(r_starn1))
                    disp('Virtual references computation');
                    [r_stari,r_starn1] = vehicle{i}.cg.compute_virtual_cmd(r{i},vehicle{N}.ctrl_sys.sys.xi(1:2), g_n, [], d_min);
                    ask_to_freeze(vehicle{i}.cg.neigh) = true;
                    virtual(4) = true; 
                    virtual(i) = true; 
                    vehicle{4}.color = [];
                end
                r{i} = r_stari;
                r{N} = r_starn1;
            end
            
            
            
            %%% If the vehicles do not need to compute reference since they
            %%% are plugging
            if((ask_to_freeze(i))) % If I have to freeze my reference
                g = vehicle{i}.g;
            else
                [g,s] = vehicle{i}.cg.compute_cmd(xa, r{i}, g_n);
                %g = r{i};
            end
            
            
            if ~isempty(g)
                vehicle{i}.g = g;
                time_to_solve = [time_to_solve, [s.solvertime; s.yalmiptime]];
            else
                disp('WARN: old references');
                t,i
            end
            
            if i == 4
                g_list = [g_list,  vehicle{i}.g];
            end
        end
    end

    for i=1:N
        u_list{i} = [u_list{i}, vehicle{i}.ctrl_sys.sim(vehicle{i}.g,Tc_cg)];
    end
    round = rem(round,length(colors))+1;
    
    %%%%%%% live plot %%%%%%%
    figure(1);
    clf;
    hold on;
    plot_struct = [];
    legend_list = [];
    title('Patrolling simulation');
    xlabel('x [m]');
    ylabel('y [m]');
    plot(vehicle{1}.planner)
    plot(vehicle{2}.planner)
    plot(vehicle{3}.planner)
    axis([-3 3 -3 3]);
    grid on;
    for k=1:N
        for kk=vehicle{k}.cg.neigh
            plot([vehicle{k}.ctrl_sys.sys.x(1,end), vehicle{kk}.ctrl_sys.sys.x(1,end)],...
                [vehicle{k}.ctrl_sys.sys.x(2,end), vehicle{kk}.ctrl_sys.sys.x(2,end)], strcat(plot_color(k), '--'), 'Linewidth', 2);
        end
    end
    len_scia = 7000;
    for k=1:N
        if(length(vehicle{k}.ctrl_sys.sys.x(1,:)) < len_scia)
            scia = 1:length(vehicle{k}.ctrl_sys.sys.x(1,:));
        else
            scia = length(vehicle{k}.ctrl_sys.sys.x(1,:)) - len_scia:length(vehicle{k}.ctrl_sys.sys.x(1,:));
        end
        % Plot vehicles trajectory
        plot_struct = [plot_struct, plot(vehicle{k}.ctrl_sys.sys.x(1,scia),vehicle{k}.ctrl_sys.sys.x(2,scia), strcat(plot_color(k), '-.'))];
       
        plot(vehicle{k}.ctrl_sys.sys.x(1,end),vehicle{k}.ctrl_sys.sys.x(2,end), strcat(plot_color(k), 'o'),'MarkerFaceColor',plot_color(k),'MarkerSize',7);
        legend_list = [legend_list, sprintf("vehicle %d", k)];
        %%%% live plot %%%%
%         plot(r{k}(1), r{k}(2), strcat(plot_color(k), 'o'));
        plot_struct = [plot_struct, plot(vehicle{k}.g(1), vehicle{k}.g(2), strcat(plot_color(k), 'x'))];
        legend_list = [legend_list, sprintf("g %d", k)];
        %%%%%%%%%%%%
    end
    
    hold off;
    legend(plot_struct, legend_list, 'Location','southwest','NumColumns', N);

    drawnow;
    writeVideo(writerObj,im2frame(print('-RGBImage', strcat('-r', '300'))));
end
close(writerObj);
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
save("plginout_patt_no_cg.mat", "sim_data");

full_time = time_to_solve(1, :);
fprintf("Minimum time: %.3f\t Maximum time: %.3f, Mean time: %.3f\n", ...
         min(full_time),  max(full_time),  mean(full_time));

figure
len = length(virtual_time)-1;
l1 = sim_data.system(1).x(1:2, 1:len) - sim_data.system(2).x(1:2, 1:len);
l2 = sim_data.system(1).x(1:2, 1:len) - sim_data.system(3).x(1:2, 1:len);
l3 = sim_data.system(1).x(1:2, 1:len) - sim_data.system(4).x(1:2, 1:len);
l4 = sim_data.system(2).x(1:2, 1:len) - sim_data.system(3).x(1:2, 1:len);
l5 = sim_data.system(2).x(1:2, 1:len) - sim_data.system(4).x(1:2, 1:len);
l6 = sim_data.system(3).x(1:2, 1:len) - sim_data.system(4).x(1:2, 1:len);
distance = zeros(6, size(l1, 2));
for i=1:length(l1)
    distance(1, i) = norm(l1(:, i));
    distance(2, i) = norm(l2(:, i));
    distance(3, i) = norm(l3(:, i));
    distance(4, i) = norm(l4(:, i));
    distance(5, i) = norm(l5(:, i));
    distance(6, i) = norm(l6(:, i));
end
hold on
plot(virtual_time(plot_window), distance(1, plot_window));
plot(virtual_time(plot_window), distance(2, plot_window));
plot(virtual_time(plot_window), distance(3, plot_window));
plot(virtual_time(plot_window), distance(4, plot_window));
plot(virtual_time(plot_window), distance(5, plot_window));
plot(virtual_time(plot_window), distance(6, plot_window));
if(~isempty(sim_data.d_min))
    hold on
    plot(virtual_time(plot_window), ones(length(virtual_time(plot_window)), 1)*sim_data.d_min-0.04, 'r--');
    hold off
end
ylabel('distance [m]');
xlabel('time [s]');
legend('1-2 distance', '1-3 distance', '1-4 distance', '2-3 distance', '2-4 distance', '3-4 distance', 'minimum admissible distance');