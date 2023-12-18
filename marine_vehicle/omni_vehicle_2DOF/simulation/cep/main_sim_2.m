%% Clear workspace
clear;  close all;

%% Load vehicles' model matrices
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

% Vehicles 1 and 4 will stand still 
vehicle{1}.planner = LinePlanner(vehicle{1}.ctrl_sys.sys.xi(1:2));
vehicle{4}.planner = LinePlanner(vehicle{4}.ctrl_sys.sys.xi(1:2));

%% Net configuration
adj_matrix =[-1  1  0  0  0;
              1 -1  1  0  0;
              0  1 -1  1  0;
              0  0  1 -1  1;
              0  0  0  1 -1];


%% Vehicles constraints
% Vehicles swarm position constraints
% ||p_i - p_j|| < d_max
d_max = 15;  % maximum distance between vehicles - [m]

% Vehicles torque constraints
T_max = 30; % max abs of motor thrust - [N]

%% Dynamic Command Governor
% Command Governor parameters
Psi = 0.01*eye(2); % vehicle's references weight matrix
k0 = 20; % prediction horizon
for i=1:N
    vehicle{i}.cg = DynamicDistribuitedCommandGovernor(i, Phi, G, Hc, L, Psi, k0, 'gurobi');
    vehicle{i}.cg.add_vehicle_cnstr('thrust', T_max);
    for j=1:N
        if(adj_matrix(i,j) == 1) % i, j is neighbour
            vehicle{i}.cg.add_swarm_cnstr(j, 'proximity', d_max);
        end
    end
end

%% Color the net (Fixed colors)
colors = 1:N;
for i=1:N
    vehicle{i}.color = colors(i);
end
plot_color = ['b', 'g', 'c', 'r', 'm'];

%% Simulation Colored Round CG
Tf = 340000; % simulation time - [s]
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time - [s]
NT = ceil(Tf/Tc_cg); % simulation steps number
round = 1;

% Simulation
for t=1:NT
    for i=1:N
        if vehicle{i}.color == colors(round)
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
            
            r = vehicle{i}.planner.compute_reference(vehicle{i},xa); % Compute local reference
            [g, diagnostic] = vehicle{i}.cg.compute_cmd(xa, r, g_n); % Compute admissible local reference using CG
            if ~isempty(g)
                vehicle{i}.g = g;
            else
                disp('WARN: old references');
                t,i
            end
        end
    end
    for i=1:N
        vehicle{i}.ctrl_sys.sim(vehicle{i}.g, Tc_cg);
    end
    round = rem(round,length(colors))+1;
    
    % Live plot
    plot_struct = []; % Legend data
    legend_list = []; % Legend data

    set(0, 'DefaultLineLineWidth', 3);
    figure(1);
    clf;
    hold on;

    plot(vehicle{2}.planner); % Plot reference trajectory v1
    plot(vehicle{3}.planner); % Plot reference trajectory v2
    plot(vehicle{5}.planner); % Plot reference trajectory v3
    for k = 1:N
        for kk=k:-1:1
            if(adj_matrix(k, kk) == 1)
                plot([vehicle{k}.ctrl_sys.sys.x(1,end), vehicle{kk}.ctrl_sys.sys.x(1,end)],...
                    [vehicle{k}.ctrl_sys.sys.x(2,end), vehicle{kk}.ctrl_sys.sys.x(2,end)],  '--',...
                    'Color', plot_color(k), 'Linewidth', 2);
            end
        end
    end
    for k=1:N
        % Plot vehicles trajectories
        plot(vehicle{k}.ctrl_sys.sys.x(1,:),vehicle{k}.ctrl_sys.sys.x(2,:), strcat(plot_color(k), '-.'), 'LineWidth', 3);
        % Plot vehicles positions 
        plot_struct = [plot_struct, plot(vehicle{k}.ctrl_sys.sys.x(1,end),vehicle{k}.ctrl_sys.sys.x(2,end), strcat(plot_color(k), 'o'), 'MarkerFaceColor', plot_color(k), 'MarkerSize', 8)];
        % Plot vehicles CG references
        plot(vehicle{k}.g(1), vehicle{k}.g(2), strcat(plot_color(k), 'x'), 'LineWidth', 1.5);
        legend_list = [legend_list, sprintf("vehicle %d", k)];
    end
    hold off;
    title('Connectivity Simulation');
    xlabel('x [m]'); ylabel('y [m]');
    legend(plot_struct, legend_list, 'Location', 'northwest');
    axis([0 40 0 40]);
    grid on;
    axis equal;
    drawnow;
end
