%% Clear workspace
clear; close all;

%% Load vehicles' model matrices 
vehicle_2DOF_model_2


%% Vehicles
N = 2; % number of vehicles
% Vehicle 1
vehicle{1} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B), Tc, Fa, Cy, Phi, G, Hc, L));
vehicle{1}.init_position(1,0);
% Vehicle 2
vehicle{2} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B), Tc, Fa, Cy, Phi, G, Hc, L));
vehicle{2}.init_position(4.5,0);


%% Net configuration
adj_matrix = [-1  1;
			   1 -1];

%% Vehicles constraints parameters
% Vehicles torque constraints
T_max = 0.5; % max abs of motor thrust - [N]
% Vehicles swarm position constraints
d_min = 0.3; % ||p_i - p_j|| > d_min [m]


%% Dynamic Command Governor
% Command Governor parameters
Psi = 0.01*eye(2); % vehicle's references weight matrix
k0 = 40; % prediction horizon
% Create Command Governors
for i=1:N
    vehicle{i}.cg = DynamicDistribuitedCommandGovernor(i, Phi, G, Hc, L, Psi, k0, 'gurobi');
    vehicle{i}.cg.add_vehicle_cnstr('thrust', T_max); % add local constraints
    for j=1:N
        if adj_matrix(i,j) == 1 % i,j is neighbour
            vehicle{i}.cg.add_swarm_cnstr(j, 'anticollision', d_min); % add shared constraints
        end
    end
end


%% Color the net (Fixed colors)
colors = [0,1];     plot_colors = ['b', 'g'];
vehicle{1}.color = colors(1);
vehicle{2}.color = colors(2);


%% Simulation Colored Round CG
Tf = 80; % simulation time - [s]
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time - [s]
NT = ceil(Tf/Tc_cg); % simulation steps number
round = 1;

% References
r{1} = [5, -5]';
r{2} = [1, -5]';

% Planner
vehicle{1}.planner = LinePlanner(r{1}, 'radius', 0.6);
vehicle{2}.planner = LinePlanner(r{2}, 'radius', 0.6);

% Simulation
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
            r = vehicle{i}.planner.compute_reference(vehicle{i},xa); % Compute local reference
            [g,s] = vehicle{i}.cg.compute_cmd(xa, r, g_n); % Compute admissible local reference using CG
            if ~isempty(g)
                vehicle{i}.g = g;
            else
                disp('WARN: No admmisible solution! Old reference is used.');
                t,i
            end
        end
    end
    for i=1:N
        vehicle{i}.ctrl_sys.sim(vehicle{i}.g, Tc_cg);
    end

    round = rem(round,length(colors))+1; % Generate next round color
     
    % Live plot
    set(0, 'DefaultLineLineWidth', 3);
    figure(1);  
    axis([0 5 -5 0]);
    axis manual;
    clf;
    grid on;
    title('Collision Simulation');
    xlabel('x [m]');    ylabel('y [m]');
    hold on;
    
    plot_struct = [];
    legend_list = {'vehicle 1 trajectory', 'vehicle 1 g', 'vehicle 2 trajectory', 'vehicle 2 g'};
    for k=1:N
    	% Plot vehicles trajectories
        plot_struct = [plot_struct, plot(vehicle{k}.ctrl_sys.sys.x(1,:),vehicle{k}.ctrl_sys.sys.x(2,:), strcat(plot_colors(k), '-.'))];        
        % Plot vehicles positions 
        plot(vehicle{k}.ctrl_sys.sys.x(1,end),vehicle{k}.ctrl_sys.sys.x(2,end), strcat(plot_colors(k), 'o'),'MarkerFaceColor',plot_colors(k),'MarkerSize',7);        
        % Plot vehicles CG references
        plot_struct = [plot_struct, plot(vehicle{k}.g(1), vehicle{k}.g(2), strcat(plot_colors(k), 'x'))];
    end
    
    plot(5, -5, 'ob');
    plot(0, -5, 'og');
    axis([0 5 -5 0]);
    axis equal;
    
    legend(plot_struct, legend_list,'Location','east');
    drawnow;
    %%%%%%%%%%%%%%%%%%%%%%%%% 
end