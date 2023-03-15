clear; close all;

vehicle_2DOF_model_2

%% Vehicles constraints
% Vehicles swarm position constraints
% ||(x,y)_i-(x,y)_j|| > d_min
d_min = 0.3; % minimum distance between vehicles - [m]

% Vehicles torque constraints
T_max = 5; % max abs of motor's thrust - [N]

% Vehicles speed constraints
%v_max = 20; % max abs of vehicle's speed - [m/s]


%% Vehicles init
N = 2; % number of vehicles

% Vehicle 1
vehicle{1} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{1}.init_position(1,0);
% Vehicle 2
vehicle{2} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{2}.init_position(0,1);


%% Command Governor parameters
Psi = 0.01*eye(2); % vehicle's references weight matrix
k0 = 10; % prediction horizon

vehicle{1}.cg = DynamicDistribuitedCommandGovernor(1, Phi, G, Hc, L, Psi, k0, 'bmibnb');
vehicle{2}.cg = DynamicDistribuitedCommandGovernor(2, Phi, G, Hc, L, Psi, k0, 'bmibnb');
vehicle{1}.cg.add_vehicle_cnstr('thrust', T_max);
vehicle{1}.cg.add_swarm_cnstr(2, 'anticollision', d_min);
vehicle{2}.cg.add_vehicle_cnstr('thrust', T_max);
vehicle{2}.cg.add_swarm_cnstr(1, 'anticollision', d_min);


%% Simulation
%%%%%%%%%% Frontal Collision References
r{1} = vehicle{2}.ctrl_sys.sys.xi(1:2);
r{2} = vehicle{1}.ctrl_sys.sys.xi(1:2);

NT = 10;
T = 0.1;
i = 0; % first round
for t=0:T:NT
    % Current vehicle
    x = vehicle{i+1}.ctrl_sys.sys.xi;
    xc = vehicle{i+1}.ctrl_sys.xci;
    xa = [x;xc];
    % Neighbour vehicle
    x = vehicle{rem(i+1,2)+1}.ctrl_sys.sys.xi;
    xc = vehicle{rem(i+1,2)+1}.ctrl_sys.xci;
    xa = [xa;x;xc];
    g_n = vehicle{rem(i+1,2)+1}.g;
    
    g = vehicle{i+1}.cg.compute_cmd(xa, r{i+1}, g_n);
    vehicle{i+1}.g = g;
    i = rem(i+1,2);
    
    % Simulate systems
    vehicle{1}.ctrl_sys.sim(vehicle{1}.g, T)
    vehicle{2}.ctrl_sys.sim(vehicle{2}.g, T)
    
    %%%%%%% live plot %%%%%%%
    figure(1);  
    clf;
    grid on;
    title('Frontal Collision Simulation');
    xlabel('x [m]');
    ylabel('y [m]');
    hold on;
    
    plot_struct = [];
    legend_list = {'vehicle 1 trajectory', 'vehicle 1 g', 'vehicle 2 trajectory', 'vehicle 2 g'};
    plot_color = ['b', 'g'];
    for k=1:2

    	% Plot vehicles trajectory
        plot_struct = [plot_struct, plot(vehicle{k}.ctrl_sys.sys.x(1,:),vehicle{k}.ctrl_sys.sys.x(2,:), strcat(plot_color(k), '-.'))];
        
        % Plot vehicles position 
        plot(vehicle{k}.ctrl_sys.sys.x(1,end),vehicle{k}.ctrl_sys.sys.x(2,end), strcat(plot_color(k), 'o'),'MarkerFaceColor',plot_color(k),'MarkerSize',7);
        
        % Plot vehicles CG references
        plot_struct = [plot_struct, plot(vehicle{k}.g(1), vehicle{k}.g(2), strcat(plot_color(k), 'x'))];
        
    end
    
    legend(plot_struct, legend_list)
    drawnow;
    
end