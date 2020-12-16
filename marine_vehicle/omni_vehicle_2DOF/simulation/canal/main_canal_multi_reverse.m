%%
clear;
close all;
% Simulation of a potential collision with obstacles that move 

vehicle_2DOF_model_2

%% Vehicles
N = 2; % number of vehicles

% Vehicle 1
vehicle{1} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{1}.init_position(-0.5,0);
% Vehicle 2
vehicle{2} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{2}.init_position(0.5,20);

%% Net configuration
%   1
%  / \
% 2   3
adj_matrix = [-1  1;
			   1 -1];


%% Vehicles constraints
% Vehicles swarm position constraints
% ||(x,y)_i-(x,y)_j|| > d_min
d_min = 0.2; % minimum distance between vehicles - [m]

% Vehicles input/speed constraints
Vx = 0.5; % max abs of speed along x - [m/s]
Vy = 0.5; % max abs of speed along y - [m/s]
T_max = 20; % max abs of motor thrust - [N]
x_max = 3; % max value of x position
y_max = 50; % max value of y position


%% Command Governor parameters
Psi = 0.01*eye(2); % vehicle's references weight matrix
k0 = 20; % prediction horizon

%% Dynamic Command Governor

for i=1:N
    vehicle{i}.cg = DynamicDistribuitedCommandGovernor(i,Phi,G,Hc,L,Psi,k0, 'gurobi');
    vehicle{i}.cg.add_vehicle_cnstr('position',[x_max,y_max],'thrust',T_max,'speed',[Vx,Vy]);
    for j=1:N
        if adj_matrix(i,j) == 1 % i,j is neighbour
            vehicle{i}.cg.add_swarm_cnstr(j,'anticollision',d_min);
        end
    end
end

%% Color the net
colors = [0,1];
vehicle{1}.color = colors(1);
vehicle{2}.color = colors(2);

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
ob2.move_obstacle(-2.5, 6.5);
ob3.move_obstacle(-0.5, 3);
ob4.move_obstacle(-4, 10);
ob5.move_obstacle(-1.5, 12);

% List of all obstacles 
oblist = [ob1, ob2, ob3, ob4, ob5];

Tf = 50; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % Recalculation references time
r{1} = [-0.5,20]'; % position references
r{2} = [0.5,0]'; % position references
NT = ceil(Tf/Tc_cg); % simulation steps number
round = 1;

% Vector used to track distance between the two vehicles
dist = [];

%%% Vision of vehicle in meters 
r_vision = 2.5;

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
                [g,s] = vehicle{i}.cg.compute_cmd(xa,r{i},g_n,ver);
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
    round = rem(round,length(colors))+1;
    
    %%%%%%% live plot %%%%%%%
  
    % Trajectory
    plot_2Dof_vehicle(vehicle{1}, r{1}, r_vision, 'RangeAxis', [-10 10 0 20],'Color', 'k','MarkerSize', 6);
    hold on;
    plot_2Dof_vehicle(vehicle{2}, r{2}, r_vision, 'RangeAxis', [-10 10 0 20],'Color', 'g','MarkerSize', 6);
    hold on;
    
    % Plotting of the obstacles (black)
    plot(ob1,'.-b');
    plot(ob2,'.-b');
    plot(ob3,'.-b');
    plot(ob4,'.-b');
    plot(ob5,'.-b');
    
    % Plotting of the closest obstacle found (red)
    if(not(isempty(obseen)))
        plot(obseen,'-r');
    end
    
    plot(ones(1,21)*-(x_max),0:20,'--r');
    plot(ones(1,21)*x_max,0:20,'--r');
    
    if(t==1)
        title('OA Canal Simulation Reverse');
        xlabel('x [m]');
        ylabel('y [m]');
    end
    
    drawnow;
    hold off;
    %%%%%%%%%%%%%
    
    
    dist = [dist, norm((vehicle{1}.ctrl_sys.sys.x(1:2,end)-vehicle{2}.ctrl_sys.sys.x(1:2,end)))];
end


figure;
plot(1:NT, dist);
hold on;
plot(1:NT,d_min*ones(1,length(1:NT)));
title('Distance between vehicles');
xlabel('time [s]');
ylabel('distance [m]');



