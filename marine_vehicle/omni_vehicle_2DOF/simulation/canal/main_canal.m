%%
clear;
close all;

% Simulation of a potential collision with obstacles in a river 


%%% Loading vehicle parameters and control variables
addpath('../../../../marine_vehicle');      addpath('../../../../marine_vehicle/omni_vehicle_2DOF');
addpath(genpath('../../../../util'));       addpath('../../../../CG');
addpath(genpath('../../../../tbxmanager'));

vehicle_2DOF_model_2 % WARN: Select the correct constraints matrix Hc, L.

vehicle = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));

% Set vehicle's initial position
vehicle.init_position(0,0); 

%% Constraints

% Vehicles input/speed constraints
Vx = 0.5; % max abs of speed along x - [m/s]
Vy = 0.5; % max abs of speed along y - [m/s]
T_max = 20; % max abs of motor thrust - [N]
x_max = 3; % max value of x position
y_max = 50; % max value of y position


%% Command Governor parameters
Psi = 0.01*eye(2); % vehicle's references weight matrix
k0 = 10; % prediction horizon

%% Dynamic Command Governor
vehicle.cg = DynamicDistribuitedCommandGovernor(1,Phi,G,Hc,L,Psi,k0, 'gurobi');
vehicle.cg.add_vehicle_cnstr('position',[x_max,y_max],'thrust',T_max,'speed',[Vx,Vy]);

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
ob2.move_obstacle(-2.2, 7.5);
ob3.move_obstacle(-0.5, 3);
ob4.move_obstacle(-4, 10);
ob5.move_obstacle(-1.5, 12);

% List of all obstacles 
oblist = [ob1, ob2, ob3, ob4, ob5];


%% Simulation 
Tf = 200; % simulation time
Tc_cg = 1*vehicle.ctrl_sys.Tc; % Recalculation references time

%%% Vehicle Reference 

r = [0,20]'; % position references
N = ceil(Tf/Tc_cg); % simulation steps number

%%% Hyperplane found (plot purpose) 
hy=[];

%%% Vision of vehicle in meters 
r_vision = 3;

figure;
legend_list = {'vehicle trajectory', 'vehicle g', 'hyperplane'};
dist = [];
time = [];
time2 = [];
g_list = [];
% writerObj = VideoWriter('canal.avi');
% open(writerObj);
g_ = zeros(2, 1);
for i=1:N
    
    %%% Augmented System State
    x = vehicle.ctrl_sys.sys.xi; % vehicle current state
    xc = vehicle.ctrl_sys.xci; % controller current state
    xa = [x;xc];
    
    %%% Vision Module of the vehicle that identifies the obstacle 
    
    obseen = seen_obstacles(x(1:2), r_vision, oblist);
    
    
    if(isempty(obseen))
        %%% If no obstacle has been found, then normal CG is been called
        [g,s] = vehicle.cg.compute_cmd(xa,r,[]);
    else
        %%% If an obstacle has been found, it is the closest, and the CG
        %%% takes into consideration the obstacle
        ver = obseen.vertices;
        hy_old = hy;
        [g,s,hy] = vehicle.cg.compute_cmd(xa,r,[], ver);
        if((r-g_list(:, end))'*Psi*(r-g_list(:, end)) <= (r-g)'*Psi*(r-g))
            g = g_list(:, end);
            hy = hy_old;
        end
        time = [time, i*Tc_cg];
        tmpx = ver(1,:)-vehicle.ctrl_sys.sys.x(1,end);
        tmpy = ver(2,:)-vehicle.ctrl_sys.sys.x(2,end);
        tmp = [tmpx; tmpy];
        norm_vect = vecnorm(tmp);
        [m, k] = min(norm_vect);

        dist = [dist, norm(vehicle.ctrl_sys.sys.x(1:2,end)-  ver(:, k))];
    end

    g_list = [g_list, g];
    time2 = [time2, i*Tc_cg];
    
    %%% Compute vehicle command given reference 
    vehicle.ctrl_sys.sim(g,Tc_cg);
    
    %%% Plotting section
    clf;
    plot_struct = [];
    title('Canal Simulation');
    xlabel('x[m]');
    ylabel('y[m]');
    grid on;
    canal = polyshape([-3 -3 3 3], [20, 0, 0, 20]);
    hold on
    plot(canal, 'FaceColor', 'cyan')
    plot_struct = [plot_struct, plot_2Dof_vehicle(vehicle, r, r_vision, 'RangeAxis', [-10 10 0 20], 'LineWidth', 3, 'D_min_style', '--')];
    
    % Plotting Command Governor Reference
    plot_struct = [plot_struct, plot(g(1),g(2),'xk')];
    % Plotting Hyperplane found by CG OA strategy
    plot(hy);
    % Plotting of the obstacles (black)
    for ii = 1:length(oblist)
        plot(oblist(ii), '.-b');
    end
    
%     plot(ones(1,21)*-(x_max),0:20,'b');
%     plot(ones(1,21)*x_max,0:20,'b');
%    Plotting of the closest obstacle found (red)
    if(not(isempty(obseen)))
        plot(obseen,'-r');
    end
    
    % Plotting of the river margins 
%     legend(plot_struct, legend_list);
    drawnow;
%     writeVideo(writerObj, getframe(gcf));
    hold off;
end
% close(writerObj);

