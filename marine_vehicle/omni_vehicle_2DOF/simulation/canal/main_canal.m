%%
clear;
close all;

% Simulation of a potential collision with obstacles in a river 


%%% Loading vehicle parameters and control variables
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
        [g,s,hy] = vehicle.cg.compute_cmd(xa,r,[],ver);
    end
    
    %%% Compute vehicle command given reference 
    vehicle.ctrl_sys.sim(g,Tc_cg);

    
    %%% Plotting section 
    plot_2Dof_vehicle(vehicle, r, r_vision, 'RangeAxis', [-10 10 0 20]);
    hold on;
    
    % Plotting Command Governor Reference
    plot(g(1),g(2),'xk');
    % Plotting Hyperplane found by CG OA strategy
    plot(hy);
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
    
    % Plotting of the river margins 
    plot(ones(1,21)*-(xmax),0:20,'--r');
    plot(ones(1,21)*xmax,0:20,'--r');
    drawnow;
    hold off;
end


