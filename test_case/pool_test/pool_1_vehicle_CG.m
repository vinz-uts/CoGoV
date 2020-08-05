%% Clear workspace
clear all;
close all;

%% Load Pre-controlled vehicle system
addpath('../../marine_vehicle');        addpath(genpath('../../util'));
addpath(genpath('../../tbxmanager'));   addpath('../../CG');

vehicle_2DOF_model % WARN: Select the correct constraints matrix Hc, L.
vehicle = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle.init_position(0,0); % set vehicle's initial position

%% Constraints
% T*c ≤ b
Max_x = 2; % max abs of speed along x - [m/s]
Max_y = 2; % max abs of speed along y - [m/s]
Tm = 100; % max abs of motor thrust - [N]

T = [ 1  0  0  0 ;
     -1  0  0  0 ;
      0  1  0  0 ;
      0 -1  0  0 ;
      0  0  1  0 ;
      0  0 -1  0 ;
      0  0  0  1 ;
      0  0  0 -1 ];

b = [Max_x, Max_x, Max_y, Max_y, Tm, Tm, Tm, Tm]';

% Reference weight matrix
Psi = [ 1  0 ;
        0  1 ];
    
k0 = 10;

%%%%%% WARN: if not selected correct constraints matrix in vehicle_model.m
Hc = [eye(2) zeros(2)  zeros(2,2) ;
              -F              f      ];
L = zeros(4,2);
vehicle.ctrl_sys.Hc = Hc;    vehicle.ctrl_sys.L = L;
%%%%%%

%% Command Governor
vehicle.cg = CommandGovernor(Phi,G,Hc,L,T,b,Psi,k0);

%%
limits = [-Max_x, Max_x, Max_y, -Max_y];
pl = BorderPlanner(0.5, -0.7, limits, 0.15);

%% Discrete circle trajectory
Ns = 20; % trajectory discretization steps
l = 10;
r = [0:l/Ns:l-l/Ns; 0:l/Ns:l-l/Ns]; % references
delta = 0.1; % reference tollerance

%% Simulation
Tf = 25; % simulation time
Tc_cg = 1*vehicle.ctrl_sys.Tc; % Recalculation references time
N = ceil(Tf/Tc_cg); % simulation steps number
k = 1; % actual reference

figure(1);
hold on;
x_plot = -Max_x:0.1:Max_x;
y_plot = -Max_y:0.1:Max_y;
plot(ones(size(y_plot))*x_plot(1), y_plot);
plot(x_plot, ones(size(x_plot))*y_plot(1));
plot(ones(size(y_plot))*x_plot(end), y_plot);
plot(x_plot, ones(size(x_plot))*y_plot(end));
axis([-Max_x-1, Max_x + 1, -Max_y - 1, Max_y + 1]);
hold on;
vold=r;

for i=1:N
    x = vehicle.ctrl_sys.sys.xi; % vehicle current state
    xc = vehicle.ctrl_sys.xci; % controller current state
    xa = [x;xc];
%     if norm(x(1:2)-r(:,k),2) <= delta && k+1 <= Ns
%         k = k+1;
%     end
    [r, pl] = pl.compute_reference(vehicle.ctrl_sys.sys);
    vold=[vold,r];
    
    plot(r(1), r(2), 'rx');
    g = vehicle.cg.compute_cmd(xa, r);
    vehicle.ctrl_sys.sim(g,Tc_cg);
    plot(vehicle.ctrl_sys.sys.x(1, :),vehicle.ctrl_sys.sys.x(2, :), 'b.-'); %, vehicle.ctrl_sys.sys.x(1, end),vehicle.ctrl_sys.sys.x(2, end), 'o');
    drawnow;
end

%% Plot Simulation Result
% plot_simulation(vehicle.ctrl_sys);
figure(5);
% plot(vehicle.ctrl_sys.sys.x(1,:),vehicle.ctrl_sys.sys.x(2,:));