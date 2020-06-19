%% Clear workspace
clear all;
close all;

%% Load Pre-controlled vehicle system
addpath('../../util');  addpath(genpath('../../tbxmanager')); addpath('../../CG');

vehicle_model % WARN: Select the correct constraints matrix Hc, L.
vehicle = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle.init_position(5,0); % set vehicle's initial position

%% Constraints
% T*c ≤ b
Vx = 2; % max abs of speed along x - [m/s]
Vy = 2; % max abs of speed along y - [m/s]
Tm = 100; % max abs of motor thrust - [N]

T = [ 1  0  0  0 ;
     -1  0  0  0 ;
      0  1  0  0 ;
      0 -1  0  0 ;
      0  0  1  0 ;
      0  0 -1  0 ;
      0  0  0  1 ;
      0  0  0 -1 ];
  
b = [Vx,Vx,Vy,Vy,Tm,Tm,Tm,Tm]';

% Reference weight matrix
Psi = [ 1  0 ;
        0  1 ];
    
k0 = 10;

%%%%%% WARN: if not selected correct constraints matrix in vehicle_model.m
Hc = [ zeros(2,2) eye(2)  zeros(2,2) ;
              -F              f      ];
L = zeros(4,2);
vehicle.ctrl_sys.Hc = Hc;    vehicle.ctrl_sys.L = L;
%%%%%%

%% Command Governor
vehicle.cg = CommandGovernor(Phi,G,Hc,L,T,b,Psi,k0);

%% Discrete circle trajectory
C = [0,0]; % circle center
rho = 5; % circle radius - [m]
Ns = 20; % trajectory discretization steps
th = 0:(2*pi/Ns):2*pi;
r = [C(1)+rho.*cos(th) ; C(2)+rho.*sin(th)]; % references
delta = 0.1; % reference tollerance

%% Simulation
Tf = 25; % simulation time
Tc_cg = 1*vehicle.ctrl_sys.Tc; % Recalculation references time
N = ceil(Tf/Tc_cg); % simulation steps number
k = 1; % actual reference

for i=1:N
    x = vehicle.ctrl_sys.sys.xi; % vehicle current state
    xc = vehicle.ctrl_sys.xci; % controller current state
    xa = [x;xc];
    if norm(x(1:2)-r(:,k),2) <= delta && k+1 <= Ns
        k = k+1;
    end
    g = vehicle.cg.compute_cmd(xa,r(:,k));
    vehicle.ctrl_sys.sim(g,Tc_cg);
end

%% Plot Simulation Result
plot_simulation(vehicle.ctrl_sys);
figure(5);
plot(vehicle.ctrl_sys.sys.x(1,:),vehicle.ctrl_sys.sys.x(2,:));