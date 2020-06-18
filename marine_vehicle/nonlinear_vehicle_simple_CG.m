%% Clear workspace
clear all;
close all;

%% Load Pre-controlled vehicle system
addpath('../util');  addpath(genpath('../tbxmanager')); addpath('../CG');

vehicle_nonlinear_model % WARN: Select the correct constraints matrix Hc, L.
vehicle = ControlledVehicle(ControlledSystem_LQI(NonlinearSystem(f,nx,nu),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle.init_position(0,0); % set vehicle's initial position

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

%%%%%% WARN: if not selected correct constraints matrix in nonlinear_vehicle_model.m
Hc = [ zeros(2,2) eye(2)  zeros(2,2) ;
              -F              f      ];
L = zeros(4,2);
vehicle.ctrl_sys.Hc = Hc;    vehicle.ctrl_sys.L = L;
%%%%%%

%% Command Governor
vehicle.cg = CommandGovernor(Phi,G,Hc,L,T,b,Psi,k0);

%% Simulation
Tf = 3; % simulation time
Tc_cg = 1*vehicle.ctrl_sys.Tc; % Recalculation references time
r = [2,3]'; % position references
N = ceil(Tf/Tc_cg); % simulation steps number

for i=1:N
    x = vehicle.ctrl_sys.sys.xi; % vehicle current state
    xc = vehicle.ctrl_sys.xci; % controller current state
    xa = [x;xc];
    g = vehicle.cg.compute_cmd(xa,r);
    vehicle.ctrl_sys.sim(g,Tc_cg);
end

%% Plot Simulation Result
plot_simulation(vehicle.ctrl_sys);
