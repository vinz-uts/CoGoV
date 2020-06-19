%% Clear workspace
clear all;
close all;

%% Load Pre-controlled vehicle system
addpath('../util');  addpath(genpath('../tbxmanager')); addpath('../CG');

nonlinear_vehicle_model % WARN: Select the correct constraints matrix Hc, L.
vehicle = ControlledVehicle(ControlledSystem_LQI(NonlinearSystem('nonlinear_vehicle_fun',nx,nu),Tc,Fa,Cy,Phi,G,Hc,L));
%vehicle.init_position(0,0); % set vehicle's initial position

%% Constraints
% T*c ≤ b
Vx = 2; % max abs of speed along x - [m/s]
Vy = 2; % max abs of speed along y - [m/s]
Vt = pi/4; % max abs of speed around z - [rad/s]
Tm = 100; % max abs of motor thrust - [N]

T = [ 1  0  0  0  0  0 ;
     -1  0  0  0  0  0 ;
      0  1  0  0  0  0 ;
      0 -1  0  0  0  0 ;
      0  0  1  0  0  0 ;
      0  0 -1  0  0  0 ;
      0  0  0  1  0  0 ;
      0  0  0 -1  0  0 ;
      0  0  0  0  1  0 ;
      0  0  0  0 -1  0 ;
      0  0  0  0  0  1 ;
      0  0  0  0  0 -1 ];
  
b = [Vx,Vx,Vy,Vy,Vt,Vt,Tm,Tm,Tm,Tm,Tm,Tm]';

% Reference weight matrix
Psi = [ 1  0  0 ;
        0  1  0 ;
        0  0  2 ];
    
k0 = 10;

%%%%%% WARN: if not selected correct constraints matrix in nonlinear_vehicle_model.m
Hc = [ zeros(3,3) eye(3)  zeros(3,3) ;
              -F              f      ];
L = zeros(6,3);
vehicle.ctrl_sys.Hc = Hc;    vehicle.ctrl_sys.L = L;
%%%%%%

%% Command Governor
vehicle.cg = CommandGovernor(Phi,G,Hc,L,T,b,Psi,k0);

%% Simulation
Tf = 10; % simulation time
Tc_cg = 1*vehicle.ctrl_sys.Tc; % Recalculation references time
r = [2,3,0]'; % position references
N = ceil(Tf/Tc_cg); % simulation steps number

for i=1:N
    x = vehicle.ctrl_sys.sys.xi; % vehicle current state
    u = x(4);   v = x(5);
    x(4) = cos(x(3))*u - sin(x(3))*v;
    x(5) = sin(x(3))*u + cos(x(3))*v;
    xc = vehicle.ctrl_sys.xci; % controller current state
    xa = [x;xc];
    g = vehicle.cg.compute_cmd(xa,r);
    vehicle.ctrl_sys.sim(g,Tc_cg);
end

%% Plot Simulation Result
plot_simulation(vehicle.ctrl_sys);
