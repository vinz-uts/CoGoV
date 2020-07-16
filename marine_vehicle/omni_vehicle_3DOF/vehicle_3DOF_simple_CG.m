%% Clear workspace
clear all;
close all;

%% Load Pre-controlled vehicle system
addpath('../../marine_vehicle');        addpath(genpath('../../util'));
addpath(genpath('../../tbxmanager'));   addpath('../../CG');

vehicle_3DOF_model % WARN: Select the correct constraints matrix Hc, L.
vehicle = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle.init_position(0,0,-pi); % set vehicle's initial position

%% Constraints
% T*c ≤ b
Vx = 2; % max abs of speed along x - [m/s]
Vy = 2; % max abs of speed along y - [m/s]
Vt = pi; % max abs of speed around z - [rad/s]
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
        0  0  10 ];
    
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
r = [-2,-3,-pi/2]'; % position references
N = ceil(Tf/Tc_cg); % simulation steps number
epsilon = 0.01; % nearness precision

for i=1:N
    x = vehicle.ctrl_sys.sys.xi; % vehicle current state
    xc = vehicle.ctrl_sys.xci; % controller current state
    xa = [x;xc];
    if norm([r(1)-x(1) r(2)-x(2)]) > epsilon
        r = [r(1),r(2),atan2(r(2)-x(2),r(1)-x(1))]';
    end
    g = vehicle.cg.compute_cmd(xa,r);
    vehicle.ctrl_sys.sim(g,Tc_cg);
end

%% Plot Simulation Result
plot_simulation(vehicle.ctrl_sys);
figure();
plot_trajectory(vehicle.ctrl_sys.sys.x(1,:),vehicle.ctrl_sys.sys.x(2,:),vehicle.ctrl_sys.sys.x(3,:));
