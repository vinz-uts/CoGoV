%% Command Governor
% Governor for robust constraints reference generation.
% 
% States:                   Inputs:             Constraints:
% z = [x,y,dx,dy]'          g = [x*,y*]'        c = [z3,z4,u1,u2]'
% 
% Pre-controlled system model:
% z(k+1) = Φ*z(k) + G*g(k)
% y(k) = Hy*z(k)
% c(k) = Hc*z(k) + L*g(k)
% 
% Constraints:
% -Vx < dx < Vx         -T_max < Tx < T_max
% -Vy < dy < Vy         -T_max < Ty < T_max
%

%% Clear workspace
clear all;
close all;

%% Load Pre-controlled vehicle system
addpath('../../marine_vehicle');        addpath(genpath('../../util'));
addpath(genpath('../../tbxmanager'));   addpath('../../CG');

vehicle_2DOF_model_2 % WARN: Select the correct constraints matrix Hc, L.
vehicle = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle.init_position(0.5,0.5); % set vehicle's initial position

%% Constraints
% T*c ≤ b
Vx = 0.2; % max abs of speed along x - [m/s]
Vy = 0.2; % max abs of speed along y - [m/s]
Tm = 10; % max abs of motor thrust - [N]

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
Psi = [ 1  0;
        0  1 ];
    
k0 = 10;

%%%%%% WARN: use this for infin. norm constraint as usual
% Hc = [ zeros(2,2) eye(2)  zeros(2,2) ;
%               -F              f      ];
% L = zeros(4,2);

%%%%%% WARN: use for quadratic norm constraints 
Hc = [ zeros(4,4)  zeros(4,2) ;
        -F         f      ];
L = zeros(6,2);

vehicle.ctrl_sys.Hc = Hc;    vehicle.ctrl_sys.L = L;
%%%%%%

%% Command Governor
vehicle.cg = CommandGovernor(Phi,G,Hc,L,T,b,Psi,k0);

%% Simulation
%%% Planner
xSamples = [1, 0.8, 0.5, 0, -0.25, -0.6, -0.75 -1, -1, -0.5, 0, 0.35, 0.7]';
ySamples = [0, 0.45, 0.7, 1, 0.9, 0.75, 0.3, 0, -0.5, -0.7, -1, -1, -0.45]';

ptp = Polar_trajectory_planner([xSamples(1:floor(length(xSamples)/2)); xSamples(1)], [ySamples(1:floor(length(ySamples)/2)); ySamples(1)]);


Tf = 30; % simulation time
Tc_cg = 1*vehicle.ctrl_sys.Tc; % Recalculation references time
r = [3,1]'; % position references
N = ceil(Tf/Tc_cg); % simulation steps number

%%%%% Data collection about optimization time %%%%%%%%
% names are self-explanatory
cputime =[];
yalmiptime = [];
figure(1);
axis([0 4 0 4]);
% 
% plot(ptp, 'k');
hold on
for i=1:N
    x = vehicle.ctrl_sys.sys.xi; % vehicle current state
    xc = vehicle.ctrl_sys.xci; % controller current state
    xa = [x;xc];
%     [g,s] = vehicle.cg.compute_cmd(xa,ptp.compute_reference(vehicle.ctrl_sys.sys));
     [g,s] = vehicle.cg.compute_cmd(xa,r);
    vehicle.ctrl_sys.sim(g,Tc_cg);
    cputime= [cputime,s.solvertime];
    yalmiptime=[yalmiptime,s.yalmiptime];
    plot(vehicle.ctrl_sys.sys.x(1,:),vehicle.ctrl_sys.sys.x(2,:),'b.');
    plot(r(1), r(2), 'bo');
    plot(g(1),g(2), 'kx');
    hold on
    drawnow
end


