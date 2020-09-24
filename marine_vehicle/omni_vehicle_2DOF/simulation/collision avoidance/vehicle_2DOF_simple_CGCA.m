%% Command Governor
% Governor for robust constraints reference generation.
% 
% States:                   Inputs:             Constraints:
% z = [x,y,dx,dy]'          g = [x*,y*]'        c = [z3,z4,u1,u2]'
% 
% Pre-controlled system model:
% z(k+1) = Î¦*z(k) + G*g(k)
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
vehicle.init_position(3,3.5); % set vehicle's initial position

%% Constraints
% T*c â‰¤ b
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
vehicle.cg = CommandGovernorCA(Phi,G,Hc,L,T,b,Psi,k0);

%% Simulation
Tf = 15; % simulation time
Tc_cg = 1*vehicle.ctrl_sys.Tc; % Recalculation references time
r = [10,3]';
% r = [9.5,4]'; % position references

%%%%% Data collection about optimization time %%%%%%%%
% names are self-explanatory
cputime =[];
yalmiptime = [];

ve1 = [5,2]';
ve2 = [5,4]';
ve3 = [8,2]';
ve4 = [8,4]';
% ve1 = [5,3]';
% ve2 = [5,5]';
% ve3 = [8,5]';
% ve4 = [8,3]';

figure
x_plot = 5:0.1:8;
y_plot = 2:0.1:4;
axis([-5 13 -5 13]);
hold on;
plot(ve1(1),ve1(2),'*');
plot(ve2(1),ve2(2),'*');
plot(ve3(1),ve3(2),'*');
plot(ve4(1),ve4(2),'*');
plot(ones(size(y_plot))*x_plot(1), y_plot);
plot(x_plot, ones(size(x_plot))*y_plot(1));
plot(ones(size(y_plot))*x_plot(end), y_plot);
plot(x_plot, ones(size(x_plot))*y_plot(end));


N = ceil(Tf/Tc_cg); % simulation steps number
for i=1:N
    x = vehicle.ctrl_sys.sys.xi; % vehicle current state
    xc = vehicle.ctrl_sys.xci; % controller current state
    xa = [x;xc];
    [g,s] = vehicle.cg.compute_cmd(xa,r,ve1,ve2,ve3,ve4);
    
    %%% Si può pensare di introdurre un modulo che generi un riferimento
    %%% safe, e questo nuovo riferimento viene passato al CG. Questo
    %%% dovrebbe garantire feasibility. Oppure si può pensare ad un modulo
    %%% che faccia il check della feasibility. 
    
    vehicle.ctrl_sys.sim(g,Tc_cg);
    cputime= [cputime,s.solvertime];
    yalmiptime=[yalmiptime,s.yalmiptime];
    plot(g(1),g(2),'xb');
    plot(vehicle.ctrl_sys.sys.x(1,:),vehicle.ctrl_sys.sys.x(2,:),'.');
    plot(r(1),r(2),'*y');
    drawnow;
    
end






%% Plot Simulation Result
%figure
%plot_simulation(vehicle.ctrl_sys);


% plot(vehicle.ctrl_sys.sys.x(1,:),vehicle.ctrl_sys.sys.x(2,:),'.');
