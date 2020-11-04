%%
clear;
close all;
% Simulation of a potential collision with obstacles that move 

vehicle_2DOF_model_2 % WARN: Select the correct constraints matrix Hc, L.


vehicle = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));


vehicle.init_position(0,0); % set vehicle's initial position

%% Constraints
% T*c ≤ b
Vx = 0.5; % max abs of speed along x - [m/s]
Vy = 0.5; % max abs of speed along y - [m/s]
Tm = 20; % max abs of motor thrust - [N]

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
Hc = [ zeros(2,2) eye(2)  zeros(2,2) ;
              -F              f      ];
L = zeros(4,2);

%%%%%% WARN: use for quadratic norm constraints 
% Hc = [ zeros(4,4)  zeros(4,2) ;
%         -F         f      ];
% L = zeros(6,2);

vehicle.ctrl_sys.Hc = Hc;    vehicle.ctrl_sys.L = L;
%%%%%%

%% Command Governor
vehicle.cg = CommandGovernorCAC(Phi,G,Hc,L,T,b,Psi,k0);


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


ob1 = Obstacle(v1a,v2a,v3a,v4a);
ob2 = Obstacle(v1b,v2b,v3b,v4b);
ob3 = Obstacle(v1b,v2b,v3b,v4b);
ob4 = Obstacle(v1c,v2c,v3c,v4c);
ob5 = Obstacle(v1b,v2b,v3b,v4b);

ob1.move_obstacle(-1, 16);
ob2.move_obstacle(-2.5, 6.5);
ob3.move_obstacle(-0.5, 3);
ob4.move_obstacle(-4, 10);
ob5.move_obstacle(-1.5, 12);

oblist = [ob1, ob2, ob3, ob4, ob5];

Tf = 200; % simulation time
Tc_cg = 1*vehicle.ctrl_sys.Tc; % Recalculation references time
r = [0,20]'; % position references
N = ceil(Tf/Tc_cg); % simulation steps number

limitLeft=-3; 
limitRight=3;

r_vision = 2.5;
%%%%% Data collection about optimization time %%%%%%%%
% names are self-explanatory


for i=1:N
    
    %%% Augmented System State
    x = vehicle.ctrl_sys.sys.xi; % vehicle current state
    xc = vehicle.ctrl_sys.xci; % controller current state
    xa = [x;xc];
    
    obseen = ob1.seen_obstacles(x(1:2), r_vision, oblist);
    
    if(isempty(obseen))
        ver = [];
    else
        ver = obseen.vertices;
    end
    
    %%% Compute vehicle command given reference 
    [g,s] = vehicle.cg.compute_cmd(xa,r,ver);
    vehicle.ctrl_sys.sim(g,Tc_cg);

    
    
    
    %%% Plotting section 
    plot_2Dof_vehicle(vehicle, r, r_vision, 'RangeAxis', [-10 10 0 20]);
    hold on;
    plot(ob1,'.-b');
    plot(ob2,'.-b');
    plot(ob3,'.-b');
    plot(ob4,'.-b');
    plot(ob5,'.-b');
    
    if(not(isempty(obseen)))
        plot(obseen,'-r');
    end

    plot(ones(1,21)*limitLeft,0:20,'--r');
    plot(ones(1,21)*limitRight,0:20,'--r');
    drawnow;
    hold off;
end


