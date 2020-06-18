%% Model parameters
m = 10; % mass - [Kg]
c = 1;  % friction coefficient - [-]
J = 3;
%% Time continuous model
A = [ 0    0    1    0  ;
      0    0    0    1  ;
      0    0  -c/m   0  ;
      0    0    0  -c/m ];
B = [ 0   0  ;
      0   0  ;
      1/m  0  ;
      0  1/m ];
%% Time discrete model
% Using Forward Euler approximation: dz*Tc = z(k+1) - z(k)
Tc = 0.1; % sampling time - [s]
Ad = [ 1   0     Tc        0     ;
       0   1     0         Tc    ;
       0   0  1-c/m*Tc     0     ;
       0   0     0      1-c/m*Tc ];
Bd = [  0     0  ;
        0     0  ;
        Tc/m    0  ;
        0   Tc/m ];
Cy = [1 0 0 0;
      0 1 0 0];
%% Augmented model for position tracking
% δz(k) = z(k)-z(k-1)           - Incremental states
% ε(k) = [(x(k)-x*),(y(k)-y*)]' - Positions' errors
% ξ(k) = [δz'(k),ε'(k)]'        - Augmented states
Aa = [  Ad   zeros(4,2);
      Cy*Ad   eye(2)   ];
Ba = [  Bd  ;
      Cy*Bd ];
Ca = [ zeros(2,4)  eye(2) ];
%% LQ optimal control
% u(k) = -F*z(k) -f*Σ(ε(i)) i=0..k
Q = [0  0  0  0  0  0  ;
     0  0  0  0  0  0  ;
     0  0  0  0  0  0  ;
     0  0  0  0  0  0  ;
     0  0  0  0 100 0  ;
     0  0  0  0  0 100 ];
R = [ 0.1  0  ;
       0  0.1 ];
Fa = dlqr(Aa,Ba,Q,R);
F = Fa(:,1:end-2);
f = Fa(:,4+1:end);
%% Pre-controlled system
sys_int = ss(eye(2),eye(2),f*eye(2),zeros(2,2),Tc);
P_ol = series(sys_int,ss(Ad-Bd*F,Bd,Cy,0,Tc));
P = feedback(P_ol,eye(2),-1);
Phi = P.A;     G = P.B;    Hy = P.C;
% Only position constraints
%Hc = [ eye(2)  zeros(2,4) ];
%L = zeros(2,2);
% Speed and input constraints
%Hc = [ zeros(2,2) eye(2)  zeros(2,2) ;
%              -F              f      ];
%L = zeros(4,2);
% Position and input constraints
%Hc = [ eye(2)        zeros(2,4)      ;
%              -F              f      ];
%L = zeros(4,2);
% Position, speed and input constraints
Hc = [ eye(4)  zeros(4,2) ;
        -F         f      ];
L = zeros(6,2);
%% Extend position control gain to the nonlinear model
Fa = [Fa(:,1:2),[0,0]',Fa(:,3:4),[0,0]',Fa(:,5:6)];
Fa = [Fa; zeros(1,8)];

%% Actuator allocation matrix
alpha = pi/4;
l = 1; 
Sig = [  sin(alpha)  sin(alpha) -sin(alpha) -sin(alpha) ;
        -cos(alpha)  cos(alpha) -cos(alpha)  cos(alpha) ;
           -l/2         l/2         l/2        -l/2     ];

%% Model dimensions
nx = 6;
nu = 3;