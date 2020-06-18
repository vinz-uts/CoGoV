%% Model parameters
m = 10; % mass - [Kg]
J = 3;  % inertia coefficient - [Kg m²]
c = 1;  % friction coefficient - [-]
ct = 1; % rotational friction coefficient - [-]

%% Model dimensions
nx = 6;
nu = 3;

%% Time continuous model
A = [ 0    0    0    1    0    0   ;
      0    0    0    0    1    0   ;
      0    0    0    0    0    1   ;
      0    0    0  -c/m   0    0   ;
      0    0    0    0  -c/m   0   ;
      0    0    0    0    0  -ct/J ];
  
B = [ 0   0   0  ;
      0   0   0  ;
      0   0   0  ;
     1/m  0   0  ;
      0  1/m  0  ;
      0   0  1/J ];
  
%% Time discrete model
% Using Forward Euler approximation: dz*Tc = z(k+1) - z(k)
Tc = 0.1; % sampling time - [s]

Ad = [ 1   0   0     Tc        0        0     ;
       0   1   0     0         Tc       0     ;
       0   0   1     0         0        Tc    ; 
       0   0   0  1-c/m*Tc     0        0     ;
       0   0   0     0     1-c/m*Tc     0     ;
       0   0   0     0         0    1-ct/J*Tc ];
   
Bd = [  0     0     0   ;
        0     0     0   ;
        0     0     0   ;
      Tc/m    0     0   ;
        0   Tc/m    0   ;
        0     0    Tc/J ];
    
Cy = [ 1 0 0 0 0 0 ;
       0 1 0 0 0 0 ;
       0 0 1 0 0 0 ];
   
%% Augmented model for position tracking
% δz(k) = z(k)-z(k-1)           - Incremental states
% ε(k) = [(x(k)-x*),(y(k)-y*),(ϑ(k)-ϑ*)]' - Positions' errors
% ξ(k) = [δz'(k),ε'(k)]'        - Augmented states
Aa = [  Ad   zeros(6,3);
      Cy*Ad   eye(3)   ];
Ba = [  Bd  ;
      Cy*Bd ];
Ca = [ zeros(3,6)  eye(3) ];

%% LQ optimal control
% u(k) = -F*z(k) -f*Σ(ε(i)) i=0..k
Q = [0  0  0  0  0  0  0  0  0  ;
     0  0  0  0  0  0  0  0  0  ;
     0  0  0  0  0  0  0  0  0  ;
     0  0  0  0  0  0  0  0  0  ;
     0  0  0  0  0  0  0  0  0  ;
     0  0  0  0  0  0  0  0  0  ;
     0  0  0  0  0  0 100 0  0  ;
     0  0  0  0  0  0  0 100 0  ;
     0  0  0  0  0  0  0  0 300 ];
 
R = [ 0.1  0   0  ;
       0  0.1  0  ;
       0   0  0.1 ];
   
Fa = dlqr(Aa,Ba,Q,R);
F = Fa(:,1:end-3);
f = Fa(:,6+1:end);

%% Pre-controlled system
sys_int = ss(eye(3),eye(3),f*eye(3),zeros(3,3),Tc);
P_ol = series(sys_int,ss(Ad-Bd*F,Bd,Cy,0,Tc));
P = feedback(P_ol,eye(3),-1);
Phi = P.A;     G = P.B;    Hy = P.C;

% Only position constraints
%Hc = [ eye(3)  zeros(3,6) ];
%L = zeros(3,3);
% Speed and input constraints
Hc = [ zeros(3,3) eye(3)  zeros(3,3) ;
              -F              f      ];
L = zeros(6,3);
% Position and input constraints
%Hc = [ eye(3)        zeros(3,6)      ;
%              -F              f      ];
%L = zeros(6,3);
% Position, speed and input constraints
%Hc = [ eye(6)  zeros(6,3) ;
%        -F         f      ];
%L = zeros(9,3);

%% Extend position control gain to the nonlinear model
%Fa = [Fa(:,1:2),[0,0]',Fa(:,3:4),[0,0]',Fa(:,5:6)];
%Fa = [Fa; zeros(1,8)];

%% Actuator allocation matrix
%alpha = pi/4;
%l = 1; 
%Sig = [  sin(alpha)  sin(alpha) -sin(alpha) -sin(alpha) ;
%        -cos(alpha)  cos(alpha) -cos(alpha)  cos(alpha) ;
%           -l/2         l/2         l/2        -l/2     ];
