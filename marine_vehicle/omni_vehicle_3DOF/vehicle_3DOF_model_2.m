%% Model for marine vehicle %%
% Dynamic model for a omnidirectional vehicle on a plane.
% 
% States:                   Inputs:
% z = [x,y,ϑ,dx,dy,dϑ]'          u = [Tx,Ty,Tϑ]' 
% 
% Dynamic Model:
%                            dx = dx
% m*ddx +  c*dx = Tx  -->    dy = dy
% m*ddy +  c*dy = Ty  -->    dϑ = dϑ
% J*ddϑ + ct*dϑ = Tϑ  -->    ddx =  -c/m*dx + 1/m*Tx
%                            ddy =  -c/m*dy + 1/m*Ty
%                            ddϑ = -ct/J*dϑ + 1/J*Tϑ

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
    
Cy = [ 1 0 0 0 0 0 ;
       0 1 0 0 0 0 ;
       0 0 1 0 0 0 ];
   
%% Augmented model for position tracking
% δz(k) = z(k)-z(k-1)           - Incremental states
% ε(k) = [(x(k)-x*),(y(k)-y*),(ϑ(k)-ϑ*)]' - Positions' errors
% ξ(k) = [δz'(k),ε'(k)]'        - Augmented states

%% LQ optimal control
% u(k) = -F*z(k) -f*Σ(ε(i)) i=0..k
Aaug=[A     zeros(6, 3);
     -Cy    zeros(3, 3)];
Baug=[B;zeros(3,3)];
Caug=[Cy,zeros(3,3)];
% 
Fa = compute_Rstab_gain(ss(Aaug,Baug,Caug,zeros(3,3)),1, pi/10,1.5, 0, 'sedumi');
%Fa = dlqr(Aa,Ba,Q,R);
F = Fa(:,1:end-3);
f = Fa(:,6+1:end);


%% Pre-controlled system


Acl=(Aaug-Baug*Fa);
Bcl=[zeros(6, 3) ; eye(3)];
AUVCL=ss(Acl,Bcl,Caug,zeros(3,3));


P = c2d(AUVCL,Tc);

%%%%%% for code compatibility
f = -f;
Fa=[F,f];


%%%%%%

Phi = P.A;     G = P.B;    Hy = P.C;


% Only position constraints
%Hc = [ eye(3)  zeros(3,6) ];
%L = zeros(3,3);
% Speed and input constraints
%Hc = [ zeros(3,3) eye(3)  zeros(3,3) ;
%              -F              f      ];
%L = zeros(6,3);
% Position and input constraints
%Hc = [ eye(3)        zeros(3,6)      ;
%              -F              f      ];
%L = zeros(6,3);
% Position, speed and input constraints
Hc = [ eye(6)  zeros(6,3) ;
        -F         f      ];
L = zeros(9,3);
