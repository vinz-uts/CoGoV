%% Model for marine vehicle %%
% Dynamic model for a omnidirectional vehicle on a plane without
% orientation.
% 
% States:                   Inputs:
% z = [x,y,dx,dy]'          u = [Tx,Ty]' 
% 
% Dynamic Model:
%                           dx = dx
% m*ddx + c*dx = Tx  -->    dy = dy
% m*ddy + c*dy = Ty  -->    ddx = -c/m*dx + 1/m*Tx
%                           ddy = -c/m*dy + 1/m*Ty
%

%% Model parameters
m = 10; % mass - [Kg]
c = 1;  % friction coefficient - [-] 

%% Time continuous model
A = [ 0    0    1    0  ;
      0    0    0    1  ;
      0    0  -c/m   0  ;
      0    0    0  -c/m ];
 
B = [ 0   0  ;
      0   0  ;
     1/m  0  ;
      0  1/m ];
  
Cy = [1 0 0 0;
      0 1 0 0];

%% Sampling time [s]
Tc = 0.1; % sampling time - [s]

%% Augmented model for position tracking
% δz(k) = z(k)-z(k-1)           - Incremental states
% ε(k) = [(x(k)-x*),(y(k)-y*)]' - Positions' errors
% ξ(k) = [δz'(k),ε'(k)]'        - Augmented states

zer=[0;0;0;0];
Aaug=[A zer zer;-Cy [0;0] [0;0]];
Baug=[B;zeros(2,2)];
Caug=[Cy,zeros(2,2)];

%% Compute control law with R-stability design 

Fa = compute_Rstab_gain(ss(Aaug,Baug,Caug,zeros(2,2)),1, pi/10,1.5, 0, 'sedumi');

% Extracting feedback gain and feedforward gain from previous output 
F = Fa(:,1:end-2);
f = Fa(:,4+1:end);


%% Pre-controlled system
Acl=(Aaug-Baug*Fa);
Bcl=[[zer;1;0],[zer;0;1]];
AUVCL=ss(Acl,Bcl,Caug,zeros(2,2));

%% Discrete time model 
% Discrete model can be extracted using different methods
P = c2d(AUVCL,Tc);

%%%%%% For code compatibility keep this sign change
f = -f;
Fa=[F,f];

Phi = P.A;     G = P.B;    Hy = P.C;

% Position, speed and input constraints
Hc = [ eye(4)  zeros(4,2) ;
        -F         f      ];
L = zeros(6,2);
