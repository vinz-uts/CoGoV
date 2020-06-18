function dz = nonlinear_vehicle_fun(z,u)
    %% NONLINEAR_VEHICLE_FUN - nonlinear model of a vehicle
    %  Nonlinear model of a marine omnidiretional vehicle
    %  
    %  States:                   Inputs:
    %  z = [x,y,ϑ,u,v,r]'        u = [Tx,Ty,Tϑ]'
    %
    %       |Tu|   | cos(ϑ) sin(ϑ) 0| |Tx|
    %  u' = |Tv| = |-sin(ϑ) cos(ϑ) 0|*|Ty|
    %       |Tr|   |   0      0    1| |Tϑ|
    % 
    %  Dynamic Model:
    %  |dx|   |0  0  0  cos(ϑ) -sin(ϑ)  0  | |x|   | 0   0   0 | 
    %  |dy|   |0  0  0  sin(ϑ)  cos(ϑ)  0  | |y|   | 0   0   0 | |Tu|
    %  |dϑ| = |0  0  0    0       0     1  |*|ϑ| + | 0   0   0 |*|Tv|
    %  |du|   |0  0  0  -m/c      0     0  | |u|   |1/m  0   0 | |Tr|
    %  |dv|   |0  0  0    0     -m/c    0  | |v|   | 0  1/m  0 |
    %  |dr|   |0  0  0    0       0   -J/c | |r|   | 0   0  1/J|
    
    %% Model parameters
    m = 10; % mass - [Kg]
    J = 3;  % inertia - [Kg m²]
    c = 1;  % friction coefficient - [-]
    % It's possible to modify c as c(u,v,r)
    
    R = [ cos(z(3)) sin(z(3)) 0 ;
         -sin(z(3)) cos(z(3)) 0 ;
             0         0      1 ];
    u = R*u;
    
    dz(1) = cos(z(3))*z(4) - sin(z(3))*z(5);
    dz(2) = sin(z(3))*z(4) + cos(z(3))*z(5);
    dz(3) = z(6);
    dz(4) = (-m/c)*z(4) + (1/m)*u(1);
    dz(5) = (-m/c)*z(5) + (1/m)*u(2);
    dz(6) = (-J/c)*z(6) + (1/J)*u(3);
end

