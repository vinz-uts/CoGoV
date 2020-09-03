function [Ka, yalmipdiagnostics] = compute_Rstab_gain(sys, alpha, theta, r, q, solver_name)
% Function used to calculate a feedback gain so that the poles of the feedback
% system pertain to a region (known as the R-region) defined by the intersection
% of:
% 1) a half plane Re{z} < -alpha where z is a complex number
% 2) a circle with radius r and centered in (q, 0)
% 3) conical sector with aperture theta.

% input:
% sys           - model expressed as a statespace system
% alpha         - parameter associated with the constraint on the half-space
% theta         - conical sector aperture
% r             - radius of the circle
% q             - x-coordinate of the circle center 
% solver_name   - optional input used to specify the solver (default lmilab)
% output:
% Ka                    - Region-stability gain
% yalmipdiagnostics     - yalmip diagnostic structure (debug feature)

% possible R-region parameters
% alpha = 0.3;
% theta = pi/3;
% r = 1;
% q = 0;

if(nargin < 6)
    solver_name = 'lmilab';
end

% xp = Ax + B1u
A = sys.A;
B1 = sys.B;

% compute system dimensions
nx = length(A);
nu = length(B1(1,:));

% define sdp variables
X = sdpvar(nx);
Y = sdpvar(nu, nx);

% constraint associated to the semi-plane defined by Re{z} <= -alpha
V1 = 2*alpha*X + (A*X+B1*Y) + (A*X+B1*Y)' <= 0;

% constraint associated to the circle of radius r
V2 = [-r*X            -q*X+(A*X+B1*Y);
      -q*X+(A*X+B1*Y)' (-r*X)        ] <= 0;

% constraint associated to the conical sector with theta aperture
V3 = [sin(theta)*((A*X+B1*Y)+(A*X+B1*Y)') cos(theta)*((A*X+B1*Y)-(A*X+B1*Y)');
      cos(theta)*((A*X+B1*Y)'-(A*X+B1*Y)) sin(theta)*((A*X+B1*Y)+(A*X+B1*Y)')] <= 0;

% X must be semipositive-definite
V4 = X >= 0;

V =  V1 + V2 + V3  + V4;

opts = sdpsettings;
opts.solver = solver_name;
opts.verbose = 0;

% we only look for a feasible solution so we can omit the objective function
yalmipdiagnostics = optimize(V, [], opts);

if(not(yalmipdiagnostics.problem == 0))
    error('R-stability gain cannot be computed.');
end

Ka = double(Y)/double(X);
Ka = -Ka;

end

