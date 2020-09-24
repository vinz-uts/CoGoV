function [aris,bris] = find_hyperplane(ve1,ve2,ve3,ve4,y)
% Function useful to find hyperplane to separate obstacle and vehicles
% position
a = sdpvar(2,1);
b = sdpvar(1,1); 
r = sdpvar(1,1);
rveh=0.5;

V1 = a'*ve1 >= b -r +rveh;
V2 = a'*ve2 >= b -r +rveh;
V3 = a'*ve3 >= b -r +rveh;
V4 = a'*ve4 >= b -r +rveh ;
V5 = a'*y <= b +r ; 
V6 = norm(a,2) <= 1;
% V6a = [1 a'; a eye(2)] >= 0 ; %LMIlab or Sedumi version 

V= V1+V2+V3+V4+V5+V6;

vx=ve1;

if(norm(y-ve2)< norm(y-ve1))
    vx=ve2;
end
if(norm(y-ve3)< norm(y-vx))
    vx=ve3;
end
if(norm(y-ve4)< norm(y-vx))
    vx=ve4;
end


opt = sdpsettings('verbose',0,'solver','gurobi');

ris = optimize(V,norm(r,2)+norm(a'*vx-b),opt); %+norm(a'*vx-b)

% ris.solvertime 

aris = double(a);
bris = double(b); 

if (norm(aris)==0 && bris==0)
    warning('Hyperplane Not Found');
end

% hy = hyperplane(aris,bris);
% plot(hy);
end

