%% Clear workspace
clear all;
close all;

%% Load vehicles' model matrices 
addpath(genpath('../util'));  addpath(genpath('../tbxmanager'));    addpath('../CG');
vehicle_model

%% Vehicles
N = 3; % number of vehicles
%for i=1:N
	% Vehicle 1
	vehicle{1} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
    vehicle{1}.init_position(1,0);
	% Vehicle 2
	vehicle{2} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
    vehicle{2}.init_position(0,1);
	% Vehicle 3
	vehicle{3} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
    vehicle{3}.init_position(0,-1);
%end

Psi = eye(2); % vehicle's references weight matrix
k0 = 10; % prediction horizon
delta = 1e-5; % constraints tollerance

%% Net configuration
%   1
%  / \
% 2   3
adj_matrix = [-1  1  1;
			   1 -1  0;
			   1  0 -1];
           
% Vehicles swarm position constraints
% ||(x,y)||∞ ≤ d_max
% ||(x,y)||∞ ≥ d_min
d_max = 1.5; % maximum distance between vehicles
d_min = 1; % minimum distance between vehicles

for i=1:N
	% Vehicle i
    % Augmented neighbour matrix
    % | Φ(i)            |
    % |      Φ(j1)      |
    % |           Φ(jm) |
    Phi = vehicle{i}.ctrl_sys.Phi;
	G = vehicle{i}.ctrl_sys.G;
	Hc = vehicle{i}.ctrl_sys.Hc;
	L = vehicle{i}.ctrl_sys.L;
    
    for j=1:N
        if adj_matrix(i,j) == 1 % i,j is neighbour
			Phi = blkdiag(Phi,vehicle{j}.ctrl_sys.Phi);
			G = blkdiag(G,vehicle{j}.ctrl_sys.G);
			Hc = blkdiag(Hc,vehicle{j}.ctrl_sys.Hc);
			L = blkdiag(L,vehicle{j}.ctrl_sys.L);
        end
    end
    
    nc = size(vehicle{i}.ctrl_sys.Hc,1); % single vehicle c dimension
    nca = size(Hc,1); % vehicle i augmented-c dimension
    T = [];     gi = [];
    U = [];     hi = [];
    V = [];     qi = [];
    
    k = 0; % neighbour number
    for j=1:N
        if adj_matrix(i,j) == 1 % i,j is neighbour
            k = k+1;
            % Split ||.||∞
			cnstr = zeros(4,nca);
            % split modules x constraints
            cnstr(1,1) = 1; 
            cnstr(1,(k*nc)+1) = -1;
            cnstr(2,1) = -1; 
            cnstr(2,(k*nc)+1) = 1;
            % split modules y constraints
            cnstr(3,2) = 1; 
            cnstr(3,(k*nc)+2) = -1;
            cnstr(4,2) = -1; 
            cnstr(4,(k*nc)+2) = 1;
            
            % Matrix for neighbour remoteness constraints
            % U*c ≤ hi
            if ~isempty(U)
                U = [U;cnstr];
                hi = [hi;[d_max,d_max,d_max,d_max]'];
            else
                U = cnstr;
                hi = [d_max,d_max,d_max,d_max]';
            end
            
            % Matrix for neighbour proximity constraints
            % V*c ≤ qi
            if ~isempty(V)
                V = [V;cnstr];
                qi = [qi;[-d_min,-d_min,-d_min,-d_min]'];
            else
                V = cnstr;
                qi = [-d_min,-d_min,-d_min,-d_min]';
            end    
        end
    end
    % Speed and thrust constraints
    % Need change Hc,L matrix in model_vehicle
    % T*c ≤ b
    Vx = 2; % max abs of speed along x - [m/s]
    Vy = 2; % max abs of speed along y - [m/s]
    Tm = 100; % max abs of motor thrust - [N]

    T = [ 0  0  1  0  0  0;% zeros(1,k*nc);
          0  0 -1  0  0  0;% zeros(1,k*nc);
          0  0  0  1  0  0;% zeros(1,k*nc);
          0  0  0 -1  0  0;% zeros(1,k*nc);
          0  0  0  0  1  0;% zeros(1,k*nc);
          0  0  0  0 -1  0;% zeros(1,k*nc);
          0  0  0  0  0  1;% zeros(1,k*nc);
          0  0  0  0  0 -1];% zeros(1,k*nc)];

    gi = [Vx,Vx,Vy,Vy,Tm,Tm,Tm,Tm]';
    
    Ta = T;
    gia = gi;
    for j=1:k
        Ta = blkdiag(Ta,T);
        gia = [gia;gi];
    end
    Ta = [Ta;U];    gia = [gia;hi];

    vehicle{i}.cg = DistribuitedCommandGovernor(Phi,G,Hc,L,Ta,gia,U,hi,V,qi,Psi,k0,delta,false);
end


%% Simulation Parallel CG
Tf = 5; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % Recalculation references time
r{1} = [4,0.5]'; % position references
r{2} = [3,1]'; % position references
r{3} = [3,-1.5]'; % position references
NT = ceil(Tf/Tc_cg); % simulation steps number

for t=1:NT
    for i=1:N
        x = vehicle{i}.ctrl_sys.sys.xi; % vehicle current state
        xc = vehicle{i}.ctrl_sys.xci; % controller current state
        xa = [x;xc];
        g_n = [];
        for j=1:N
            if adj_matrix(i,j) == 1 % i,j is neighbour
                g_n = [g_n;vehicle{j}.g];
                x = vehicle{j}.ctrl_sys.sys.xi; % vehicle current state
                xc = vehicle{j}.ctrl_sys.xci; % controller current state
                xa = [xa;x;xc];
            end
        end
        
        g = vehicle{i}.cg.compute_cmd(xa,r{i},g_n);
        vehicle{i}.g = g;
    end
    for i=1:N
        vehicle{i}.ctrl_sys.sim(vehicle{i}.g,Tc_cg);
    end
end


%% Plot Vehicles trajectory and velocities
for i=1:N
    % Trajectory
    figure(1);  hold on;
    plot(vehicle{i}.ctrl_sys.sys.x(1,:),vehicle{i}.ctrl_sys.sys.x(2,:),'.');
    plot(vehicle{i}.ctrl_sys.sys.x(1,end),vehicle{i}.ctrl_sys.sys.x(2,end),'o');
    % Position
    figure(2); hold on;
    subplot(6,1,(i-1)*2+1);  plot(vehicle{i}.ctrl_sys.sys.t,vehicle{i}.ctrl_sys.sys.x(1,:));
    subplot(6,1,(i-1)*2+2);  plot(vehicle{i}.ctrl_sys.sys.t,vehicle{i}.ctrl_sys.sys.x(2,:));
    % Velocities
    figure(3); hold on;
    subplot(6,1,(i-1)*2+1);  plot(vehicle{i}.ctrl_sys.sys.t,vehicle{i}.ctrl_sys.sys.x(3,:));
    subplot(6,1,(i-1)*2+2);  plot(vehicle{i}.ctrl_sys.sys.t,vehicle{i}.ctrl_sys.sys.x(4,:));
end
