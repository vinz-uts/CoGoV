%% Clear workspace
clear all;
close all;

%% Load vehicles' model matrices 
% addpath('../../marine_vehicle');        addpath(genpath('../../util'));
% addpath(genpath('../../tbxmanager'));   
% addpath('../../CG');

vehicle_2DOF_model_2

%% Vehicles
N = 3; % number of vehicles

% Vehicle 1
vehicle{1} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{1}.init_position(1,0);
% Vehicle 2
vehicle{2} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{2}.init_position(0,1);

vehicle{3} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{3}.init_position(1.5,1.5);
% end

%% Net configuration
%   1
%  / \
% 2   3
adj_matrix = [-1  0 0;
			   0 -1 0;
               0  0 -1];

%% Vehicles constraints
% Vehicles swarm position constraints
% ||(x,y)_i-(x,y)_j||∞ ≤ d_max
% ||(x,y)_i-(x,y)_j||∞ ≥ d_min
d_max = 200; % maximum distance between vehicles - [m]
d_min = 0.1; % minimum distance between vehicles - [m]

% Vehicles input/speed constraints
Vx = 200; % max abs of speed along x - [m/s]
Vy = 200; % max abs of speed along y - [m/s]
T_max = 20; % max abs of motor thrust - [N]

%% Command Governor parameters
Psi = 0.01*eye(2); % vehicle's references weight matrix
k0 = 10; % prediction horizon

%% Dynamic Command Governor
for i=1:N
    vehicle{i}.cg = DynamicDistribuitedCommandGovernor(i,Phi,G,Hc,L,Psi,k0, 'gurobi');
    vehicle{i}.cg.add_vehicle_cnstr('thrust',T_max);
    for j=1:N
%         if adj_matrix(i,j) == 1 % i,j is neighbour
%             vehicle{i}.cg.add_swarm_cnstr(vehicle{i}.ctrl_sys, j,'proximity',d_max,'anticollision',d_min);
%         end
    end
end

%% Color the net
colors = [0,1,2];
vehicle{1}.color = colors(1);
vehicle{2}.color = colors(2);
vehicle{3}.color = colors(3);

%% Simulation Colored Round CG
Tf = 10; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time
NT = ceil(Tf/Tc_cg); % simulation steps number
round = 1;

%%%%% Data collection about optimization time %%%%%%%%
% names are self-explanatory
cputime =[];
yalmiptime = [];

dist12 = []; %%% check for collition constraints
dist13 = []; %%% check for collition constraints
dist23 = []; %%% check for collition constraints



%%%%%%%%%% Frontal Collision References
r{1} = vehicle{2}.ctrl_sys.sys.xi(1:2);
r{2} = vehicle{1}.ctrl_sys.sys.xi(1:2);
r{3} = [-1, -1]';

%%%% Dynamic constraits management %%%%
added13 = false;
added23 = false;
added12 = false;
%%%%%%%%%%%

%%%%%% Sim cycle %%%%%%%%%
for t=1:NT
    for i=1:N
        if vehicle{i}.color == colors(round)
            x = vehicle{i}.ctrl_sys.sys.xi; % vehicle current state
            xc = vehicle{i}.ctrl_sys.xci; % controller current state
            xa = [x;xc];
            
            %%%%%% Dynamic constraints management %%%%%%%%%
            if norm(vehicle{2}.ctrl_sys.sys.xi(1:2) - vehicle{1}.ctrl_sys.sys.xi(1:2)) <= 3 && not(added12) && false
                adj_matrix(1:2, 1:2) = [-1  1;
                                         1 -1];
                added12 = true;
                disp('communication added 1, 2');
                vehicle{1}.cg.add_swarm_cnstr(2,'proximity',d_max,'anticollision',d_min);
                vehicle{2}.cg.add_swarm_cnstr(1,'proximity',d_max,'anticollision',d_min);
            end
%             if norm(vehicle{2}.ctrl_sys.sys.xi(1:2) - vehicle{3}.ctrl_sys.sys.xi(1:2)) <= 1 && not(added23)
%                 adj_matrix(2:3, 2:3) = [-1  1;
%                                          1 -1];
%                 added23 = true;
%                 disp('communication added 2,3');
%                 vehicle{2}.cg.add_swarm_cnstr(3,'proximity',d_max,'anticollision',d_min);
%                 vehicle{3}.cg.add_swarm_cnstr(2,'proximity',d_max,'anticollision',d_min);
%                 vehicle{2}.cg.check([vehicle{2}.ctrl_sys.sys.xi; vehicle{2}.ctrl_sys.xci], vehicle{2}.g, [vehicle{3}.ctrl_sys.sys.xi; vehicle{3}.ctrl_sys.xci], vehicle{3}.g)
%             end
%             if norm(vehicle{1}.ctrl_sys.sys.xi(1:2) - vehicle{3}.ctrl_sys.sys.xi(1:2)) <= 1 && not(added13)
%                 adj_matrix(1, 3) = 1;
%                 adj_matrix(3, 1) = 1;
%                 added13 = true;
%                 disp('communication added 1,3');
%                 vehicle{1}.cg.add_swarm_cnstr(3,'proximity',d_max,'anticollision',d_min);
%                 vehicle{3}.cg.add_swarm_cnstr(1,'proximity',d_max,'anticollision',d_min);
%                 vehicle{1}.cg.check([vehicle{1}.ctrl_sys.sys.xi; vehicle{1}.ctrl_sys.xci], vehicle{1}.g, [vehicle{3}.ctrl_sys.sys.xi; vehicle{3}.ctrl_sys.xci], vehicle{3}.g)
%             end
            %%%%%%%%%%%%%%%
            
            g_n = [];
            for j=1:N
                if adj_matrix(i,j) == 1 % i,j is neighbour
                    g_n = [g_n;vehicle{j}.g];
                    x = vehicle{j}.ctrl_sys.sys.xi; % vehicle current state
                    xc = vehicle{j}.ctrl_sys.xci; % controller current state
                    xa = [xa;x;xc];
                end
            end
            
            [g,s] = vehicle{i}.cg.compute_cmd(xa, r{i}, g_n);                 
                      
            
            if ~isempty(g)
                vehicle{i}.g = g;
                
                %%%% live plot %%%%
                if(i==1)
                    plot(r{1}(1), r{1}(2), 'bo');
                    plot(g(1), g(2), 'bx');
                elseif(i==2)
                    plot(r{2}(1), r{2}(2), 'go');
                    plot(g(1), g(2), 'gx');
                else
                    plot(r{3}(1), r{3}(2), 'ko');
                    plot(g(1), g(2), 'kx');
                end
                %%%%%%%%%%%%
                
                %%%%% Data collection (optimization) %%%%%%
                cputime = [cputime,s.solvertime];
                yalmiptime = [yalmiptime,s.yalmiptime];
                %%%%%%%%%%%%%%
            else
                disp('WARN: old references');
                t,i
            end
            
             plaggable = vehicle{1}.cg.check([vehicle{1}.ctrl_sys.sys.xi; vehicle{1}.ctrl_sys.xci], [vehicle{2}.ctrl_sys.sys.xi; vehicle{2}.ctrl_sys.xci], vehicle{1}.g, vehicle{2}.g);
             if(plaggable)
                 disp('ok');
             else
                 disp('no');
             end
        end
    end
    
    
    for i=1:N
        vehicle{i}.ctrl_sys.sim(vehicle{i}.g,Tc_cg);
    end
    round = rem(round,length(colors))+1;
    
    %%%%%%% live plot %%%%%%%
    for k=1:N
        figure(1);
        hold on;
        
        if(k==1)
            plot(vehicle{1}.ctrl_sys.sys.x(1,:),vehicle{1}.ctrl_sys.sys.x(2,:),'b.');
        elseif(k == 2)
            plot(vehicle{2}.ctrl_sys.sys.x(1,:),vehicle{2}.ctrl_sys.sys.x(2,:),'g.');
        else
             plot(vehicle{3}.ctrl_sys.sys.x(1,:),vehicle{3}.ctrl_sys.sys.x(2,:),'k.');
        end
        
    end
    
    if(t==1)
        title('Frontal Collision Simulation');
        xlabel('x [m]');
        ylabel('y [m]');
    end
    
    drawnow;
    %%%%%%%%%%%%%
    
    
    dist12 = [dist12, norm((vehicle{1}.ctrl_sys.sys.x(1:2,end)-vehicle{2}.ctrl_sys.sys.x(1:2,end)))];
    dist13 = [dist13, norm((vehicle{1}.ctrl_sys.sys.x(1:2,end)-vehicle{3}.ctrl_sys.sys.x(1:2,end)))];
    dist23 = [dist23, norm((vehicle{2}.ctrl_sys.sys.x(1:2,end)-vehicle{3}.ctrl_sys.sys.x(1:2,end)))];
end


figure;
subplot(3, 1, 1);
plot(1:NT, dist12);
hold on;
plot(1:NT,d_min*ones(1,length(1:NT)));
title('Distance between vehicles 1 2');
xlabel('time [s]');
ylabel('distance [m]');

%%%%%%%%%%%%
subplot(3, 1, 2);
plot(1:NT, dist13);
hold on;
plot(1:NT,d_min*ones(1,length(1:NT)));
title('Distance between vehicles 1 3');
xlabel('time [s]');
ylabel('distance [m]');

%%%%%%%%%
subplot(3, 1, 3);
plot(1:NT, dist23);
hold on;
plot(1:NT,d_min*ones(1,length(1:NT)));
title('Distance between vehicles 2 3');
xlabel('time [s]');
ylabel('distance [m]');

