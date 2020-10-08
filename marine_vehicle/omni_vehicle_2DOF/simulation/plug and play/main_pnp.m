%% Clear workspace
clear all;
close all;

%% Load vehicles' model matrices 

vehicle_2DOF_model_2

%% Vehicles
N = 4; % number of vehicles

% Vehicle 1
vehicle{1} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{1}.init_position(0.5,0);
% Vehicle 2
vehicle{2} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{2}.init_position(0,1);

vehicle{3} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{3}.init_position(0,-1);

vehicle{4} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
vehicle{4}.init_position(1,2.5);  %%% A 2.2 abbiamo dei problemi 
% end

%% Net configuration
%   1
%  / \
% 2   3
adj_matrix = [-1  1  1  0;
			   1 -1  0  0;
               1  0 -1  0;
               0  0  0 -1];

%% Vehicles constraints
% Vehicles swarm position constraints
% ||(x,y)_i-(x,y)_j||? ? d_max
% ||(x,y)_i-(x,y)_j||? ? d_min
d_max = 1.5; % maximum distance between vehicles - [m]
d_min = 0.3; % minimum distance between vehicles - [m]

% Vehicles input/speed constraints
Vx = 1; % max abs of speed along x - [m/s]
Vy = 1; % max abs of speed along y - [m/s]
T_max = 2; % max abs of motor thrust - [N]

%% Command Governor parameters
Psi = eye(2); % vehicle's references weight matrix
k0 = 10; % prediction horizon

%% Dynamic Command Governor
for i=1:N
    vehicle{i}.cg = DynamicDistribuitedCommandGovernor(i,Phi,G,Hc,L,Psi,k0, 'gurobi');
    vehicle{i}.cg.add_vehicle_cnstr('speed', [Vx, Vy]', 'thrust',T_max);
    for j=1:N
        if adj_matrix(i,j) == 1 % i,j is neighbour
            vehicle{i}.cg.add_swarm_cnstr(j,'proximity',d_max,'anticollision',d_min);
        end
    end
end


%% Color the net
colors = [0,1,2];
vehicle{1}.color = colors(1);
vehicle{2}.color = colors(2);
vehicle{3}.color = colors(2);
vehicle{4}.color = colors(3);

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

plot_color = ['b', 'g', 'k', 'r'];
plaggable = false;
alreadyplag = false;

%%%%%%%%%% Frontal Collision References
r{1} = vehicle{1}.ctrl_sys.sys.xi(1:2) + [2, 0]';
r{2} = vehicle{2}.ctrl_sys.sys.xi(1:2) + [2, 0]';
r{3} =  vehicle{3}.ctrl_sys.sys.xi(1:2) + [2, 0]';
% r{4} =  vehicle{4}.ctrl_sys.sys.xi(1:2) + [0, 2]';
r{4} = [1,-3]';

%%%%% Binary variables usefull to PnP operation
ask_to_freeze = [false, false, false, false]';
ask_to_plug = false;
pluggable = false;

%%%%% Virtual references 
r_stari = [];  
r_starn1 = [];

% %%%% Dynamic constraits management %%%%
% added13 = false;
% added23 = false;
% added12 = false;
% %%%%%%%%%%%

%%% Vector for distance between i and N+1 for plotting purposes

dist = [];


%%%%%% Sim cycle %%%%%%%%%
for t=1:NT
    for i=1:N
        if vehicle{i}.color == colors(round)
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
            
            %%%% Condition to plug verified by vehicle N+1
            if(colors(round)==vehicle{4}.color)
                if(norm(vehicle{2}.ctrl_sys.sys.xi(1:2)-vehicle{4}.ctrl_sys.sys.xi(1:2))<1 ...
                        && not(ask_to_plug) && isempty(find(vehicle{2}.cg.neigh==4, 1)))
                    ask_to_plug = true; 
                    disp('Richiesta di Plug settata');
                end
            end
            
            if(norm(vehicle{2}.ctrl_sys.sys.xi(1:2)-vehicle{4}.ctrl_sys.sys.xi(1:2)) < 0.01)
                error('Collided');
            end
            %%%% Pluggability condition verified by vehicle i
            if(ask_to_plug && i==2)
                pluggable = vehicle{2}.cg.check([vehicle{2}.ctrl_sys.sys.xi; vehicle{2}.ctrl_sys.xci], ...
                    [vehicle{4}.ctrl_sys.sys.xi; vehicle{4}.ctrl_sys.xci], vehicle{2}.g, vehicle{4}.g, d_max, d_min);
                disp('Funzione di check Pluggable');
            end
            
            %%%% Now that we have pluggability result we procede attaching
            %%%% the vehicles
            if(pluggable && i==2)
                disp('Mi posso pluggare');
                adj_matrix(4,2)=1;
                adj_matrix(2,4)=1;
                disp('communication added 2, 4');
                vehicle{2}.cg.add_swarm_cnstr(4,'proximity',d_max,'anticollision',d_min);
                vehicle{4}.cg.add_swarm_cnstr(2,'proximity',d_max,'anticollision',d_min);
                r{2} = [2, 1]';
                r{4} = [1,-3]';
                ask_to_plug = false;
                pluggable = false;
                r_stari = [];
                r_starn1 = [];
                ask_to_freeze(vehicle{i}.cg.neigh) = false;
                %%% Since adj_matrix is modified, we need to update state
                %%% variables with new neighboor
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
                %%% If they are not pluggable we procede finding a virtual
                %%% reference and freezing references for neig of
                
            elseif(ask_to_plug && i==2  && isempty(find(vehicle{2}.cg.neigh==4, 1)))
                disp('Non mi posso pluggare');
                %%% If virtual references were not computed then compute
                %%% (ONLY ONCE)
                if(isempty(r_stari) && isempty(r_starn1))
                    disp('Calcolo riferimenti virtuali');
                    [r_stari,r_starn1] = vehicle{i}.cg.compute_virtual_cmd(r{i},g_n, d_max, d_min);%, [vehicle{4}.ctrl_sys.sys.xi;vehicle{4}.ctrl_sys.xci]);
                end
                r{2} = r_stari;
                r{4} = r_starn1;
                
                ask_to_freeze(vehicle{i}.cg.neigh) = true;
            end
            
            %%% If the vehicles do not need to compute reference since they
            %%% are plugging 
            
            if((ask_to_freeze(i))) % || ... %% If I have to freeze my reference
%                     (i==2 && not(isempty(r_stari))) || ... %% If I have to track virtual reference
%                     (i==4 && not(isempty(r_starn1)))) %% If I have to track virtual reference 
                     g = vehicle{i}.g;
            else
                [g,s] = vehicle{i}.cg.compute_cmd(xa, r{i}, g_n);
            end
             
            if ~isempty(g)
                vehicle{i}.g = g;
                %%%% live plot %%%%
                plot(r{i}(1), r{i}(2), strcat(plot_color(i), 'o'));
                plot(g(1), g(2), strcat(plot_color(i), 'x'));
                %%%%%%%%%%%%

                %%%%% Data collection of optimization times %%%%%%
%                 cputime= [cputime,s.solvertime];
%                 yalmiptime=[yalmiptime,s.yalmiptime];
%                 %%%%%%%%%%%%%%
            else
                disp('WARN: old references');
                t,i
            end
        end
    end
    
    
    
    for i=1:N
        vehicle{i}.ctrl_sys.sim(vehicle{i}.g,Tc_cg);
    end
    round = rem(round,length(colors))+1;
    
    %%%%%%% live plot %%%%%%%
    for k=1:N
        % Trajectory
        figure(1);
        axis([0 5 -4 4]);
        hold on;
        plot(vehicle{k}.ctrl_sys.sys.x(1,:),vehicle{k}.ctrl_sys.sys.x(2,:), strcat(plot_color(k), '.'));
    end
    
    if(t==1)
        title('Frontal Collision Simulation');
        xlabel('x [m]');
        ylabel('y [m]');
    end
    dist = [dist, norm((vehicle{2}.ctrl_sys.sys.x(1:2,end)-vehicle{4}.ctrl_sys.sys.x(1:2,end)))];
    drawnow;
end


figure;
plot(1:NT, dist);
hold on;
plot(1:NT,d_min*ones(1,length(1:NT)));
title('Distance between vehicles i and N+1');
xlabel('time [s]');
ylabel('distance [m]');

% figure;
% subplot(3, 1, 1);
% plot(1:NT, dist12);
% hold on;
% plot(1:NT,d_min*ones(1,length(1:NT)));
% title('Distance between vehicles 1 2');
% xlabel('time [s]');
% ylabel('distance [m]');
% 
% %%%%%%%%%%%%
% subplot(3, 1, 2);
% plot(1:NT, dist13);
% hold on;
% plot(1:NT,d_min*ones(1,length(1:NT)));
% title('Distance between vehicles 1 3');
% xlabel('time [s]');
% ylabel('distance [m]');
% 
% %%%%%%%%%
% subplot(3, 1, 3);
% plot(1:NT, dist23);
% hold on;
% plot(1:NT,d_min*ones(1,length(1:NT)));
% title('Distance between vehicles 2 3');
% xlabel('time [s]');
% ylabel('distance [m]');
% 
