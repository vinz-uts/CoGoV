%% Clear workspace
clear all;  close all;

%% Load vehicles' model matrices 
addpath('../../../../marine_vehicle');      addpath('../../../../marine_vehicle/omni_vehicle_2DOF');
addpath(genpath('../../../../util'));       addpath('../../../../CG');
addpath(genpath('../../../../tbxmanager'));

vehicle_2DOF_model_2

%% Init vehicles
N = 5; % number of vehicles
for i=1:N
    vehicle{i} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
    vehicle{i}.init_position(10*sin(2*pi/N*i),10*cos(2*pi/N*i));
end

%% Communication constraints 
R   = 5; % maximum distance of communications - [m]
R__ = 3; % maximum distance of connectivity - [m]
R_  = 2; % minimum distance for cooperation - [m]

%% Vehicles constraints
% Vehicles swarm position constraints
% ||(x,y)_i-(x,y)_j||∞ ≤ d_max
% ||(x,y)_i-(x,y)_j||∞ ≥ d_min
d_max = R__;  % maximum distance between vehicles - [m]
d_min = R_/3; % minimum distance between vehicles - [m]

% Vehicles input/speed constraints
Vx = 0.5; % max abs of speed along x - [m/s]
Vy = 0.5; % max abs of speed along y - [m/s]
T_max = 50; % max abs of motor thrust - [N]

%% Command Governor parameters
Psi = 1000*eye(2); % vehicle's references weight matrix
k0 = 15; % prediction horizon

%% Dynamic Command Governor
for i=1:N
    vehicle{i}.cg = DynamicDistribuitedCommandGovernor(i,Phi,G,Hc,L,Psi,k0);
    vehicle{i}.cg.add_vehicle_cnstr('speed',[Vx,Vy],'thrust',T_max);
end

%% Color the net
colors = 1:N;
for i=1:N
    vehicle{i}.color = colors(i);
end

%% Simulation Colored Round CG
Tf = 20; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time
NT = ceil(Tf/Tc_cg); % simulation steps number

% References
for i=1:N
    r{i} = [10*sin(2*pi/N*i+2*2*pi/N) , 10*cos(2*pi/N*i+2*2*pi/N)]';
endif plugable_status == 'virtual_cmd'

round = 1;
for t=1:NT 
    for i=1:N
        if vehicle{i}.color == colors(round)        
            % Receive msg from v{j} : ||v{i}-v{j}||∞ ≤ R
            for j=1:N
                d_ij = norm(vehicle{i}.ctrl_sys.sys.xi(1:2)-vehicle{i}.ctrl_sys.sys.xi(1:2));
                if i~=j && d_ij <= R
                    if d_ij > R__
                        % Remove constraints and send plug-out message to v{j}
                        vehicle{i}.cg.remove_swarm_cnstr(j);
                        vehicle{j}.cg.remove_swarm_cnstr(i);
                    end
                    if d_ij <= R_ && vehicle{i}.pending_plugin ~= -1 % ||v{i}-v{j}||∞ ≤ R && not already pending plug-in request
                        vehicle{i}.pending_plugin = j;
                        % Send plug-in request message to v{j}
                        pluggable_status = vehicle{j}.cg.check(vehicle{j}.ctrl_sys.sys.xi(1:2),vehicle{i}.ctrl_sys.sys.xi(1:2),vehicle{j}.g,vehicle{i}.g,d_min,d_max);
                        if pluggable_status == 'success'
                            vehicle{j}.pending_plugin = -1;
                            vehicle{j}.add_swarm_cnstr(i,'anticollision',d_min); % Setted from v{j} after check
                            vehicle{i}.add_swarm_cnstr(j,'anticollision',d_min);
                            % Send unfreeze request to v{j} neighbours
                            for k = vehicle{j}.cg.neigh
                                vehicle{k}.freeze = 0;
                            end
                        else
                            [g_j,g_i] = vehicle{j}.cg.compute_virtual_cmd(r{j},r{i},vehicle{i}.g,dmax,dmin);
                            if isempty(g_i)
                                pluggable_status = 'failed'; % sended from v{j}
                                vehicle{j}.pending_plugin = -1;
                            else
                                pluggable_status = 'virtual_cmd';
                                vehicle{j}.pending_plugin = i;
                                vehicle{j}.g = g_j; % Setted from v{j} after computation
                                vehicle{i}.g = g_i;
                                % Send freeze request to v{j} neighbours
                                for k = vehicle{j}.cg.neigh
                                    vehicle{k}.freeze = 1;
                                end
                            end
                        end
                    end
                end
            end
            if vehicle{i}.pending_plugin ~= -1 % Already pending plug-in request
                pluggable_status = vehicle{vehicle{i}.pending_plugin}.cg.check(vehicle{vehicle{i}.pending_plugin}.ctrl_sys.sys.xi(1:2),vehicle{i}.ctrl_sys.sys.xi(1:2),vehicle{vehicle{i}.pending_plugin}.g,vehicle{i}.g,d_min,d_max);
                if pluggable_status == 'success'
                    vehicle{j}.pending_plugin = -1;
                    vehicle{j}.add_swarm_cnstr(i,'anticollision',d_min); % Setted from v{j} after check
                    vehicle{i}.add_swarm_cnstr(j,'anticollision',d_min);
                    % Send unfreeze request to v{j} neighbours
                    for k = vehicle{j}.cg.neigh
                        vehicle{k}.freeze = 0;
                    end
                end
            elseif vehicle{i}.freeze == 0
                x = vehicle{i}.ctrl_sys.sys.xi; % vehicle current state
                xc = vehicle{i}.ctrl_sys.xci; % controller current state
                xa = [x;xc];
                g_n = [];
                for j = vehicle{i}.cg.neigh
                    g_n = [g_n;vehicle{j}.g];
                    x = vehicle{j}.ctrl_sys.sys.xi; % vehicle{j} current state
                    xc = vehicle{j}.ctrl_sys.xci; % controller{j} current state
                    xa = [xa;x;xc];
                end
                [g,s] = vehicle{i}.cg.compute_cmd(xa,r{i},g_n);
                if ~isempty(g)
                    vehicle{i}.g = g;
                    cputime= [cputime,s.solvertime];
                    yalmiptime=[yalmiptime,s.yalmiptime];
                else
                    disp('WARN: old references');
                    t,i
                end
            end
        end
    end
    % Simulate for Tc_cg
    for i=1:N
        vehicle{i}.ctrl_sys.sim(r{i},Tc_cg);
    end
    round = rem(round,length(colors))+1;
end