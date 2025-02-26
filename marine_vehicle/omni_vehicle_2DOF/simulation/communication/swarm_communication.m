%% Clear workspace
clear;  close all;

%% Load vehicles' model matrices
addpath('../../../../marine_vehicle');      addpath('../../../../marine_vehicle/omni_vehicle_2DOF');
addpath(genpath('../../../../util'));       addpath('../../../../CG');
addpath(genpath('../../../../tbxmanager'));

vehicle_2DOF_model_2

%% Init vehicles
N = 5; % number of vehicles
vehicle = cell(1, N);
for i=1:N
    vehicle{i} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
    vehicle{i}.init_position(10*sin(2*pi/N*i) + 20 ,10*cos(2*pi/N*i)+20);
end

%%%% To simulate deployment scenario comment the two lines below 

vehicle{1}.init_position(23, 20);
vehicle{3}.init_position(16, 13);


%% Communication constraints
R   = 7; % maximum distance of communications - [m]
R__ = 6; % maximum distance of connectivity - [m]
R_  = 5; % minimum distance for cooperation - [m]

%% Vehicles constraints
% Vehicles swarm position constraints
% ||(x,y)_i-(x,y)_j|| > d_min
d_min = 3; % minimum distance between vehicles - [m]

% Vehicles input/speed constraints
Vx = 0.5; % max abs of speed along x - [m/s]
Vy = 0.5; % max abs of speed along y - [m/s]
T_max = 20; % max abs of motor thrust - [N]

%% Command Governor parameters
Psi = 1000*eye(2); % vehicle's references weight matrix
k0 = 25; % prediction horizon

%% Dynamic Command Governor
for i=1:N
    vehicle{i}.cg = DynamicDistribuitedCommandGovernor(i,Phi,G,Hc,L,Psi,k0,'gurobi');
    vehicle{i}.cg.add_vehicle_cnstr('speed',[Vx,Vy],'thrust',T_max);
end

%% Color the net
colors = 1:N;
for i=1:N
    vehicle{i}.color = colors(i);
end

plot_color = ['b', 'g', 'k', 'r', 'm'];

%% Simulation Colored Round CG
Tf = 150; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time
NT = ceil(Tf/Tc_cg); % simulation steps number


% References
for i=1:N
    r{i} = [10*sin(2*pi/N*i+2*2*pi/N) , 10*cos(2*pi/N*i+2*2*pi/N) ]' + 20 ;
    r_{i} = [10*sin(2*pi/N*i+2*2*pi/N) , 10*cos(2*pi/N*i+2*2*pi/N) ]'+ 20 ;
end

round = 1;

% Vectors needed to track times of solver
cputime= [];
yalmiptime=[];

for t=1:NT
    for i=1:N
        if vehicle{i}.color == colors(round)       
            for j=1:N
                if i~=j
                    d_ij = norm(vehicle{i}.ctrl_sys.sys.xi(1:2)-vehicle{j}.ctrl_sys.sys.xi(1:2));
                    if d_ij <= R
                        if d_ij > R__ && ~isempty(find(vehicle{j}.cg.id==vehicle{i}.cg.neigh))
                            fprintf('Unplugged: %d - %d \n',i,j);
                            % Remove constraints and send plug-out message to v{j}
                            vehicle{i}.cg.remove_swarm_cnstr(j);
                            vehicle{j}.cg.remove_swarm_cnstr(i);
                        end
                        
                        
                        % If I am not in a plug in operation and neither is
                        % the vehicle that i have encountered, then the
                        % check function is called the first time.
                        % If returns true than the plug in operation can be
                        % completed, else virtual references are extracted 
                        
                        if d_ij <= R_ && ... 
                                (vehicle{i}.pending_plugin == -1 || vehicle{i}.pending_plugin == 0)  && ...
                                vehicle{j}.pending_plugin == -1 && ... %%% No plugin pending
                                isempty(find(vehicle{j}.cg.id==vehicle{i}.cg.neigh)) %  ||v{i}-v{j}||∞ ≤ R && not already pending plug-in request
                            fprintf('Plug-in request from %d to %d \n',i,j);
                            vehicle{i}.pending_plugin = j;
                            
                            % Send plug-in request message to v{j}
                            
                            vehicle{j}.pending_plugin = i;
                            pluggable_status = vehicle{j}.cg.check([vehicle{j}.ctrl_sys.sys.xi; vehicle{j}.ctrl_sys.xci],...
                                [vehicle{i}.ctrl_sys.sys.xi; vehicle{i}.ctrl_sys.xci],vehicle{j}.g,vehicle{i}.g,[],d_min);
                            
                            if pluggable_status %== 'success'                                
                                fprintf('Plug-in success: %d - %d \n',i,j);
                                vehicle{j}.pending_plugin = -1;
                                vehicle{i}.pending_plugin = -1;
                                vehicle{j}.cg.add_swarm_cnstr(i,'anticollision',d_min); % Setted from v{j} after check
                                vehicle{i}.cg.add_swarm_cnstr(j,'anticollision',d_min);
                            else
                                fprintf('Unpluggable: %d - %d. Need a virtual command \n',i,j);
                                pluggable_status = 'virtual_cmd';
                                g_n = [];
                                for k = vehicle{j}.cg.neigh
                                    g_n = [g_n; vehicle{k}.g];
                                end                              
                                [g_j,g_ii] = vehicle{j}.cg.compute_virtual_cmd(vehicle{j}.g,vehicle{i}.ctrl_sys.sys.xi(1:2),g_n,[],d_min+0.1*d_min);
                                
                                g_n = [];
                                for k = vehicle{i}.cg.neigh
                                    g_n = [g_n; vehicle{k}.g];
                                end
                                
                                [g_i] = vehicle{i}.cg.compute_virtual_cmd_fixed(vehicle{i}.ctrl_sys.sys.xi(1:2),g_j,g_n,[],d_min+0.1*d_min);
                                
                                r{j} = g_j;
                                r{i} = g_i;
                                
                                % Send freeze request to v{j} neighbours
                                for k = vehicle{j}.cg.neigh
                                    vehicle{k}.pending_plugin = j;
                                    vehicle{k}.freeze = 1;
                                end
                                for k = vehicle{i}.cg.neigh
                                    vehicle{k}.pending_plugin = i;
                                    vehicle{k}.freeze = 1;
                                end
                            end
                        % If the vehicle encountered is already in a plug in
                        % operation, then a virtual reference is computed
                        % considering as objective the previous g reference
                       
                        elseif(d_ij <= R_ && not(vehicle{j}.pending_plugin == i) && not(vehicle{i}.pending_plugin == 0) &&...
                                not(vehicle{j}.pending_plugin == -1) && ...
                                vehicle{i}.pending_plugin == -1 && isempty(find(vehicle{j}.cg.id==vehicle{i}.cg.neigh)))
                            fprintf('Vehicle %d already in plug-in operations. Computing proper virtual reference(%d) \n',j,i);
                            
                            g_n = [];
                            for k = vehicle{i}.cg.neigh
                                g_n = [g_n; vehicle{k}.g];
                            end
                            
                            [g_i,g_j] = vehicle{i}.cg.compute_virtual_cmd(vehicle{i}.g,r{j},g_n,[],d_min+0.1*d_min);
                            r{i} = g_i;
                            vehicle{i}.pending_plugin = 0;
                        
                        end
                            
                        % It is necessary call again the check function in
                        % order to verify pluggability conditions 
                         if d_ij <= R_ && ... 
                                vehicle{i}.pending_plugin == j && vehicle{i}.freeze == 0 && ...
                                isempty(find(vehicle{j}.cg.id==vehicle{i}.cg.neigh)) 
                            
                                pluggable_status = vehicle{j}.cg.check([vehicle{j}.ctrl_sys.sys.xi; vehicle{j}.ctrl_sys.xci],...
                                    [vehicle{i}.ctrl_sys.sys.xi; vehicle{i}.ctrl_sys.xci],vehicle{j}.g,vehicle{i}.g,[],d_min);
                                if pluggable_status %== 'success'
                                    fprintf('Plug-in success: %d - %d \n',i,j);
                                    vehicle{j}.pending_plugin = -1;
                                    vehicle{i}.pending_plugin = -1;
                                    vehicle{j}.cg.add_swarm_cnstr(i,'anticollision',d_min); % Setted from v{j} after check
                                    vehicle{i}.cg.add_swarm_cnstr(j,'anticollision',d_min);
                                    r{j} = r_{j};
                                    r{i} = r_{i};
                                    % Send unfreeze request to v{j} neighbours
                                    for k = vehicle{j}.cg.neigh
                                        vehicle{k}.pending_plugin = -1;
                                        vehicle{k}.freeze = 0;
                                    end
                                    for k = vehicle{i}.cg.neigh
                                        vehicle{k}.pending_plugin = -1;
                                        vehicle{k}.freeze = 0;
                                    end
                                end                        
                         end
                        % If the vehicle has reached its virtual reference
                        % and has not been plugged, then the pending plug
                        % in variable has to be resetted 
                        if((vehicle{i}.pending_plugin==0) && norm(r{i}-vehicle{i}.ctrl_sys.sys.xi(1:2))< 0.1)
                            vehicle{i}.pending_plugin = -1;
                            r{i}=r_{i};
                        end
                    end
                end
            end
            
            
            
            if vehicle{i}.freeze == 0
                %%% If the vehicle has not been freezed by a neighbor
                
                x = vehicle{i}.ctrl_sys.sys.xi; % vehicle current state
                xc = vehicle{i}.ctrl_sys.xci; % controller current state
                xa = [x;xc];
                g_n = [];
                for k = vehicle{i}.cg.neigh
                    g_n = [g_n;vehicle{k}.g];
                    x = vehicle{k}.ctrl_sys.sys.xi; % vehicle{j} current state
                    xc = vehicle{k}.ctrl_sys.xci; % controller{j} current state
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
        vehicle{i}.ctrl_sys.sim(vehicle{i}.g,Tc_cg);
    end
    round = rem(round,length(colors))+1;
    
    figure(1);
    
    axis equal
    for k=1:N
        % Trajectory
        plot(vehicle{k}.ctrl_sys.sys.x(1,:),vehicle{k}.ctrl_sys.sys.x(2,:), strcat(plot_color(k), '-.'),'LineWidth',0.8);
        hold on;
        plot(vehicle{k}.ctrl_sys.sys.x(1,end),vehicle{k}.ctrl_sys.sys.x(2,end), strcat(plot_color(k), 'o'),'MarkerFaceColor',plot_color(k),'MarkerSize',7);
        %%%% live plot %%%%
        plot(r{k}(1), r{k}(2), strcat(plot_color(k), 'o'));
        plot(vehicle{k}.g(1), vehicle{k}.g(2), strcat(plot_color(k), 'x'));
        %%%%%%%%%%%%
    end
    
    hold off;
    
    drawnow;
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