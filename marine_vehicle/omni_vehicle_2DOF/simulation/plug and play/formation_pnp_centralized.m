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
    if(i==1 || i==2 || i==3)
        if(i==2)
            vehicle{i}.init_position(i*2 ,6.5);
        else
            vehicle{i}.init_position(i*2 ,3);
        end
        
    else
        if(i==5)
            vehicle{i}.init_position((i-3)*2 ,8);
        else
            vehicle{i}.init_position((i-3)*2.2 ,10);
        end
        
    end
end

%% Communication constraints
R   = 7; % maximum distance of communications - [m]
R__ = 6; % maximum distance of connectivity - [m]
R_  = 3.5; % minimum distance for cooperation - [m]

%% Vehicles constraints
% Vehicles swarm position constraints
% ||(x,y)_i-(x,y)_j||∞ ≤ d_max
% ||(x,y)_i-(x,y)_j||∞ ≥ d_min
d_max = 200;  % maximum distance between vehicles - [m]
d_min = 2; % minimum distance between vehicles - [m]

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
    if(i==1 || i==4)
        vehicle{i}.cg.add_vehicle_cnstr('speed',[Vx,Vy]*3.5,'thrust',T_max*3.5);
    else
        vehicle{i}.cg.add_vehicle_cnstr('speed',[Vx,Vy]*2,'thrust',T_max*2);
    end
    
end

%% Color the net
colors = 1:N;
for i=1:N
    vehicle{i}.color = colors(i);
end

vehicle{1}.cg.add_swarm_cnstr(3,'proximity',d_max,'anticollision',d_min);
vehicle{3}.cg.add_swarm_cnstr(1,'proximity',d_max,'anticollision',d_min);

vehicle{2}.cg.add_swarm_cnstr(3,'proximity',d_max,'anticollision',d_min);
vehicle{3}.cg.add_swarm_cnstr(2,'proximity',d_max,'anticollision',d_min);

vehicle{1}.cg.add_swarm_cnstr(2,'proximity',d_max,'anticollision',d_min);
vehicle{2}.cg.add_swarm_cnstr(1,'proximity',d_max,'anticollision',d_min);


vehicle{4}.cg.add_swarm_cnstr(5,'proximity',d_max,'anticollision',d_min);
vehicle{5}.cg.add_swarm_cnstr(4,'proximity',d_max,'anticollision',d_min);

%% Simulation Colored Round CG
Tf = 150; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time
NT = ceil(Tf/Tc_cg); % simulation steps number

plot_color = ['b', 'g', 'k', 'r', 'm'];


% References
for i=1:N
    if(i==1 || i==2 || i==3)
        if(i==2)
        r{i} = [ i*2 ; 10 ]; 
        r_{i} =  [ i*2 ; 10 ]; 
        else
        r{i} = [ i*2 ; 10 ]; 
        r_{i} =  [ i*2 ; 10 ]; 
        end

    else
        r{i} = [ (i-3)*2 ; 0 ];
        r_{i} =  [ (i-3)*2 ; 0 ];
    end
end
% r{2} = vehicle{2}.ctrl_sys.sys.xi(1:2);
% r{4} = vehicle{4}.ctrl_sys.sys.xi(1:2);
% r{5} = vehicle{5}.ctrl_sys.sys.xi(1:2);
% 
% r_{2} = vehicle{2}.ctrl_sys.sys.xi(1:2);
% r_{4} = vehicle{1}.ctrl_sys.sys.xi(1:2);
% r_{5} = vehicle{1}.ctrl_sys.sys.xi(1:2);

r_virtual = r;

round = 1;
for t=1:NT
    for i=1:N
        if vehicle{i}.color == colors(round)
            % Receive msg from v{j} : ||v{i}-v{j}||∞ ≤ R
           
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
                        
                        if d_ij <= R__ && d_ij > R_
                        end
                        % Should be considerated an else condition if i ask
                        % to plug with a vehicle already in a
                        % pending_plugin
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
%                                 % Send unfreeze request to v{j} neighbours
%                                 for k = vehicle{j}.cg.neigh
%                                     vehicle{k}.pending_plugin = -1;
%                                     vehicle{k}.freeze = 0;  % 1 is eq to frez.
%                                 end
                           
                            else
                                fprintf('Unpluggable: %d - %d. Need a virtual command \n',i,j);
                                pluggable_status = 'virtual_cmd';
                                
                                r_ofall = [vehicle{i}.ctrl_sys.sys.xi(1:2)];
                                
                                for k = vehicle{i}.cg.neigh
                                    r_ofall = [r_ofall; vehicle{k}.g];
                                end
                                
                                r_ofall = [r_ofall; vehicle{j}.g];
                                
                                for k = vehicle{j}.cg.neigh
                                    r_ofall = [r_ofall; vehicle{k}.g];
                                end
                                
                                [r_all] = vehicle{i}.cg.compute_all_virtual_cmds(r_ofall,[],d_min+0.1*d_min);
                                
                                
                                r{i} = r_all(1:2);
                                
                                count=3;
                                
                                for k = vehicle{i}.cg.neigh
                                    r_virtual{k} = r_all(count:count+1);
                                    count = count+2;
                                end
                                
                                r{j} = r_all(count:count+1);
                                
                                count = count+2; 
                                
                                for k = vehicle{j}.cg.neigh
                                     r_virtual{k} = r_all(count:count+1);
                                     count = count+2;
                                end

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
                        if((vehicle{i}.pending_plugin==0) && norm(r{i}-vehicle{i}.ctrl_sys.sys.xi(1:2))< 0.1)
                            vehicle{i}.pending_plugin = -1;
                            r{i}=r_{i};
                        end
                    end
                end
            end
            
            
            
            if vehicle{i}.freeze == 0
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
                    %                     cputime= [cputime,s.solvertime];
                    %                     yalmiptime=[yalmiptime,s.yalmiptime];
                else
                    disp('WARN: old references');
                    t,i
                end
            else
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
                
                [g,s] = vehicle{i}.cg.compute_cmd(xa,r_virtual{i},g_n);
                if ~isempty(g)
                    vehicle{i}.g = g;
                    %                     cputime= [cputime,s.solvertime];
                    %                     yalmiptime=[yalmiptime,s.yalmiptime];
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
        %         axis([0 5 -4 4])
        plot(vehicle{k}.ctrl_sys.sys.x(1,:),vehicle{k}.ctrl_sys.sys.x(2,:), strcat(plot_color(k), '-.'),'LineWidth',0.8);
        hold on;
        plot(vehicle{k}.ctrl_sys.sys.x(1,end),vehicle{k}.ctrl_sys.sys.x(2,end), strcat(plot_color(k), 'o'),'MarkerFaceColor',plot_color(k),'MarkerSize',7);
        %%%% live plot %%%%
        plot(r{k}(1), r{k}(2), strcat(plot_color(k), 'o'));
        plot(r_virtual{k}(1), r_virtual{k}(2), strcat(plot_color(k), 'd'));
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