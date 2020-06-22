%% Close plot windows
close all;

%% Configure vehicle swarm
vehicle_2DOF_cooperation

%% Simulation Parallel CG
Tf = 5; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time
r{1} = [4,0.5]'; % position references
r{2} = [3,1]'; % position references
r{3} = [3,-1.5]'; % position references
NT = ceil(Tf/Tc_cg); % simulation steps number

old_g = cell(N,1);
for i=1:N
    old_g{i} = vehicle{i}.g;
end

for t=1:NT 
    for i=1:N
        x = vehicle{i}.ctrl_sys.sys.xi; % vehicle current state
        xc = vehicle{i}.ctrl_sys.xci; % controller current state
        xa = [x;xc];
        g_n = [];
        for j=1:N
            if adj_matrix(i,j) == 1 % i,j is neighbour
                g_n = [g_n;old_g{j}]; %[g_n;vehicle{j}.g];
                %g_n = [g_n;vehicle{j}.g];
                x = vehicle{j}.ctrl_sys.sys.xi; % vehicle current state
                xc = vehicle{j}.ctrl_sys.xci; % controller current state
                xa = [xa;x;xc];
            end
        end
        
        g = vehicle{i}.cg.compute_cmd(xa,r{i},g_n);
        if ~isempty(g)
            vehicle{i}.g = g;
        else
            disp('WARN: old references');
            t,i
        end
        old_g{i} = vehicle{i}.g;
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
