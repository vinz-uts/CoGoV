%% Configure vehicle swarm
vehicle_2DOF_cooperation_centralized

%% Color the net
colors = [0,1];
vehicle{1}.color = colors(1);
vehicle{2}.color = colors(2);
vehicle{3}.color = colors(2);

%% Simulation Colored Round CG
Tf = 20; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time
r{1} = [4,0.5]'; % position references
r{2} = [3,1]'; % position references
r{3} = [3,-1.5]'; % position references
NT = ceil(Tf/Tc_cg); % simulation steps number
nr = size(r{1},1); % size of single vehicle reference

cputime =[];
yalmiptime = [];

round = 1;
for t=1:NT 
  xa = [];
    for i=1:N
        x = vehicle{i}.ctrl_sys.sys.xi; % vehicle current state
        xc = vehicle{i}.ctrl_sys.xci; % controller current state
        xa = [xa;x;xc];

        r_ = [];
        for j=1:N
            r_ = [r_;r{j}];
        end
        
    end
    g = cg.compute_cmd(xa,r_);
    if ~isempty(g)
        for i=1:N
            vehicle{i}.g = g(((i-1)*nr)+1:((i-1)*nr)+nr);
        end
    else
        disp('WARN: old references');
        t,i
    end
    
    for i=1:N
        vehicle{i}.ctrl_sys.sim(vehicle{i}.g,Tc_cg);
    end
    round = rem(round,length(colors))+1;
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
