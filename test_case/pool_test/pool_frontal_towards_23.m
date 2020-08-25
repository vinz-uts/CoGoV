vehicle_2DOF_cooperation_pool_frontal_23

           
%% Planner
limits = [-Max_x, Max_x, Max_y, -Max_y];

% pl(1) =  BorderPlanner(0.5, 0.3, limits, 0.15);
% pl(2) =  BorderPlanner(0.5, -0.7, limits, 0.15);
% pl(3) =  BorderPlanner(0.5, 0.2, limits, 0.15);

% Color the net
colors = [0,1];
vehicle{1}.color = colors(1);
vehicle{2}.color = colors(2);
vehicle{3}.color = colors(2);

%% Simulation Colored Round CG
Tf = 5; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time
% r{1} = [4,0.5]'; % position references
% r{2} = [3,1]'; % position references
% r{3} = [3,-1.5]'; % position references
NT = ceil(Tf/Tc_cg); % simulation steps number
zerr = [0,0]';
figure(1);
hold on;
x_plot = -Max_x:0.1:Max_x;
y_plot = -Max_y:0.1:Max_y;
plot(ones(size(y_plot))*x_plot(1), y_plot);
plot(x_plot, ones(size(x_plot))*y_plot(1));
plot(ones(size(y_plot))*x_plot(end), y_plot);
plot(x_plot, ones(size(x_plot))*y_plot(end));
axis([-Max_x-1, Max_x + 1, -Max_y - 1, Max_y + 1]);
dist=[];
hold on;

round = 1;
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
            
 %
%             plan = pl(i);
%             [r, pl(i)] = plan.compute_reference(vehicle{i}.ctrl_sys.sys);
            
            if(i==1)
                r=[1,0]';
            end
            
            if(i==2)
                r=[0,-1]';
            end
            
            if(i==3)
                r=[0,1]';
            end
            
            
            g = vehicle{i}.cg.compute_cmd(xa, r, g_n);
            
            if(i==1)
                plot(r(1), r(2), 'bx');
                if(not(isempty(g)))
                    if(norm(g-zerr)==0)
                        
                        disp('WARN: Zero reference');
                    end
                    plot(g(1), g(2), 'rx');
                end
            end
            if ~isempty(g)
                vehicle{i}.g = g;
                if(i==1)
                    vehicle{i}.g 
                    % perché va a zero?????? 
                end
            else
                disp('WARN: old references');
                if(i==1)
                    vehicle{i}.g 
                end
                t,i
            end
        end
        end
    
        
    for i=1:N
        vehicle{i}.ctrl_sys.sim(vehicle{i}.g,Tc_cg);
    end
    round = rem(round,length(colors))+1;
%    
    %t=96 , dmin = 0.3
    for k=1:N
    % Trajectory
    figure(1);  hold on;
    plot(vehicle{k}.ctrl_sys.sys.x(1,:),vehicle{k}.ctrl_sys.sys.x(2,:),'.');
%     plot(vehicle{k}.ctrl_sys.sys.x(1,end),vehicle{k}.ctrl_sys.sys.x(2,end),'o');
    drawnow;
    end
     dist=[dist, norm((vehicle{1}.ctrl_sys.sys.x(1:2,end)-vehicle{2}.ctrl_sys.sys.x(1:2,end)))];
end
 
dati = struct('x_vehicle1',vehicle{1}.ctrl_sys.sys.x(1,:),'y_vehicle1',vehicle{1}.ctrl_sys.sys.x(2,:),...
    'x_vehicle2',vehicle{2}.ctrl_sys.sys.x(1,:),'y_vehicle2',vehicle{2}.ctrl_sys.sys.x(2,:),...
    'x_vehicle3',vehicle{3}.ctrl_sys.sys.x(1,:),'y_vehicle3',vehicle{3}.ctrl_sys.sys.x(2,:),'distance', dist);
 
save('vehicle_towards23_03', 'dati');



