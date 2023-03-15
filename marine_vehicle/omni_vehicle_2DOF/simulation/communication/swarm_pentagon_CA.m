% This simulation shows how a swarm, in order to tackle the deadlock issued
% by the presence of an obstacle and the proximity constraints, avoides an
% obstacle going all in one direction 

%% Clear workspace
clear;  close all;

%% Load vehicles' model matrices
addpath('../../../../marine_vehicle');      addpath('../../../../marine_vehicle/omni_vehicle_2DOF');
addpath(genpath('../../../../util'));       addpath('../../../../CG');
addpath(genpath('../../../../tbxmanager'));

set(0, 'DefaultLineLineWidth', 3);
vehicle_2DOF_model_2

%%%% Simulation Settings
%% Init vehicles
N = 5; % number of vehicles
vehicle = cell(1, N);
for i=1:N
    vehicle{i} = ControlledVehicle(ControlledSystem_LQI(StateSpaceSystem(A,B),Tc,Fa,Cy,Phi,G,Hc,L));
    vehicle{i}.init_position(11 ,i*4 - 5);
    r{i} = [8*sin(2*pi/N*i) + 25,8*cos(2*pi/N*i)+5]';
    cloud_points(:,i) = [6*sin(2*pi/N*i) + 25,6*cos(2*pi/N*i)+5]';
    r_{i} = r{i};
end

%% References Initialization 
% 
r{1} = vehicle{1}.ctrl_sys.sys.xi(1:2) + [40;0];
r{2} = vehicle{2}.ctrl_sys.sys.xi(1:2) + [40;0];
r{3} = vehicle{3}.ctrl_sys.sys.xi(1:2) + [40;0];
r{4} = vehicle{4}.ctrl_sys.sys.xi(1:2) + [40;0];
r{5} = vehicle{5}.ctrl_sys.sys.xi(1:2) + [40;0];

r_{1} = r{1};
r_{2} = r{2};
r_{3} = r{3};
r_{4} = r{4};
r_{5} = r{5};

%% Net configuration

spanning_tree =[-1  1  0  0  0;
                 1 -1  1  0  0;
                 0  1 -1  1  0;
                 0  0  1 -1  1;
                 0  0  0  1 -1];
%%%%%%%%%%%%%

             
% It is assumed that vehicle 1 is the root of the spanning tree, and cant
% change
vehicle{1}.parent = 0;

% Parent initialization 
for i=2:N
    vehicle{i}.parent = i-1;
end


%% Vehicles constraints
% Vehicles swarm position constraints
% ||(x,y)_i-(x,y)_j|| < d_max
% ||(x,y)_i-(x,y)_j|| > d_min
d_max = 6;  % maximum distance between vehicles - [m]
d_min = 1; % minimum distance between vehicles - [m]

% Vehicles input/speed constraints
Vx = 0.8; % max abs of speed along x - [m/s]
Vy = 0.8; % max abs of speed along y - [m/s]
T_max = 30; % max abs of motor thrust - [N]

%% Command Governor parameters
Psi = 1000*eye(2); % vehicle's references weight matrix
k0 = 30; % prediction horizon

figure;
legend_list = {'vehicle 1 trajectory','vehicle 2 trajectory', 'vehicle 3 trajectory','vehicle 4 trajectory', 'vehicle 5 trajectory'};

%%% Vector for distance for plotting purposes
dist = [];
dist2 = [];
dist3 = [];
dist4 = [];

% Pentagon shaped obstacle

pentagon = polyshape( cloud_points(1,:),cloud_points(2,:));
hypeblack = [];

%% Dynamic Command Governor
for i=1:N
    vehicle{i}.cg = DynamicDistribuitedCommandGovernor(i,Phi,G,Hc,L,Psi,k0,'gurobi');
    vehicle{i}.cg.add_vehicle_cnstr('speed',[Vx,Vy],'thrust',T_max);
    for j=1:N
        if(not (i==j))
            if(spanning_tree(i,j) == 1) % i,j is neighbour
                vehicle{i}.cg.add_swarm_cnstr(j,'anticollision',d_min, 'proximity',d_max);
            else
                vehicle{i}.cg.add_swarm_cnstr(j,'anticollision',d_min);
            end
        end
    end
end

%% Color the net
colors = 1:N;

for i=1:N
    vehicle{i}.color = colors(i);
end

plot_color = ['b', 'g', 'k', 'r', 'm'];

%% Simulation Colored Round CG
Tf = 80; % simulation time
Tc_cg = 1*vehicle{1}.ctrl_sys.Tc; % references recalculation time
NT = ceil(Tf/Tc_cg); % simulation steps number


% writerObj = VideoWriter('Swarm_Pentagon_multihy.avi');
% writerObj.FrameRate = 10;
% open(writerObj);
cont = 0 ;
plot_hy = [0, 0 ,0 ,0 ,0];

g_ = {[], [], [], [], []};
state_ = {[], [], [], [], []};
time_ = [];
round = 1;
for t=1:NT
    for i=1:N
        
        if vehicle{i}.color == colors(round)
            index_min = -1;
            d_ijmin = 100;
             % Search for the closest vehicle
%             for j=1:N
%                 if i~=j && not(vehicle{i}.parent==j) && not(vehicle{i}.parent==0)
%                     d_ij = norm(vehicle{i}.ctrl_sys.sys.xi(1:2)-vehicle{j}.ctrl_sys.sys.xi(1:2));
%                     if(d_ij < d_ijmin)
%                         index_min = j;
%                         d_ijmin = d_ij;
%                     end
%                 end
%             end
            
%             
%             if(not(vehicle{i}.parent==0))
%                 % Check if the distance between the actual parent is larger
%                 % than the distance with the closest vehicle 
%                 if(d_ijmin < norm(vehicle{i}.ctrl_sys.sys.xi(1:2)-vehicle{vehicle{i}.parent}.ctrl_sys.sys.xi(1:2)))
%                     spanning_tree_tmp = spanning_tree;
%                     spanning_tree_tmp(i,vehicle{i}.parent) = 0;
%                     spanning_tree_tmp(vehicle{i}.parent,i) = 0;
%                     spanning_tree_tmp(i,index_min) = 1;
%                     spanning_tree_tmp(index_min,i) = 1;
%                     if(isConnected(spanning_tree_tmp))                     
%                         % Remove the old parent proximity constraints
%                         vehicle{vehicle{i}.parent}.cg.remove_swarm_cnstr(i);
%                         vehicle{i}.cg.remove_swarm_cnstr(vehicle{i}.parent);
%                         vehicle{vehicle{i}.parent}.cg.add_swarm_cnstr(i,'anticollision',d_min);
%                         vehicle{i}.cg.add_swarm_cnstr(vehicle{i}.parent,'anticollision',d_min);
%                         
%                         % Change parent
%                         vehicle{i}.parent = index_min;
%                         
%                         fprintf('***************Parent changed (%d ,%d) **************** \n ',i,index_min);
%                         
%                         % Add proximity constraints with new parent
%                         vehicle{vehicle{i}.parent}.cg.remove_swarm_cnstr(i);
%                         vehicle{i}.cg.remove_swarm_cnstr(vehicle{i}.parent);
%                         
%                         vehicle{vehicle{i}.parent}.cg.add_swarm_cnstr(i,'proximity',d_max,'anticollision',d_min);
%                         vehicle{i}.cg.add_swarm_cnstr(vehicle{i}.parent,'proximity',d_max,'anticollision',d_min);
%                         
%                         spanning_tree = spanning_tree_tmp;
%                     else
%                         fprintf('************Change parent request denied (%d ,%d) **************** \n ',i,index_min);
%                     end
%                 end
%             end
            
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
            
            % Only the first vehicle compute the formation shared
            % hyperplane
            if(i==1)           
                [g,s, hypeblack] = vehicle{i}.cg.compute_cmd(xa,r{i},g_n,cloud_points,[]);
            else          
                [g,s, hypeblack] = vehicle{i}.cg.compute_cmd(xa,r{i},g_n,cloud_points,hypeblack);
            end
%             
%             [g,s,hypeblack] = vehicle{i}.cg.compute_cmd(xa,r{i},g_n,cloud_points); %,[]);
            cont = i;    
            if ~isempty(g)
                vehicle{i}.g = g;
            else
                disp('WARN: old references');
                t,i
            end
            g_{i} = [g_{i}; g];
            state_{i} = [state_{i}; x];
            time_ = [time_; t];
        end
    end
    % Simulate for Tc_cg
    for i=1:N
        vehicle{i}.ctrl_sys.sim(vehicle{i}.g,Tc_cg);
    end
    round = rem(round,length(colors))+1;
    
    clf;
    plot_struct = [];
    title('Swarm Pentagon Simulation');
    xlabel('x[m]');
    ylabel('y[m]');
    grid on;
    
    %axis([0 50 -25 25]);
    
    pentagon = polyshape( cloud_points(1,:),cloud_points(2,:));
    hold on
    plot(pentagon);
    
    r_axis = [0 55 -25 30];
    plot_struct = [plot_struct, plot_2Dof_vehicle(vehicle{1},r,0,'MarkerSize', 6,'Color', 'k', 'RangeAxis', r_axis, 'LineWidth', 3, 'D_min_style', '--')];
    plot_struct = [plot_struct, plot_2Dof_vehicle(vehicle{2},r,0,'MarkerSize', 6,'Color', 'm', 'RangeAxis', r_axis, 'LineWidth', 3, 'D_min_style', '--')];
    plot_struct = [plot_struct, plot_2Dof_vehicle(vehicle{3},r,0,'MarkerSize', 6,'Color', 'g', 'RangeAxis', r_axis, 'LineWidth', 3, 'D_min_style', '--')];
    plot_struct = [plot_struct, plot_2Dof_vehicle(vehicle{4},r,0,'MarkerSize', 6, 'Color', 'c', 'RangeAxis', r_axis, 'LineWidth', 3, 'D_min_style', '--')];
    plot_struct = [plot_struct, plot_2Dof_vehicle(vehicle{5},r,0,'MarkerSize', 6,'Color', 'b', 'RangeAxis', r_axis, 'LineWidth', 3, 'D_min_style', '--')];
    plot(vehicle{1}.g(1),vehicle{1}.g(2),'xk');
    plot(vehicle{2}.g(1),vehicle{2}.g(2),'xm');
    plot(vehicle{3}.g(1),vehicle{3}.g(2),'xg');
    plot(vehicle{4}.g(1),vehicle{4}.g(2),'xc');
    plot(vehicle{5}.g(1),vehicle{5}.g(2),'xb');
    plot(r{1}(1),r{1}(2),'ko');
    plot(r{2}(1),r{2}(2),'mo');
    plot(r{3}(1),r{3}(2),'go');
    plot(r{4}(1),r{4}(2),'co');
    plot(r{5}(1),r{5}(2),'bo');
    
%     for k=1:N
%         plot_struct = [plot_struct, plot_2Dof_vehicle(vehicle{k},'Color', strcat(plot_color(k)), 'RangeAxis', [0 50 -25 25], 'LineWidth', 3, 'D_min_style', '--')];
%     end
%     
     if(not(isempty(hypeblack)))
         hold on;
         %plot(hypeblack, 'k');
         if(cont == 1 && plot_hy(1) == 0 )
             plot(hypeblack,'k', 'LineWidth', 1);
             %plot_hy(1) = 1; 
         end
         if(cont == 2 && plot_hy(2) == 0 ) 
             plot(hypeblack,'m', 'LineWidth', 1);
             %plot_hy(2) = 1; 
         end
         if(cont == 3 && plot_hy(3) == 0)
              plot(hypeblack,'g', 'LineWidth', 1);
              %plot_hy(3) = 1; 
         end
         if(cont == 4 && plot_hy(4) == 0)
              plot(hypeblack,'c', 'LineWidth', 1);
              %plot_hy(4) = 1; 
         end
         if(cont == 5 && plot_hy(5) == 0 )
              plot(hypeblack,'b', 'LineWidth', 1);
              %plot_hy(5) = 1; 
         end
     end
%     for k=1:N
%         % Trajectory
%         plot(vehicle{k}.ctrl_sys.sys.x(1,:),vehicle{k}.ctrl_sys.sys.x(2,:), strcat(plot_color(k), '-.'),'LineWidth',0.8);
%         hold on;
%         axis([0 50 -25 25]);
%         plot(vehicle{k}.ctrl_sys.sys.x(1,end),vehicle{k}.ctrl_sys.sys.x(2,end), strcat(plot_color(k), 'o'),'MarkerFaceColor',plot_color(k),'MarkerSize',7);
%         %%%% live plot %%%%
%         plot(r{k}(1), r{k}(2), strcat(plot_color(k), 'o'));
%         plot(r_{k}(1), r_{k}(2), strcat(plot_color(k), 'o'));
%         plot(vehicle{k}.g(1), vehicle{k}.g(2), strcat(plot_color(k), 'x'));
%         plot(pentagon);
%         %%%%%%%%%%%% Hyperplane plot
% %         if(k==1 && not(isempty(hypeblack)) )
% %             plot(hypeblack);
% %         end
%         % Plotting of the parent connection 
%         if(not(vehicle{k}.parent==0))
%             v = vehicle{k}.ctrl_sys.sys.xi(1:2);
%             p =  vehicle{vehicle{k}.parent}.ctrl_sys.sys.xi(1:2);
%             plot([v(1), p(1)], [v(2), p(2)], strcat(plot_color(k), ':'),'LineWidth',1);
% 
%         end
% 
%     end
    dist = [dist, norm((vehicle{1}.ctrl_sys.sys.x(1:2,end)-vehicle{2}.ctrl_sys.sys.x(1:2,end)),inf)];
    dist2 = [dist2, norm((vehicle{2}.ctrl_sys.sys.x(1:2,end)-vehicle{3}.ctrl_sys.sys.x(1:2,end)),inf)];
    dist3 = [dist3, norm((vehicle{3}.ctrl_sys.sys.x(1:2,end)-vehicle{4}.ctrl_sys.sys.x(1:2,end)),inf)];
    dist4 = [dist3, norm((vehicle{4}.ctrl_sys.sys.x(1:2,end)-vehicle{5}.ctrl_sys.sys.x(1:2,end)),inf)];
    
    
    legend(plot_struct, legend_list);
    drawnow;
%     writeVideo(writerObj, im2frame(print('-RGBImage', strcat('-r', '300'))));
    hold off;
end

% close(writerObj);

figure;
subplot(4, 1, 1);
plot(0:0.1:t*0.1 - 0.1, dist3);
hold on;
plot(0:0.1:t*0.1 - 0.1,d_min*ones(1,length(0:0.1:t*0.1 - 0.1)));
title('Distance between vehicles 3 and vehicle 4');
xlabel('time [s]');
ylabel('distance [m]');

%%%%%%%%%%%%
subplot(4, 1, 2);
plot(1:t, dist2);
hold on;
plot(1:t,d_min*ones(1,length(1:t)));
title('Distance between vehicles 2 and vehicle 3');
xlabel('time [s]');
ylabel('distance [m]');

%%%%%%%%%
subplot(4, 1, 3);
plot(1:NT, dist3);
hold on;
plot(1:NT,d_max*ones(1,length(1:NT)));
title('Distance between vehicles 3 and vehicle 4');
xlabel('time [s]');
ylabel('distance [m]');
%

%%%%%%%%%
subplot(4, 1, 4);
plot(1:NT, dist3);
hold on;
plot(1:NT,d_max*ones(1,length(1:NT)));
title('Distance between vehicles 4 and vehicle 5');
xlabel('time [s]');
ylabel('distance [m]');
%