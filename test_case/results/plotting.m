clear;
close all;


%%%%%%%%%%
%1 Pool No comunication (on edge reference)
%2 Towards 0.5 m
%3 Towards 0.3 m, k0 = 10 
%4 Towards 0.3 m, k0 = 30
%5 Frontal (min distance = 0.4 m)
%6 Towards 2-3 0.5 m
%7 Towards 2-3 0.3 m
%8 Pool comunication (3 still, min distance = 0.5 m)
%9 Circular 23 frontal
%10 Circular 1 2 crossed
%11 Circular 1 2 3 crossed

plot1 = 11;

if(plot1 == 1)
    load('pool_nocomunication');
    Max_x = 2;
    Max_y = 2;
    limits = [-Max_x, Max_x, Max_y, -Max_y];
    figure;
    hold on;
    x_plot = -Max_x:0.1:Max_x;
    y_plot = -Max_y:0.1:Max_y;
    plot(ones(size(y_plot))*x_plot(1), y_plot);
    plot(x_plot, ones(size(x_plot))*y_plot(1));
    plot(ones(size(y_plot))*x_plot(end), y_plot);
    plot(x_plot, ones(size(x_plot))*y_plot(end));
    axis([-Max_x-1, Max_x + 1, -Max_y - 1, Max_y + 1]);
    hold on;
    for i=1:2:length(dati.x_vehicle1)
         axis([-3, 3, -3, 3]);
         plot(dati.x_vehicle1(i), dati.y_vehicle1(i), '.g');
         plot(dati.x_vehicle2(i), dati.y_vehicle2(i), '.b');
         plot(dati.x_vehicle3(i), dati.y_vehicle3(i), '.r');
         drawnow;
    end
    
end

if(plot1 == 2)
    load('vehicle_towards5');
    figure;
    hold on;

    for i=1:3:length(dati.x_vehicle1)-900
         axis([-2, 2, -2, 2]);
         plot(dati.x_vehicle1(i), dati.y_vehicle1(i), '.g');
         plot(dati.x_vehicle2(i), dati.y_vehicle2(i), '.b');
         plot(dati.x_vehicle3(i), dati.y_vehicle3(i), '.r');
         drawnow;
    end
    
    figure;
    plot(1:length(dati.distance), 0.5*ones(1,length(dati.distance)));
    hold on;
    plot(1:50,dati.distance);
    
end


if(plot1 == 3)
    load('vehicle_towards3');
    figure;
    hold on;

    

    for i=1:3:length(dati.x_vehicle1)-900
         axis([-2, 2, -2, 2]);
         plot(dati.x_vehicle1(i), dati.y_vehicle1(i), '.g');
         plot(dati.x_vehicle2(i), dati.y_vehicle2(i), '.b');
         plot(dati.x_vehicle3(i), dati.y_vehicle3(i), '.r');
         drawnow;
    end
    
    figure;
    plot(1:length(dati.distance), 0.3*ones(1,length(dati.distance)));
    hold on;
    plot(1:50,dati.distance);
    
end

if(plot1 == 4)
    load('vehicle_towards3_k0_15');
    figure;
    hold on;

    for i=1:3:length(dati.x_vehicle1)-900
         axis([-2, 2, -2, 2]);
         plot(dati.x_vehicle1(i), dati.y_vehicle1(i), '.g');
         plot(dati.x_vehicle2(i), dati.y_vehicle2(i), '.b');
         plot(dati.x_vehicle3(i), dati.y_vehicle3(i), '.r');
         drawnow;
    end
    
    figure;
    plot(1:length(dati.distance), 0.3*ones(1,length(dati.distance)));
    hold on;
    plot(1:50,dati.distance);
    
end

if(plot1 == 5)
    load('vehicle_frontalx');
    figure;
    hold on;
    plot(1.5,1,'xb');
    plot(1,1.5,'xg');

    for i=1:3:length(dati.x_vehicle1)-900
         axis([-2, 2, -2, 2]);
         plot(dati.x_vehicle1(i), dati.y_vehicle1(i), '.g');
         plot(dati.x_vehicle2(i), dati.y_vehicle2(i), '.b');
         plot(dati.x_vehicle3(i), dati.y_vehicle3(i), '.r');
         drawnow;
    end
    
    figure;
    plot(1:length(dati.distance), 0.4*ones(1,length(dati.distance)));
    hold on;
    plot(1:50,dati.distance);
    
end

if(plot1 == 6)
    load('vehicle_towards23_05');
    figure;
    hold on;

    for i=1:3:length(dati.x_vehicle1)-900
         axis([-2, 2, -2, 2]);
         plot(dati.x_vehicle1(i), dati.y_vehicle1(i), '.g');
         plot(dati.x_vehicle2(i), dati.y_vehicle2(i), '.b');
         plot(dati.x_vehicle3(i), dati.y_vehicle3(i), '.r');
         drawnow;
    end
    
    figure;
    plot(1:length(dati.distance), 0.5*ones(1,length(dati.distance)));
    hold on;
    plot(1:50,dati.distance);
    
end

if(plot1 == 7)
    load('vehicle_towards23_03');
    figure;
    hold on;

    for i=1:3:length(dati.x_vehicle1)-900
         axis([-2, 2, -2, 2]);
         plot(dati.x_vehicle1(i), dati.y_vehicle1(i), '.g');
         plot(dati.x_vehicle2(i), dati.y_vehicle2(i), '.b');
         plot(dati.x_vehicle3(i), dati.y_vehicle3(i), '.r');
         drawnow;
    end
    
    figure;
    plot(1:length(dati.distance), 0.3*ones(1,length(dati.distance)));
    hold on;
    plot(1:50,dati.distance);
    
end

if(plot1 == 8)
    load('pool_dist05');
    Max_x = 2;
    Max_y = 2;
    limits = [-Max_x, Max_x, Max_y, -Max_y];
    figure;
    hold on;
    x_plot = -Max_x:0.1:Max_x;
    y_plot = -Max_y:0.1:Max_y;
    plot(ones(size(y_plot))*x_plot(1), y_plot);
    plot(x_plot, ones(size(x_plot))*y_plot(1));
    plot(ones(size(y_plot))*x_plot(end), y_plot);
    plot(x_plot, ones(size(x_plot))*y_plot(end));
    axis([-Max_x-1, Max_x + 1, -Max_y - 1, Max_y + 1]);
    hold on;
    for i=1:6:length(dati.x_vehicle1)-200
         axis([-3, 3, -3, 3]);
         plot(dati.x_vehicle1(i), dati.y_vehicle1(i), '.g');
         plot(dati.x_vehicle2(i), dati.y_vehicle2(i), '.b');
         plot(dati.x_vehicle3(i), dati.y_vehicle3(i), '.r');
         drawnow;
    end
    figure;
    plot(1:length(dati.distance), 0.5*ones(1,length(dati.distance)));
    hold on;
    plot(1:length(dati.distance),dati.distance);
    
end

if(plot1 == 9)
    load('vehicle_circular23');
    figure;
    hold on;

    for i=1:5:length(dati.x_vehicle1)
         axis([-2, 2, -2, 2]);
         %plot(dati.x_vehicle1(i), dati.y_vehicle1(i), '.g');
         plot(dati.x_vehicle2(i), dati.y_vehicle2(i), '.b');
         plot(dati.x_vehicle3(i), dati.y_vehicle3(i), '.r');
         drawnow;
    end
    
    figure;
    plot(1:length(dati.distance), 0.3*ones(1,length(dati.distance)));
    hold on;
    plot(1:length(dati.distance),(dati.distance));
    
end

if(plot1 == 10)
    load('vehicle_circular_crossed');
    figure;
    hold on;

    for i=1:10:length(dati.x_vehicle1)
         axis([-2, 2, -2, 2]);
         plot(dati.x_vehicle1(i), dati.y_vehicle1(i), '.g');
         plot(dati.x_vehicle2(i), dati.y_vehicle2(i), '.b');
         %plot(dati.x_vehicle3(i), dati.y_vehicle3(i), '.r');
         drawnow;
    end
    
    figure;
    plot(1:length(dati.distance), 0.2*ones(1,length(dati.distance)));
    hold on;
    plot(1:length(dati.distance),(dati.distance));
    
end

if(plot1 == 11)
    load('vehicle_circular_crossed_3');
    figure;
    hold on;

    for i=1:10:length(dati.x_vehicle1)-1500
         axis([-2, 2, -2, 2]);
         plot(dati.x_vehicle1(i), dati.y_vehicle1(i), '.g');
         plot(dati.x_vehicle2(i), dati.y_vehicle2(i), '.b');
         plot(dati.x_vehicle3(i), dati.y_vehicle3(i), '.r');
         drawnow;
    end
    
    figure;
    plot(1:length(dati.distance), 0.1*ones(1,length(dati.distance)));
    hold on;
    plot(1:length(dati.distance),(dati.distance));
    
end
