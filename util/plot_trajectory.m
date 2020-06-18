function plot_trajectory(x,y,th,N,d)
    %% PLOT_TRAJECTORY - plot trajectory with oriented points
    %  Plot the trajectory of [x,y,ϑ] data with sparse oriented points.
    %  N := points density
    %  d := point dimension
    %  Usage:
    %  >> plot_trajectory(x,y,th)
    %  >> plot_trajectory(x,y,th,N)
    %  >> plot_trajectory(x,y,th,N,d)
    
    N_ = 10; % points density
    d_ = 0.01; % triangle dimension
    if nargin > 3
        N_ = N;
    end
    if nargin > 4
        d_ = d;
    end
    
    plot(x,y);  hold on; % plot trajectory
    
    % plot points orientation
    for i = 1:N_:length(x)
       p_i.x = [x(i)+d_*cos(th(i)); x(i)-d_*cos(pi/3-th(i)); x(i)-d_*sin(pi/3-th(i)); x(i)+d_*cos(th(i))];
       p_i.y = [y(i)+d_*sin(th(i)); y(i)+d_*sin(pi/3-th(i)); y(i)-d_*cos(pi/3-th(i)); y(i)+d_*sin(th(i))];
       plot(p_i.x,p_i.y,'r');
    end
