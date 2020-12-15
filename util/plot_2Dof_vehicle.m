function  plot_2Dof_vehicle(vehicle, r, d_min, varargin)
    
    hold_state = ishold;
    
    plot_pair = {'Color',           'k';
                 'LineWidth',       0.8;
                 'Reference_style', 'o';
                 'CG_ref_style'     'x';
                 'D_min_style'      '-';
                 'Traj_style'       '-';
                 'MarkerSize',      10;
                 'MarkerStyle'      'o';
                 'RangeAxis'         []}; 
    
    nargs = length(varargin);
    params = varargin(1:2:nargs);   values = varargin(2:2:nargs);

    for name = params
        pos = strcmp(name{:}, params);
        switch name{:}
            case 'Color'
                plot_pair{1, 2} = values{pos};
            case 'LineWidth'
                plot_pair{2, 2} = values{pos};
            case 'Reference_style'
                plot_pair{3, 2} = values{pos};
            case 'CG_ref_style'
                plot_pair{4, 2} = values{pos};
            case 'D_min_style'
                plot_pair{5, 2} = values{pos};
            case 'Traj_style'
                plot_pair{6, 2} = values{pos};
            case 'MarkerSize'
                plot_pair{7, 2} = values{pos};
            case 'MarkerStyle'
                plot_pair{8, 2} = values{pos};
            case 'RangeAxis' 
                plot_pair{9,2} = values{pos};
        end
    end
    
    
    % Trajectory plot
    plot(vehicle.ctrl_sys.sys.x(1,:), vehicle.ctrl_sys.sys.x(2,:), strcat(plot_pair{6, 2}, plot_pair{1, 2}),...
         plot_pair{2, 1}, plot_pair{2, 2});
    hold on;
    if(not(isempty(plot_pair{9,2})))
        axis(plot_pair{9,2});
    else
        axis equal;
    end
   
    % Vehicle current position plot
    plot(vehicle.ctrl_sys.sys.x(1,end), vehicle.ctrl_sys.sys.x(2,end), strcat(plot_pair{8, 2}, plot_pair{1, 2}),...
        'MarkerFaceColor', plot_pair{1, 2}, plot_pair{7, 1}, plot_pair{7, 2});
    
    if(not(isempty(d_min)))
        theta = 0:0.1:2*pi+0.1;
        x1 = vehicle.ctrl_sys.sys.x(1,end) + d_min*cos(theta);
        y1 = vehicle.ctrl_sys.sys.x(2,end) + d_min*sin(theta);
        plot(x1,y1, strcat(plot_pair{6, 2}, plot_pair{1, 2}));
    end
    
    if(not(isempty(r)))
        plot(r(1), r(2), strcat(plot_pair{3, 2}, plot_pair{1, 2}));
    end
    
    if(not(isempty(vehicle.g)))
        plot(vehicle.g(1), vehicle.g(2), strcat(plot_pair{4, 2}, plot_pair{1, 2}));
    end
    
    % Restore previous hold state
    if(hold_state)
        hold on;
    else
        hold off;
    end
end

