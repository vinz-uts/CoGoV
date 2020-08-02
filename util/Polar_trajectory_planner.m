classdef Polar_trajectory_planner < Planner
    % A planner for general but closed trajectories
    % Used to recreate a trajectory starting from a generic number of points. 
    % Samples must be supplied from right to left (clockwise). 
    % No need for first and last sample equality (curve closure).
    
    properties
        fun             % Trajectory function in polar coordinate
        xSamples        % Samples along the x axis
        ySamples        % Samples along the x axis
        center          % Trajectory polar origin
%         r_old           % Old reference
        clockwise       % If 1 the trajectory is traveled clockwise, otherwise counterclockwise
%         tol             % Tolerance ([m]) used to understand if the vehicle has reached the reference
        step            % Variation in radians used to get the next reference
%         counter         % How much time the previous refence is still the same 
%         p_old           % Old position of vehicle 
%         standstill      % Dynamic value of old position freezing
%         rec_tolerance   % How many times you tolerate to stay still
%         recovery_from_collision   % To differentiate between Circular or Rho recovery
        rho_step        % In case of Rho recovery this is the delta_rho to add
        sum             % In case of Rho recovery this is how many times delta_rho has been added
%         is_recovery_active % If the recovery reference has been reached or not
    end
    
    methods
        
        function obj = Polar_trajectory_planner(xSamples, ySamples, varargin)
            
            % Class constructor
            % xSamples - x coordinate of the samples
            % ySamples - y coordinate of the samples
            % Valid variable arguments:
            % step_size - distance (in radiants) of the next reference
            % tolerance - tolerance used to understand if the reference was
            %             reached (in rad)
            % clockwise - used to specify the direction of travel on the curve
            
            obj = obj@Planner(varargin{:});
            
            %%%%%% Input management %%%%%%
            if(~iscolumn(xSamples))
                xSamples = xSamples';
            end
            if(~iscolumn(ySamples))
                ySamples = ySamples';
            end
            if(~(xSamples(end)==xSamples(1)) || ~(ySamples(end)==ySamples(1)))
                xSamples = [xSamples;xSamples(1)];
                ySamples = [ySamples;ySamples(1)];
            end
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            %%%%%% Computation of the interpolation function
            obj = obj.computeParameterization(xSamples, ySamples);
            %%%%%
            
            %%%%%% Default values %%%%%%%%%
            obj.step = 0.1;
            obj.tol = 0.1;
            obj.clockwise = 1;
            obj.rho_step = 0.011;
            %%%%%%%%%%%%
            
            %%%%%%% Variable arguments management %%%%%%%
            validnames = {'step_size', 'tolerance', 'clockwise', 'rho_step'};
            
            nargs = length(varargin);
            params = varargin(1:2:nargs);   values = varargin(2:2:nargs);
            
            for name = params
                pos = strcmp(name{:}, params);
                switch name{:}
                    case validnames{1}
                        obj.step = values{pos};
                    case validnames{2}
                        obj.tol = values{pos};
                    case validnames{3}
                        if(not(islogical(values{pos})))
                            warning('clockwise must be a logical value (true for clockwise) default (true) used.');
                            values{pos} = true;
                        end
                        if(values{pos})
                            obj.clockwise = 1;
                        else
                            obj.clockwise = -1;
                        end
                    case validnames{4}
                        obj.rho_step =  values{pos};
                end
            end   
            %%%%%%%%%%%%%%%
        end % end of constructor
        
        
        function transform(obj, sF, xy)
            % Input:
            % sf - scale factor used to enlarge or reduce the trajectory
            x = sF*(obj.xSamples)+ xy(1);
            y = sF*(obj.ySamples)+ xy(2);
            %%% New interpolation function 
            obj.computeParameterization(x, y);
            %%%%%
            
            %%% adjustment of paramenters
            obj.r_old = [];
            obj.sum = 0;
            obj.counter  = 0;
            %%%%%%%
        end
        
        function r = initialize_old_reference(obj, p)
            theta_v = obj.xy2polar(p(1), p(2));
            rho_ref = obj.evaluate(theta_v);
            r = zeros(2, 1);
            [r(1), r(2)] = obj.polar2xy(rho_ref, theta_v);
        end
        
        function [r, theta] = compute_referecence_when_not_reached(obj, p)
            r = obj.r_old;
            theta = atan2(r(2) - p(2),r(1) - p(1));
        end
        
        function res = reference_reached(obj, p)
            theta_p = obj.xy2polar(p(1), p(2));
            theta_r = obj.xy2polar(obj.r_old(1), obj.r_old(2));
            res = norm(theta_p - theta_r) <= obj.tol;
            
%             res = norm(p(1)-obj.r_old(1)) <= obj.tol || norm(p(2)-obj.r_old(2)) <= obj.tol ;
        end
        
        function [r, theta] = compute_standard_reference(obj, p)
            % Compute old reference in polar coordinate
            theta_v = obj.xy2polar(obj.r_old(1), obj.r_old(2));
            
            % move clockwise or counterclockwise
            theta_ref = theta_v + obj.clockwise*obj.step;
            if(theta_ref <= 0)
                theta_ref = 2*pi + theta_ref;
            end
            rho_ref = obj.evaluate(theta_ref);
            
            % express the new reference in cartesian coordinate
            r = zeros(2, 1);
            [r(1), r(2)] = obj.polar2xy(rho_ref, theta_ref);
            theta = atan2(r(2)-p(2),r(1)-p(1));
            obj.sum = 0;
        end
        
        function [r, theta] = compute_alternative_recovery(obj, sys, xa)
            p = sys.ctrl_sys.sys.xi(1:2);
            
            obj.sum = obj.sum + obj.rho_step;
            theta_v = obj.xy2polar(obj.r_old(1), obj.r_old(2));
            theta_ref = theta_v;
            rho_ref = obj.evaluate(theta_ref);
            r = zeros(2, 1);
            [r(1), r(2)] = obj.polar2xy(rho_ref + obj.sum, theta_ref);
            theta = atan2(r(2)-p(2),r(1)-p(1));
        end
          
        function rho = evaluate(obj, theta)
            rho = (ppval(mod(theta, 2*pi), obj.fun));
        end
        
        function plot(obj, color)
            if(nargin < 2)
                color = 'k';
            end
            theta = 0:0.01:2*pi + 0.01;
            rho = obj.evaluate(theta);
            [x, y] = obj.polar2xy(rho, theta);
            plot(x, y, strcat(':', color));
            tf = ishold;
            hold on;
%             plot(obj.center(1), obj.center(2), strcat('o', color));
%             plot(obj.xSamples, obj.ySamples, 'x');
            if(not(tf))
                hold off;
            end
        end
        
    end % end public methods
    
    methods (Access = private)
        
        function obj = computeParameterization(obj, xSamples, ySamples)
            % Compute trajectory from samplings interpolation
            % Center or origin of the polar coordinates system
            minX = min(xSamples);
            maxX = max(xSamples);
            obj.center = [(minX+maxX)/2; ySamples(1)];
            n = length(xSamples);
            
            % Traslation to polar coordinates
            theta = zeros(n, 1);
            rho = zeros(n, 1);
            for i = 1:n
                [rho(i), theta(i)] = obj.xy2polar(xSamples(i), ySamples(i));
            end
            theta(end) = 2*pi;
            
            % interpolation usin pchip function
            obj.fun = pchip(theta, rho);
            obj.xSamples = xSamples;
            obj.ySamples = ySamples;
            
        end
        
        function varargout = xy2polar(obj, x, y)
            % From cartesion to polar
            rho = norm([x; y] - obj.center);
            theta = atan2(y - obj.center(2), x - obj.center(1));
            
            if(theta < 0)
                theta = 2*pi + theta;
            end
            
            % Output management
            if(nargout == 1)
                varargout{1} = theta;
            else
                varargout{1} = rho;
                varargout{2} = theta;
            end
        end
        
        function [x, y] = polar2xy(obj, rho, theta)
            % From polar to cartesian
            theta = mod(theta, 2*pi);
            x = rho.*(cos(theta)) + obj.center(1);
            y = rho.*(sin(theta)) + obj.center(2);
        end
        
    end % end private methods
    
end % end of class
