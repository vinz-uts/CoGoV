classdef Polar_trajectory_planner < handle
    
    properties
        fun % Trajectory function in polar coordinate
        xSamples % Samples along the x axis
        ySamples % Samples along the x axis
        center % Trajectory polar origin
        r_old % Old reference
        clockwise % If 1 the trajectory is traveled clockwise, otherwise counterclockwise
        tol % Tolerance ([m]) used to understand if the vehicle has reached the reference
        step % Variation in radians used to get the next reference
        counter % How much time the previous refence is still the same 
        p_old % Old position of vehicle 
        standstill % Dynamic value of old position freezing
    end
    
    methods
        function obj = Polar_trajectory_planner(xSamples, ySamples, step, tol, clockwise, standstill)
            obj = obj.computeParameterization(xSamples, ySamples);
            obj.counter  = 0;
            switch nargin
                case 3
                    obj.step = step;
                    obj.tol = 0.1;
                    obj.clockwise = true;
                    obj.standstill = 1000;
                case 4
                    obj.step = step;
                    obj.tol = tol;
                    obj.clockwise = true;
                    obj.standstill = 1000;
                case 5
                    if((not(clockwise == 1) && not(clockwise == -1)))
                        error('The value clockwise must be a logical value (true(1) or false(-1))!');
                    end
                    obj.step = step;
                    obj.tol = tol;
                    obj.clockwise = clockwise;
                    obj.standstill = 1000;
                case 6
                    if((not(clockwise == 1) && not(clockwise == -1)))
                        error('The value clockwise must be a logical value (true(1) or false(-1))!');
                    end
                    obj.step = step;
                    obj.tol = tol;
                    obj.clockwise = clockwise;
                    obj.standstill = standstill;
                otherwise
                    obj.step = 0.1;
                    obj.tol = 0.1;
                    obj.clockwise = true;
                    obj.standstill = 1000;
            end
        end
        function transform(obj, sF, xy)
            if(sF == 1)
                obj.center = obj.center + xy;
                return
            end
            x = sF*(obj.xSamples + xy(1));
            y = sF*(obj.ySamples + xy(2));
            obj.computeParameterization(x, y);
            obj.r_old = [];
        end
        function [r, theta] = compute_reference(obj, sys)
            % If the simulation has just started, the initial state is used
            if(isempty(sys.x))
                p = [sys.xi(1); sys.xi(2)];
            else % use the current state
                p = [sys.x(1, end); sys.x(2, end)];
            end
            % Initialize the previous reference if necessary
            if(isempty(obj.r_old))
                theta_v = obj.xy2polar(p(1), p(2));
                rho_ref = obj.evaluate(theta_v);
                obj.r_old = zeros(2, 1);
                [obj.r_old(1), obj.r_old(2)] = obj.polar2xy(rho_ref, theta_v);
            end
            if(isempty(obj.p_old))
                obj.p_old = p;
            end
            [rho_p, theta_p] = obj.xy2polar(p(1), p(2));
            [rho_r, theta_r] = obj.xy2polar(obj.r_old(1), obj.r_old(2));
            % The reference is not updated if the current one has not been reached
            if(norm(theta_p - theta_r) > obj.tol && obj.counter < obj.standstill)
                r = obj.r_old;
                theta = atan2(r(2)-p(2),r(1)-p(1));
                if(norm(p - obj.p_old) < 0.05)
                    obj.counter = obj.counter + 1; 
                end

            else
                theta_v = obj.xy2polar(obj.r_old(1), obj.r_old(2));
                theta_ref = theta_v + obj.clockwise*obj.step;
                if(theta_ref <= 0)
                    theta_ref = 2*pi+theta_ref;
                end
                rho_ref = obj.evaluate(theta_v + obj.step);
                r = zeros(2, 1);
                [r(1), r(2)] = obj.polar2xy(rho_ref, theta_ref);
                theta = atan2(r(2)-p(2),r(1)-p(1));
                obj.r_old = r;
                obj.counter = 0;
            end
            obj.p_old = p;
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
            plot(x, y, color);
            hold on;
            plot(obj.center(1), obj.center(2), strcat('o', color));
%             plot(obj.xSamples, obj.ySamples, 'x');
            hold off;
        end
    end
    methods (Access = private)
        function obj = computeParameterization(obj, xSamples, ySamples)
            minX = min(xSamples);
            maxX = max(xSamples);
            minY = min(ySamples);
            maxY = max(ySamples);
            obj.center = [(minX+maxX)/2; ySamples(1)];
            n = length(xSamples);
            theta = zeros(n, 1);
            rho = zeros(n, 1);
            for i = 1:n
                [rho(i), theta(i)] = obj.xy2polar(xSamples(i), ySamples(i));
            end
            theta(end) = 2*pi;
            obj.fun = pchip(theta, rho);
            obj.xSamples = xSamples;
            obj.ySamples = ySamples;
        end
        function varargout = xy2polar(obj, x, y)
            rho = norm([x; y] - obj.center);
            theta = atan2(y - obj.center(2), x - obj.center(1));
            if(theta < 0)
                theta = 2*pi + theta;
            end
            if(nargout == 1)
                varargout{1} = theta;
            else
                varargout{1} = rho;
                varargout{2} = theta;
            end
        end
        function [x, y] = polar2xy(obj, rho, theta)
            theta = mod(theta, 2*pi);
            x = rho.*(cos(theta)) + obj.center(1);
            y = rho.*(sin(theta)) + obj.center(2);
        end
    end
end