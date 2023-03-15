classdef Trajectory_tracker
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        fun_tx          % Trend of x variable over time
        fun_ty          % Trend of y variable over time
        radius
        time_interval
    end
    
    methods
        function obj = Trajectory_tracker(xSamples, ySamples, time_interval, sF, xy, radius)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            if(nargin < 4)
                sF = 1;
                xy = [0,0]';
                radius = 100;
            elseif(nargin < 5)
                xy = [0,0]';
                radius = 100;
            elseif(nargin < 6)
                radius = 100;
            end
            xSamples = sF*(xSamples)+ xy(1);
            ySamples = sF*(ySamples)+ xy(2);
            obj.radius = radius;
            obj.fun_tx =  spline(time_interval, [0;xSamples;0]);
            obj.fun_ty =  spline(time_interval, [0;ySamples;0]);
            obj.time_interval = time_interval;
        end
        
        
        function [r, theta] = compute_trajectory_reference(obj,t,g,p)
            r = zeros(2,1);
            r(1) =  ppval(t, obj.fun_tx);
            r(2) =  ppval(t, obj.fun_ty);
            if(norm(p - r) > obj.radius)
                r = inner_reference(obj, p, r);
            end
             if(not(iscolumn(r)))
                r = r';
            end
            theta = atan2(g(2) - p(2),g(1) - p(1));
        end
        
        function ris = inner_reference(obj, p, r) % inherited abstract method
            if(r(1) - p(1) > 0)
                direction = 1;
            else
                direction = -1;
            end
            
            if((r(1) - p(1)) == 0)
                slope = inf;
                intercept = p(1);
                if(r(2) - p(2) < 0)
                    direction = 1;
                else
                    direction = -1;
                end
            else
                slope = (r(2) - p(2))/(r(1) - p(1));
                intercept = -slope*p(1) + p(2);
            end
            
            [xout,yout] = linecirc(slope, intercept,p(1), p(2), obj.radius);
            % Choose the right poit to use as an intermediate reference
            if(slope == inf)
                if(direction < 0)
                    if(yout(1) > yout(2))
                        ris(1) = xout(1);
                        ris(2) = yout(1);
                    else
                        ris(1) = xout(2);
                        ris(2) = yout(2);
                    end
                else
                    if(yout(1) < yout(2))
                        ris(1) = xout(1);
                        ris(2) = yout(1);
                    else
                        ris(1) = xout(2);
                        ris(2) = yout(2);
                    end
                end
            else
                if(direction < 0)
                    if(xout(1) < xout(2))
                        ris(1) = xout(1);
                        ris(2) = yout(1);
                    else
                        ris(1) = xout(2);
                        ris(2) = yout(2);
                    end
                else
                    if(xout(1) > xout(2))
                        ris(1) = xout(1);
                        ris(2) = yout(1);
                    else
                        ris(1) = xout(2);
                        ris(2) = yout(2);
                    end
                end
            end
        end
        
        function plot(obj, color)
            if(nargin < 2)
                color = 'k';
            end
            if(obj.time_interval(1)<obj.time_interval(end))
                t = obj.time_interval(1):0.1:obj.time_interval(end);
            else
                t = obj.time_interval(end):0.1:obj.time_interval(1);
            end
            %             t = obj.time_interval(1):0.1:obj.time_interval(end);
            x = ppval(t, obj.fun_tx);
            y = ppval(t, obj.fun_ty);
            plot(x,y,strcat(color, '--'));
        end
        
        function plot_speed(obj, color)
            if(nargin < 2)
                color = 'k';
            end
            if(obj.time_interval(1)<obj.time_interval(end))
                t = obj.time_interval(1):0.1:obj.time_interval(end);
            else
                t = obj.time_interval(end):0.1:obj.time_interval(1);
            end
            %             t = obj.time_interval(1):0.1:obj.time_interval(end);
            funx = fnder(obj.fun_tx,1);
            funy = fnder(obj.fun_ty,1);
            x = ppval(t, funx);
            y = ppval(t, funy);
            subplot(2, 1, 1);
            plot(t,x,color);
            subplot(2,1,2);
            plot(t,y,color);
        end
        
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
    end
end

