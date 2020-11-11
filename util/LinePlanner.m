classdef LinePlanner < Planner
    %UNTITLED Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        points   % A vector containing all points that have to be reached
        point_iterator % Index of curret point to be reached
        step            % Variation in radians used to get the next reference
        line
        direction
    end
    
    methods
        function obj = LinePlanner(references, varargin)
            %UNTITLED Construct an instance of this class
            %   Detailed explanation goes here
            obj = obj@Planner(varargin{:});
            obj.points = references;
            obj.point_iterator = 1;
            obj.step = 0.5;
            obj.tol = 0.2;
        end
        
         function r = initialize_old_reference(obj, p)
             r = obj.points(obj.point_iterator:obj.point_iterator+1)';
             if(r(1) - p(1) > 0)
                 obj.direction = 1;
             else
                 obj.direction = -1;
             end
             obj.line = @(x) ((x - p(1))/(r(1) - p(1))*(r(2) - p(2)) + p(2));
             r(1) = p(1) + obj.step*obj.direction;
             r(2) = obj.line(r(1));
        end
        
        function [r, theta] = compute_referecence_when_not_reached(obj, p)
            r = obj.r_old;
            theta = atan2(r(2) - p(2),r(1) - p(1));
        end
        
        function res = reference_reached(obj, p)
            res = norm(p(1) - obj.r_old(1)) < obj.tol || norm(p(2) - obj.r_old(2)) < obj.tol;
        end
        
        function [r, theta] = compute_standard_reference(obj, p)
           ref = obj.points(obj.point_iterator:obj.point_iterator+1)';
           if(norm(p - ref) > obj.tol)
               r(1) = p(1) + obj.step*obj.direction;
               r(2) = obj.line(r(1));
               r = r';
               if(obj.direction < 0)
                   if(r(1) < ref(1))
                       r = ref;
                   end
               else
                   if(r(1) > ref(1))
                       r = ref;
                   end
               end
           else
               obj.point_iterator = obj.point_iterator + 2;
               
               if(obj.point_iterator >= length(obj.points))
                   obj.point_iterator = 1;
               end
               
               r = obj.initialize_old_reference(p);
               hold on;
               plot(-2:0.1:2.5, obj.line(-2:0.1:2.5))

           end
           theta = atan2(r(2) - p(2),r(1) - p(1));
           
        end
    end
end

