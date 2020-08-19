classdef BorderPlanner
    
    properties
        p_old;
        step;
        m;
        verso;
        limits;
        r_old;
        tol; 
        offsetline; 
        
    end
    
    methods
        function obj = BorderPlanner(step, m, limits, tol)
            obj.step = step;
            obj.p_old = [];
            obj.r_old = [];
            obj.m = m;
            obj.offsetline = [];
            if(nargin <= 3)  
                obj.tol = 0.01;
            else
                obj.tol = tol;
            end
            obj.limits = struct('x_l', limits(1), 'x_r', limits(2), 'y_t', limits(3), 'y_b', limits(4));
        end
        
        function [r, obj] = compute_reference(obj, sys)
           
            
            if(isempty(sys.x))
                p = [sys.xi(1); sys.xi(2)];
                
            else
                p = [sys.x(1, end); sys.x(2, end)];
            end
            
            
            if(not(isempty(obj.r_old)) && norm(p - obj.r_old) > obj.tol)
                r = obj.r_old;
                return;
            end
            
            if(isempty(obj.r_old))
                obj.r_old = p;
            end
            
            if(length(sys.x) > 60 && not(isempty(obj.p_old)))
                if(norm(p(1) - obj.limits.x_l) < obj.tol || norm(p(1) - obj.limits.x_r) < obj.tol)
                    %alpha = atan(obj.m);
                    m_new = - obj.m;    
                    obj.step = - obj.step;
                    obj.m = m_new;
                    obj.offsetline = p(2) - obj.m*p(1); 
                    
                    
                elseif(norm(p(2) - obj.limits.y_t) < obj.tol || norm(p(2) - obj.limits.y_b) < obj.tol)
%                     alpha = atan(obj.m);
%                     m_new = tan(pi - alpha);
                    m_new = - obj.m; 
                    obj.m = m_new;
                    obj.offsetline = p(2) - obj.m*p(1); 
                end
            end
            
            if((isempty(obj.offsetline)))
                obj.offsetline = p(2) - obj.m*p(1); 
            end
            
            obj.p_old = p;
%             q = obj.offsetline; 
            
%             rect1 = @(x) (obj.m*x + q);
           
                
            rect = @(C, x) (C(2) + obj.m*(x - C(1)));
            r = [p(1) + obj.step, rect(obj.r_old, p(1) + obj.step)]';

%             x_as = r(1)-0.3:0.1:r(1)+0.3;
%             figure(1)
%             plot(x_as, rect(obj.r_old,x_as));

            
            if(r(1) > obj.limits.x_r)
                r(1) = obj.limits.x_r ;
                r(2) = rect(obj.r_old,r(1));
            elseif(r(1) < obj.limits.x_l)
                r(1) = obj.limits.x_l ;
                r(2) = rect(obj.r_old,r(1));
                
            elseif(r(2) < obj.limits.y_b)
                r(2) = obj.limits.y_b ;
            elseif(r(2) > obj.limits.y_t)
                r(2) = obj.limits.y_t ;
            end
            
            obj.r_old = r;
            
            
        end
    end
    

end


