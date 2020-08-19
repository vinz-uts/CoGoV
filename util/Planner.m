classdef Planner
    
    properties
        p_old;
        step;
        m;
        verso;
    end
    
    methods
        function obj = Planner(step, m)
            obj.step = step;
            obj.p_old = [];
            obj.m = m;
        end
        
        function [r, obj] = compute_reference(obj, sys)
           if(isempty(sys.x))
                p = [sys.xi(1); sys.xi(2)];
           else
                p = [sys.x(1, end); sys.x(2, end)];
           end
           
           if(length(sys.x) > 60 && not(isempty(obj.p_old)))
               if(norm(p(2) - obj.p_old(2)) < 0.001) % da verificare la lunghezza
                    alpha = atan(obj.m);
                    m_new = tan(pi - alpha);
                    
                    q = p(2) - m_new*p(1);

                    rect = @(x) (m_new*x + q);
                    obj.p_old = p;
                    
                    r = [p(1) + obj.step, rect(p(1) + obj.step)]';
                    obj.m = m_new;
                   return;
               end
               
               if(norm(p(1) - obj.p_old(1)) < 0.001) % da verificare la lunghezza
                    alpha = atan(obj.m);
                    m_new = tan(pi - alpha);
                    
                    q = p(2) - m_new*p(1);

                    rect = @(x) (m_new*x + q);
                    obj.p_old = p;
                    
                    obj.step = - obj.step;
                    r = [p(1) + obj.step, rect(p(1) + obj.step)]';
                    obj.m = m_new;
                   return;
               end
           end
              
            q = p(2) - obj.m*p(1);
           
            rect = @(x) (obj.m*x + q);
            obj.p_old = p;
            
            r = [p(1) + obj.step, rect(p(1) + obj.step)]';
            
        end
    end
end

