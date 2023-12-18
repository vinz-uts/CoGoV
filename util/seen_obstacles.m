function [ob_seen] = seen_obstacles(x_pos, R_vision, oblist)
%ob_seen = SEEN_OBSTACLES(x_pos, R_vision, oblist)
%   Given a point x_pos, a radius R_vision and a list of obstalces
%   oblist compute and return the list of obstalces ob_seen visible
%   from a vehicle located at x_pos.
%   An obstalce ob is considered visible if at least the distance if
%   one vertex v from x_pos is above or equal the parameter R_vision
%
%  Authors: Vincenzo D'Angelo, Ayman El Qemmah, Franco Angelo Torchiaro
%  Credits: Alessandro Casavola, Francesco Tedesco
%  Copyright 2021 - CoGoV.

ob_seen = [];
best_d = 1000;
for i = 1:length(oblist)
    ob = oblist(i);
    vert = ob.vertices;
    d1 = norm(x_pos-vert(:,1));
    d2 = norm(x_pos-vert(:,2));
    d3 = norm(x_pos-vert(:,3));
    d4 = norm(x_pos-vert(:,4));

    if(min([d1 d2 d3 d4])==d1)
        if(d1 < R_vision)
            if(d1 < best_d)
                best_d = d1;
                ob_seen = ob;
            end
        end
    elseif(min([d1 d2 d3 d4]) == d2)
        if(d2 < R_vision)
            if(d2 < best_d)
                best_d = d2;
                ob_seen = ob;
            end
        end
    elseif(min([d1 d2 d3 d4]) == d3)
        if(d3 < R_vision)
            if(d3 < best_d)
                best_d = d3;
                ob_seen = ob;
            end
        end
    else
        if(d4 < R_vision)
            if(d4 < best_d)
                best_d = d4;
                ob_seen = ob;
            end
        end
    end

end


end