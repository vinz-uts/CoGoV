function updateted_colors = update_color(vehicles, possible_colors, adj_matrix)
%updated_colors = UPDATE_COLOR(vehicle, possible_colors, adj_matrix)
%   Given a collection of vehicles, the list of available colors
%   possible_colors and the adjacency matrix adj_matrix of the 
%   communication graph compute and return the colors updateted_colors to 
%   assign to each vehicle by solving a graph coloring problem.
%  
%   In this way, it is possible to solve the turn determination problem to
%   arrange vehicles into Turn (colors).
%
%  Authors: Vincenzo D'Angelo, Ayman El Qemmah, Franco Angelo Torchiaro
%  Credits: Alessandro Casavola, Francesco Tedesco
%  Copyright 2021 - CoGoV.
 std_color = 'k';
 colors = intvar(length(vehicles), 1);
 cnstr = [];
 for i = 1:length(vehicles)
     cnstr = [cnstr colors(i) >= 0];
     for j = i:length(vehicles)
        if(adj_matrix(i, j) == 1)
            cnstr = [cnstr abs(colors(i) - colors(j)) >= 0.5];
        end
     end
 end
 options = sdpsettings('verbose', 0, 'solver', 'gurobi');
 obj_fun = (colors)'*(colors);
 ris = optimize(cnstr, obj_fun, options);
 
 updateted_colors = possible_colors((double(colors)+1));
 
end

