function connected = isConnected(adj_matrix)
%% ISCONNECTED - Check if a graph is connected
%  Given the adjacency matrix of a nonoriented graph, the function check if
%  the graph is totally connected.
    g = graph(adj_matrix);
    idx = conncomp(g);
    connected = all(idx==1);
end

