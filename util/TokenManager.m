classdef TokenManager < handle
    %TOKENMANAGER Concrete class for token heuristics
    %   Solve the turns determination problem (graph coloring problem) 
    %   in a dystributed fashion by applying the token heuristics. 
    %   Each vehicle is equipped with a TokenManager instance to keep track
    %   of the available tokens.
    %   When using the token heuristics each vehicle generate a token whenever
    %   a new connection is created. The vehicle requesting the connection
    %   release the token giving it to the vehicle that has accepted the 
    %   connection.
    %   A vehicle is able to compute a new reference only when it possesses
    %   all the tokens associated to their connections.
    %   At the end of the computation phase all the tokens should be
    %   released and given to the corresponding neighboring vehicles.
    %
    %tm = TOKENMANAGER(id) create a token manager with IDentifier id
    %   the id should be the same IDentifier of the vehicle that whose the
    %   particular instance of the token manager belongs to.
    %      
    %
    %  Authors: Vincenzo D'Angelo, Ayman El Qemmah, Franco Angelo Torchiaro
    %  Credits: Alessandro Casavola, Francesco Tedesco
    %  Copyright 2021 - CoGoV.
    
    properties
        id          % IDentifier of the vehicle associated to the particular instance.
        connections % Map containing the available connection at each time.
    end
    
    methods
        function obj = TokenManager(id)
            %TOKENMANAGER(id) Construct an instance of the class
            %   Given an IDentifier id build a token manager instance with
            %   empty connections
            obj.id = id;
            obj.connections = containers.Map('keyType', 'double',...
                'valueType', 'any');
        end

        function con = getConnections(obj)
            %con = GETCONNECTIONS() get a list of all the active
            %   connections. A connection is the couple defined by the id
            %   of a vehicle and the boolen token. The token variable is
            %   true if the associated token belogs to this instance.
            keys = obj.connections.keys();
            con = [];
            for i = 1:length(keys)
                con = [con, keys{i}];
            end
        end
        
        function generateLinks(obj, adj_matrix)
            % GENERATELINKS(adj_matrix) generate multiple connections
            % Generate all the connections found in the the adjacency matrix 
            % adj_matrix.
            for i = 1:size(adj_matrix, 1)
                if(adj_matrix(obj.id, i) == 1)
                    obj.create_connection(i)
                end
            end
        end

        function create_connection(obj, id)
            %CREATE_CONNECTION(id) generate a connection with vehicle id
            %   Generate a conncetion (id, token) using the vehicle identifier id.
            %   The connection is generated with token = false.
            obj.connections(id) = false;
        end
        
        function remove_connection(obj, id)
            %REMOVE_CONNECTION(id) Delete the connection with the vehiclr id
            %   If a connection with vehicle id does exist, remove it.
            obj.connections.remove(id);
        end
        
        function receive_token(obj, id)
            %RECEIVE_TOKEN(id) Receive token from the vehicle id
            %   If a connection with the vehicle id does exists, set the
            %   token value to true.
            obj.connections(id) = true;
        end
        
        function release_token(obj, id)
            %RELEASE_TOKEN(id) Release token to vehicle id
            %   If a connection with the vehicle id does exists, set the
            %   token value to false. This method does not set the token
            %   variable of the vehicle id.
            obj.connections(id) = false;
        end
        
        function release_all_tokens(obj)
            %RELEASE_ALL_TOKENS Release all the tokens received so far
            %   Set the token variable to false for all the connection
            %   created so far.
            keys = obj.connections.keys();
            for i = 1:length(keys)
                obj.connections(keys{i}) = false;
            end
        end
        
        function answer = isMyTurn(obj)
            %answer = ISMYTURN Check if the vehicle can compute its reference
            %   Check if the vehicle can compute its reference. Return true
            %   if all the token stored so far are true (the vehicle holds 
            %   all the token associated to its connections).
            keys = obj.connections.keys();
            for i = 1:length(keys)
                if(not(obj.connections(keys{i})))
                    answer = false;
                    return;
                end
            end
            answer = true;
        end

        function tokens = getNumTokens(obj)
            %tokens = GETNUMCONNECTIONS Get the total number of connections
            %   Get the total number of connection created so far. All the
            %   connections are considered dispite the value of the token
            %   variable.
            keys = obj.connections.keys();
            tokens = 0;
            for i = 1:length(keys)
                if(obj.connections(keys{i}) > 0)
                    tokens = tokens + 1;
                end
            end
        end
        
        function toString(obj)
            %TOSTRING Print a string representation of this class
            %   Dispay a string representation of this class showing the
            %   total number of connection, the ids of the neighbors, the
            %   and the tokens values.
            to_disp = "";
            keys = obj.connections.keys();
            for i = 1:length(keys)
                to_disp = to_disp + sprintf("%d <---- %d ----> %d\n",...
                    obj.id, obj.connections(keys{i}), keys{i});
            end
            disp(to_disp);
        end
    end
end

