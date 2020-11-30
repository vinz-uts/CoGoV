classdef Phisical_Layer < handle
    %Phisical_Layer Class usefull for implementing communication strategies
    %   Detailed explanation goes here
    
    properties
        droprate
        c_matrix_sent
        c_matrix_recived
        c_matrix_bytes_sent
        c_matrix_bytes_recived
        packets
        packets_recived
        maximum_distance
        positions
        verbose
        N
        t
    end
    
    methods
        function obj = Phisical_Layer(N, maximum_distance, droprate, queue_length)
            obj.droprate = droprate;
            obj.c_matrix_sent = zeros(N);
            obj.c_matrix_recived = zeros(N);
            obj.c_matrix_bytes_sent = zeros(N);
            obj.c_matrix_bytes_recived = zeros(N);
            obj.packets = cell(N, queue_length);
            obj.packets_recived = zeros(N, 1);
            obj.maximum_distance =  maximum_distance;
            obj.positions = zeros(2, N);
            obj.verbose = false;
            obj.N = N;
            obj.t = 0;
        end
        
        function send_packet(obj, id_trasmitter, id_reciver, packet)
            %SEND_PACKET Summary of this method goes here
            %   Detailed explanation goes here
            
            if(obj.is_communication_avalable(id_trasmitter, id_reciver))
                if(not(obj.is_packet_dropped()))
                    if(obj.packets_recived(id_reciver) > 0)
                        if(obj.packets_recived(id_reciver) < length(obj.packets{id_reciver}))
                            obj.add_packet(id_trasmitter, id_reciver, packet);
                        else
                            if(obj.verbose)
                                fprintf('Packet from %d to %d dropped beacause the reciver queue is full.\n', id_trasmitter, id_reciver);
                            end
                        end
                    else
                        obj.add_packet(id_trasmitter, id_reciver, packet);
                    end
                else
                    if(obj.verbose)
                        fprintf('Packet from %d to %d was lost.\n', id_trasmitter, id_reciver);
                    end
                end
            else
                if(obj.verbose)
                    fprintf('Packet from %d to %d was lost because the two vehicles are too far apart.\n', id_trasmitter, id_reciver);
                end
            end
            obj.c_matrix_sent(id_trasmitter,  id_reciver) = obj.c_matrix_sent(id_trasmitter,  id_reciver) + 1;
            s = whos('packet');
            obj.c_matrix_bytes_sent(id_trasmitter,  id_reciver) = obj.c_matrix_bytes_sent(id_trasmitter,  id_reciver) + s.bytes;
        end
        
        function packet = get_packet(obj, id_reciver)
            if(obj.packets_recived(id_reciver) > 0)
                packet = obj.packets{id_reciver, obj.packets_recived(id_reciver)};
                obj.packets{id_reciver, obj.packets_recived(id_reciver)} = [];
                obj.packets_recived(id_reciver) = obj.packets_recived(id_reciver) - 1;
            else
               packet = []; 
            end
            obj.c_matrix_recived(packet.id_trasmitter,  id_reciver) = obj.c_matrix_recived(packet.id_trasmitter,  id_reciver) + 1;
            data = packet.data;
            s = whos('data');
            obj.c_matrix_bytes_recived(packet.id_trasmitter,  id_reciver) = obj.c_matrix_bytes_recived(packet.id_trasmitter,  id_reciver) + s.bytes;
            packet = data;
        end
        
        function res = look_for_packets(obj, id_reciver)
            res = obj.packets_recived(id_reciver) > 0;
        end
        
        function update(obj, vehicles, t)
            for i = 1:length(vehicles)
                obj.positions(:, i) = vehicles{i}.ctrl_sys.sys.xi(1:2);
            end
            obj.t = t;
        end
        
        function plot(obj)
            for i = 1:obj.N
                for j = i:obj.N
                    if(not(i == j) && obj.c_matrix_sent(i, j) > 0)
                        x = (obj.positions(1, i) + obj.positions(1, j))/2;
                        y = (obj.positions(2, i) + obj.positions(2, j))/2;
                        plot([obj.positions(1, i); obj.positions(1, j)], [obj.positions(2, i); obj.positions(2, j)],':k');
                        text(x, y, sprintf('%d', obj.c_matrix_sent(i, j)));
                    end
                end
            end
        end
        
        function out = get_total_packets_sent(obj)
            At = obj.c_matrix_sent.';
            m  = (1:size(At,1)).' >= (1:size(At,2));
            v  = At(m);
            out = sum(v)*2;
        end
        
        function out = get_total_packets_recived(obj)
            At = obj.c_matrix_recived.';
            m  = (1:size(At,1)).' >= (1:size(At,2));
            v  = At(m);
            out = sum(v)*2;
        end
        
        function out = get_total_packets_lost(obj)
            At = (obj.c_matrix_sent - obj.c_matrix_recived).';
            m  = (1:size(At,1)).' >= (1:size(At,2));
            v  = At(m);
            out = sum(v)*2;
        end
        
        function out = get_total_bytes_sent(obj)
            At = obj.c_matrix_bytes_sent.';
            m  = (1:size(At,1)).' >= (1:size(At,2));
            v  = At(m);
            out = sum(v)*2;
        end
        
        function out = get_total_bytes_recived(obj)
            At = obj.c_matrix_bytes_recived.';
            m  = (1:size(At,1)).' >= (1:size(At,2));
            v  = At(m);
            out = sum(v)*2;
        end
        
        function out = get_total_bytes_lost(obj)
            At = (obj.c_matrix_bytes_sent - obj.c_matrix_bytes_recived).';
            m  = (1:size(At,1)).' >= (1:size(At,2));
            v  = At(m);
            out = sum(v)*2;
        end
        
        function out = get_sent_packets_per_second(obj)
            num_p = obj.get_total_packets_sent();
            out = (num_p+0.0)/(obj.t + 0.0);
        end
        
        function out = get_sent_bytes_per_second(obj)
            num_p = obj.get_total_bytes_sent();
            out = (num_p+0.0)/(obj.t + 0.0);
        end
    end
    
    methods (Access = private)
        function add_packet(obj, id_trasmitter, id_reciver, data)
            packet = struct('id_trasmitter', id_trasmitter, 'data', data);
            obj.packets_recived(id_reciver) = obj.packets_recived(id_reciver) + 1;
            obj.packets{id_reciver, obj.packets_recived(id_reciver)} = packet;
        end
        
        function res = is_packet_dropped(obj)
            res = rand() < obj.droprate;
        end
        
        function res = is_communication_avalable(obj, id_trasmitter, id_reciver)
            res = norm(obj.positions(:, id_trasmitter) - obj.positions(:, id_reciver)) < obj.maximum_distance;
        end
    end
end

