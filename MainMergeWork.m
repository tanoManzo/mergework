close all; clear all; clc
tic

rng(100)
%% load data from file
% list of events
%   * 1 id: node identifier;
%   * 2 id_cell: cell identifier;
%   * 3 sim_time: simulation time;
%   * 4 node_speed: node's speed;
%   * 5 GPS_X: node position on sumo x-axis;
%   * 5 GPS_Y: node position on sumo y-axis;
fileID = fopen('data/test/node_info0.txt','r');
data_read_info = textscan(fileID,'%f%s%f%f%f%f','Delimiter',' ');
fclose(fileID);

%% get data
temporal_node_ids=data_read_info{1,1};
temporal_cell_ids=data_read_info{1,2};
temporal_simTime=data_read_info{1,3};
temporal_node_speed=data_read_info{1,4};


node_ids=unique(temporal_node_ids);
time=unique(temporal_simTime);
cell_ids=unique(temporal_cell_ids);

%List of neighbor per node (key=node id, value=neighbor list)
ids_map_contact = containers.Map(node_ids,cell(length(node_ids),1));

%node state map (key=node id, value=node state [requester(-1), neutral(0), producer(1)] )
ids_map_state = containers.Map(node_ids,zeros(length(node_ids),1));

%node content map (key=node id, value=true [it has the content] , false [otherwise] )
ids_map_PIT = containers.Map(node_ids,zeros(length(node_ids),1));

%node content map (key=node id, value=true [it has the content] , false [otherwise] )
ids_map_content = containers.Map(node_ids,zeros(length(node_ids),1));

%cell_probabities (key=cell id, value=cell prob. [replication(0-1), caching(0-1)])
cell_ids_prob_replication=containers.Map(cell_ids,zeros(length(cell_ids),1));
cell_ids_prob_caching=containers.Map(cell_ids,zeros(length(cell_ids),1));

%cell_requester distribution (key=cell id, value=cell prob. to be a
%requester)
cell_ids_prob_requester=containers.Map(cell_ids,zeros(length(cell_ids),1));


%% initialization
%replication caching and requester
for ii=1:length(cell_ids)
    cell_ids_prob_replication(cell_ids{ii})=rand;
    cell_ids_prob_caching(cell_ids{ii})=rand;
    cell_ids_prob_requester(cell_ids{ii})=rand;
end

%producer (t==1 or t==0)
flag=1;
jj=1;
PROB_PRODUCER=0.1;
producer_num=0;
while flag==1
    if(temporal_simTime(jj)~=1)
        flag=0;
    else
        
        if(rand<PROB_PRODUCER)
            %node becomes producer
            ids_map_state(temporal_node_ids(jj))=1;
            ids_map_content(temporal_node_ids(jj))=1;
            producer_num=producer_num+1;
        end
        jj=jj+1;
        
    end
    
end


%% contact event
%load list of contact from file
fileID = fopen('data/test/node_contact0.txt','r');
data_read_contact =textscan(fileID, '%s', 'CollectOutput'  ...
    ,   true, 'Delimiter', ';'  );
fclose(fileID);

sequence_contacts=data_read_contact{1,1};


requester_sadisfied=0;
requester_overall=0;
%update contact list
for ii=1:10000%length(temporal_simTime)
    node_id=temporal_node_ids(ii);
    cell_id=temporal_cell_ids{ii};
    
    %update contact list
    node_list_contact=str2num(sequence_contacts{ii});
    if(length(node_list_contact)>1)
        node_list_contact=node_list_contact(2:end);
        ids_map_contact(node_id)=node_list_contact(2:end);
    else
        node_list_contact=[];
    end
    
    %check node content (1 it has the content, 0 otherwise)
    %and if I am not a producer.
    if(ids_map_content(node_id) && ids_map_state(node_id)~=1)
        
        %check if I was a requester
        if(ids_map_state(node_id)==-1)
            requester_sadisfied=requester_sadisfied+1;
            ids_map_state(node_id)=0;
        end
        
        %check if you have to keep it (producer?)
        if(rand<cell_ids_prob_caching(cell_id))
            ids_map_state(node_id)=1;
        end
        
        %check for PIT entry
        if(isempty(find(node_list_contact==ids_map_PIT(node_id))))
        else
            ids_map_content(ids_map_PIT(node_id))=1;
        end
    end
    
    %if I am a producer start opp. communication
    if(ids_map_state(node_id)==1)
        for idx_contact=1:length(node_list_contact)
            %no exchange between producers
            if(ids_map_state(node_list_contact(idx_contact))~=1)
                %replicate opp. the content
                if(rand<cell_ids_prob_replication(cell_id))
                    ids_map_content(node_list_contact(idx_contact))=1;
                end
            end
        end
    end
    
    
    
    %neutral node
    if(ids_map_state(node_id)==0)
        %check if you become requester
        if(rand<cell_ids_prob_requester(cell_id))
            ids_map_state(node_id)=-1;
            requester_overall=requester_overall+1;
        end
    end
    
    %requester node
    if(ids_map_state(node_id)==-1)
        for idx_contact=1:length(node_list_contact)
            ids_map_PIT(node_list_contact(idx_contact))=node_id;
        end
        
    end
end


requester_sadisfied/requester_overall
toc