clear all; close all; clc;

%% INPUT
return
%number of messagesppp
n_messages=76;

%%%%%%%%%%%warning3%%%%%%%%%%%
%time to transfer the message in tenth of a second
%10=1s half second or 5=0.5s
transfer_message_time=5;

%floating time in s
total_sim_time=1800; 


tic
%% contacts
%   in contact_list
%   * 1 id: node identifier;
%   * 2 id: node identifier;
%   * 3 sim_time: simulation time;
%   * 4 contact_start: event contact start (1) or event contact end (0);
fileID = fopen('data/contact_list.txt','r');
data_read_contact = textscan(fileID,'%f%f%f%f','Delimiter',' ');
fclose(fileID);
data_contact=cell2mat(data_read_contact);

%%%%%%%%warning1%%%%%%%%%%%%%
%time in (s)*10 -> 1002.1==10021
data_contact(:,3)=data_contact(:,3)*10;


%contact start: event of contact start (id,id,sim_time)
start_contact(:,1:3)=data_contact(data_contact(:,4)==1,1:3);

%contact end: even of contact end (id,id,sim_time)
end_contact(:,1:3)=data_contact(data_contact(:,4)==0,1:3);


%% density
%   in node_list
%   * 1 id: node identifier;
%   * 2 sim_time: simulation time;
%   * 3 position vehicle x on the map
%   * 4 position vehicle y on the map
fileID = fopen('data/node_list.txt','r');
data_read_density = textscan(fileID,'%f%f%f%f','Delimiter',' ');
fclose(fileID);
data_density=cell2mat(data_read_density);

%%%%%%%%warning 1%%%%%%%%%%%%%
%time in (s)*10 -> 1002.1==10021
data_density(:,2)=data_density(:,2)*10;

%all nodes identifiers
node_ids=unique(data_density(1:length(data_density)))';

%group ids by simulation time
time_node_density={};
time_start=1;
jj=1;
data_density(1,5)=1;
for ii=2:length(data_density)
    if(data_density(ii,2)~=data_density(ii-1,2) || ii==length(data_density))
        
        if( ii<length(data_density))
            time_end=ii-1;
        else
            time_end=ii;
        end
        
        time_node_density{jj}=data_density(time_start:time_end,1);
        time_start=ii;
        jj=jj+1;
    end
    
    data_density(ii,5)=jj;
    time_end=ii;
    
end

%extract simulation time (s) and normalize it
time_sim=(1:length(unique(data_density(:,2))))';

%normalize contact time e.g., sim_time (1000,1100)=>(1,101)
start_contact(:,3)=start_contact(:,3)-min(data_density(:,2))+1;
end_contact(:,3)=end_contact(:,3)-min(data_density(:,2))+1;



%list of nodes exiting the AZ over time
time_node_exiting={};
jj=1;
for ii=2:length(time_node_density)
    tmp=setdiff(time_node_density{ii-1},time_node_density{ii})   ;
    
    if(~isempty(tmp))
        time_node_exiting(jj,1)={tmp};
        time_node_exiting(jj,2)={ii};
        jj=jj+1;
    end
end

%% main algorithm parameters

%contact map (key=id, value=neighbor list)
ids_map_contact = containers.Map(node_ids,cell(length(node_ids),1));

%message map (key=id, value=message list [available(1), miss(0)])
ids_map_message = containers.Map(node_ids,cell(length(node_ids),1));


%%%%%%%%%%%warning2%%%%%%%%%%%
%state map (key=id, value=node state [available(0), busyfor(ds)] )
%ds is time to transfer the message in tenth of a second
ids_map_state = containers.Map(node_ids,zeros(length(node_ids),1));

%transfer map (key=id, value=[index message in the channel,id in contact])
ids_map_transfer = containers.Map(node_ids,cell(length(node_ids),1));





%% message struct in each node
for ids_idx=1:length(node_ids)
    
    for message_idx=1:n_messages
        ids_map_message(node_ids(ids_idx))=zeros(1,n_messages);
    end
    
end

%% seeding
perc_node_seeded=1; %1->100% & .5->50% % 0->0%
for ids_idx=1:length(time_node_density{1,1})
    
    for message_idx=1:n_messages
        ids_map_message(time_node_density{1,1}(ids_idx))=[rand(1,n_messages)<perc_node_seeded];
    end
    
end












%% mean density
node_density=zeros(length(time_node_density),1);
for ii=1:length(time_node_density)
node_density(ii)=length(time_node_density{1,ii});
end

mean_density_over_time=mean(node_density)

%% mean contact time 
contact_start_order_by_ids=unique(sort(start_contact(:,1:2),2),'rows');
contact_end_order_by_ids=unique(sort(end_contact(:,1:2),2),'rows');

for ii=1:length(contact_end_order_by_ids)
find_start=sum(ismember(contact_start_order_by_ids,contact_end_order_by_ids(ii,:)),2)>1;
idx_contact_start=find(find_start>0);


event_start_time=start_contact(idx_contact_start,3);
event_end_time=end_contact(ii,3);


event_duration=event_end_time-event_start_time;

if(event_duration<0)
    event_duration=0;
end

%%%%warning 4-> /10 because tenth of seconds%%%
contact_time(ii)=event_duration/10; %in seconds



end
contact_time=contact_time(contact_time>0);
mean_contact_time=mean(contact_time) %contact per seconds 

%% mean node exting over time 

for ii=1:length(time_node_exiting)
    node_exiting(ii)=length(time_node_exiting{ii,1});
end


mean_node_exiting_over_time=sum(node_exiting)/total_sim_time %exiting per seconds


%% mean contact rate
mean_contact_rate=length(contact_time)/total_sim_time %contact rate per sec




%indexs
start_contact_idx=1;
end_contact_idx=1;
exting_idx=1;

%message availabilit over time
availability_messages=zeros(length(time_sim),n_messages);


%time in tenth of second-> time_idx=10 => 1s
for time_idx=1:length(time_sim)
    
    
    %contact event start
    while(start_contact_idx<=length(start_contact) && time_idx==start_contact(start_contact_idx,3))
        %exctract contact node ids
        id_1=start_contact(start_contact_idx,1);
        id_2=start_contact(start_contact_idx,2);
        
        %add id in neighborlist
        ids_map_contact(id_1)=[ids_map_contact(id_1) id_2];
        ids_map_contact(id_2)=[ids_map_contact(id_2) id_1];
        
        %check other contacts starting at the same timestamp
        start_contact_idx=start_contact_idx+1;
        
    end
    
    %contact event end
    while(end_contact_idx<=length(end_contact) && time_idx==end_contact(end_contact_idx,3))
        id_1=end_contact(end_contact_idx,1);
        id_2=end_contact(end_contact_idx,2);
        
        %remove id in neighborlist
        tmp_list=ids_map_contact(id_1);
        idx_to_delate=ismember(tmp_list,id_2);
        tmp_list(idx_to_delate)=[];
        ids_map_contact(id_1)=tmp_list;
        
        tmp_list=ids_map_contact(id_2);
        idx_to_delate=ismember(tmp_list,id_1);
        tmp_list(idx_to_delate)=[];
        ids_map_contact(id_2)=tmp_list;
        
        %check other contacts ending at the same timestamp
        end_contact_idx=end_contact_idx+1;
    end
    
    %nodes exting event
    while(exting_idx<=length(time_node_exiting) && time_idx==time_node_exiting{exting_idx,2})
        
        %list node_exiting at time_idx
        list_node_exiting=time_node_exiting{exting_idx,1};
        
        for delete_idx=1:length(list_node_exiting)
            
            %neighbor list node of the exiting node
            list_neighbor_to_delate=ids_map_contact(list_node_exiting(delete_idx));
            
            %delete exit node in every neighbor list
            for delete_neighbor_idx=1:length(list_neighbor_to_delate)
                tmp_list=ids_map_contact(list_neighbor_to_delate(delete_neighbor_idx));
                idx_to_delate=ismember(tmp_list,list_node_exiting(delete_idx));
                tmp_list(idx_to_delate)=[];
                ids_map_contact(list_neighbor_to_delate(delete_neighbor_idx))=tmp_list;
            end
            
            %delete node list of the exit node
            ids_map_contact(list_node_exiting(delete_idx))=[];
        end
        exting_idx=exting_idx+1;
    end
    
    
    %% content transfer update
    density_list=time_node_density{1,time_idx};
    
    %update exchange
    for density_idx=1:length(density_list)
        
        %node info
        node_id=density_list(density_idx);
        node_state=ids_map_state(node_id);
        node_neighbor=ids_map_contact(node_id);
        node_messages=ids_map_message(node_id);
        
        
        %check node state
        if(node_state>0)
            %extract transfer time and neighbor
            tmp_var=ids_map_transfer(node_id);
            id_neighbor=tmp_var(2);%in the channel
            
            
            
            if(ismember(id_neighbor,node_neighbor))
                %still in contact
                if(node_state==1)
                    %transfer completed (update message list)
                    idx_message_to_update=tmp_var(1);
                    node_messages(idx_message_to_update)=1;
                    ids_map_message(node_id)=node_messages;
                    ids_map_transfer(node_id)=[0 0];
                end
                
                %update state
                ids_map_state(node_id)=node_state-1;
            else
                %neighbor in the channel bur not in the neighbor list
                % reset state
                ids_map_state(node_id)=0;
                ids_map_transfer(node_id)=[0 0];
                node_state=0;
            end
        end
        
        
    end
    
    %check new exchange
    for density_idx=1:length(density_list)
        
        %node info
        node_id=density_list(density_idx);
        node_state=ids_map_state(node_id);
        node_neighbor=ids_map_contact(node_id);
        node_messages=ids_map_message(node_id);
        
        
        
        
        if(node_state==0 && ~isempty(node_neighbor))
            %node availabe to tx/rx
            node_neighbor_idx=1;
            while(node_state==0 && node_neighbor_idx<=length(node_neighbor))
                %extract neighbor id, state, and message list
                neighbor_id=node_neighbor(node_neighbor_idx);
                neighbor_state=ids_map_state(neighbor_id);
                neighbor_messages=ids_map_message(neighbor_id);
                
                %list message to tx/rx
                
                list_messages_to_exchange=xor(node_messages,neighbor_messages);
                
                %check state and message to tx/rx
                if(neighbor_state==0 && sum(list_messages_to_exchange)>0)
                    %update both node state and internal variable
                    ids_map_state(node_id)=transfer_message_time;
                    ids_map_state(neighbor_id)=transfer_message_time;
                    node_state=transfer_message_time; %to exit from the loop
                    
                    %save index message to transfer
                    indx_message_candidates=...
                        find(list_messages_to_exchange==1);
                    choose_rand_message=...
                        indx_message_candidates(...
                        randperm(sum(list_messages_to_exchange),1));
                    idx_message_to_update=choose_rand_message;
                    
                    %update transfer map
                    ids_map_transfer(node_id)=...
                        [idx_message_to_update,neighbor_id] ;
                    ids_map_transfer(neighbor_id)=...
                        [idx_message_to_update,node_id];
                    
                end
                
                node_neighbor_idx=node_neighbor_idx+1;
            end
            
        end
        
        
        
        
    end
    
    %check message availability
    for density_idx=1:length(density_list) 
    availability_messages(time_idx,:)=availability_messages(time_idx,:)+...
        ids_map_message(density_list(density_idx));
    end
    availability_messages(time_idx,:)=availability_messages(time_idx,:)/length(density_list);
end



toc