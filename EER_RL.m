%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                              EER-RL                                  %
%       Energy-Efficient Routing based on Reinforcement Learning       %        %                      %
%                      Mobile Information Systems                      %
%                           Research Article                           %
%                                                                      %
% (c) Vially KAZADI MUTOMBO, PhD Candidate                             %
% Soongsil University                                                  %
% Department of Computer Science                                       %
% mutombo.kazadi@gmail.com                                             %
% February 2021                                                        %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
close all;
clear;
clc;

%%%%%%%%%%%%%%%%%%%%%% Beginning  %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%% Network Settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Sensing Field Dimensions in meters %
xm=100;
ym=100;
x=0; % added for better display results of the plot
y=0; % added for better display results of the plot
% Number of Nodes in the field %
n=100;
% Number of Dead Nodes in the beggining %
dead_nodes=0;
% Coordinates of the Sink (location is predetermined in this simulation) %
sinkx=50;
sinky=50;

%%% Energy parameters %%%
% Energy required to run circuity (both for transmitter and receiver) %
Eelec=50*10^(-9); % units in Joules/bit
ETx=50*10^(-9); % units in Joules/bit
ERx=50*10^(-9); % units in Joules/bit
% Transmit Amplifier Types %
Eamp=100*10^(-12); % units in Joules/bit/m^2 (amount of energy spent by the amplifier to transmit the bits)
% Data Aggregation Energy %
EDA=5*10^(-9); % units in Joules/bit
% Size of data package %
k=4000; % units in bits 
% Round of Operation %
rnd=0;
tot_rnd=10000;
% Current Number of operating Nodes %
op_nodes=n; %Operating nodes
transmissions=0;
d(n,n)=0;
source=1;
flag1stdead=0;
range_C = 20; %Transmission range
alpha=1; %Learning Rate
gamma = 0.95; % Discount Factor
p=0.5 % Energy's Probabilistic parameter 
q1=1-p % Hop count probabilistic parameter

CH_tot= ceil(n*0.1);
%%%%%%%%%%%%%%%%%%%%%%%%%%% End of Network settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%% WSN Creattiom %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plotting the WSN %
for i=1:n
    
    
    NET(i).id=i;	% sensor's ID number
    NET(i).x=rand(1,1)*xm;	% X-axis coordinates of sensor node
    NET(i).y=rand(1,1)*ym;	% Y-axis coordinates of sensor node
    %NET(i).E=Eo;     % nodes energy levels (initially set to be equal to "Eo
    NET(i).E = randi([1,2]); % For heterogeneous WNET
    NET(i).Eo = NET(i).E;
    NET(i).cond=1;   % States the current condition of the node. when the node is operational its value is =1 and when dead =0
    %NET(i).dts=0;    % nodes distance from the sink
    NET(i).dts= sqrt((sinkx-NET(i).x)^2 + (sinky-NET(i).y)^2);
    NET(i).hop=ceil(NET(i).dts/range_C); %Hop count estimate to the sink
    NET(i).role=0;   % node acts as normal if the value is '0', if elected as a cluster head it  gets the value '1' (initially all nodes are normal)
    %NET(i).pos=0;
    %NET(i).first=0;  %Initial route available. If it the first time a node send a packet its value is 0, otherwise it's 1
    NET(i).closest=0;
    NET(i).prev=0;
    %NET(i).next=0;
    %NET(i).dis=0;	% distance between two nodes headin towards to the cluster head from position 1
    NET(i).sel=0;    % states if the node has already operated for this round or not (if 0 then no, if 1 then yes) 
    NET(i).rop=0;    % number of rounds node was operational
    NET(i).dest=0;
    NET(i).dts_ch=0;
    %order(i)=0;

    hold on;
    figure(1);
    plot(x,y,xm,ym,NET(i).x,NET(i).y,'ob','DisplayName','cm');
    plot(x,y,xm,ym,sinkx,sinky,'*r','DisplayName','sink');
    title 'EER-RL';
    xlabel '(m)';
    ylabel '(m)';
end

% find Neighbord nodes
%Compute Q-Value
min_E = min([NET.E]); 
max_E = max([NET.E]);
for i=1:n
    if(min_E ==max_E)
        Q(i) = 1 / NET(i).hop;
        NET(i).Q = Q(i);
    else
        Q(i) = (p*(NET(i).E - min_E)/(max_E-min_E)+(q1/NET(i).hop));
        NET(i).Q = Q(i);
        %CH = maxk(Q,10); %Find 10 strongest nodes 
    end
end
%------------------- BEGINNING OF CLUSTERING -----------------------------
%CLUSTER HEAD ELECTION
for i=1:n
   CM(i) = NET(i); %Make a copy of the network
end
tot = 1;

while(tot<=CH_tot)
for i=1:n
    %maxx= max([CM.Q]);
    %disp(maxx);
    if(CM(i).Q == max([CM.Q]))
        if tot == 1 && CM(i).dts>=15 && CM(i).dts<=50
            CH(tot) = CM(i);
            NET(i).role=1;
            
            plot(x,y,xm,ym,NET(i).x,NET(i).y,'Or','DisplayName','CH');
            CM(i).Q = 0;
            tot =tot+1;
        elseif tot>1 &&  CM(i).dts>=15 && CM(i).dts<=50
            cl = 0;
            for t = 1:length(CH)
                dts = sqrt((CM(i).x-CH(t).x)^2 + (CM(i).y-CH(t).y)^2);
                if(dts <=15)
                    cl=cl +1;
                    break;
                end
            end
            if cl==0
                CH(tot) = CM(i);
                
                plot(x,y,xm,ym,NET(i).x,NET(i).y,'Or');
                
                NET(i).role=1;
                CM(i).Q = 0;
                tot =tot+1;  
            else
                CM(i).Q = 0;
            end
        else
            CM(i).Q = 0;
        end
            
    end
       if tot >CH_tot
           break;
       end
end
end
%END CLUSTER HEAD ELECTION

%CLUSTER FORMATION
for i=1:n
    for ch=1:CH_tot
        dts_ch(i,ch) = sqrt((NET(i).x-CH(ch).x)^2 + (NET(i).y-CH(ch).y)^2);
        dts_ch(dts_ch==0)=NaN;
    end
    if NET(i).dts<=range_C
        NET(i).dest = 0;
        NET(i).dts_ch = 0;
    else
        if NET(i).role==0
            for ch = 1:CH_tot
                dtsCh = sqrt((NET(i).x-CH(ch).x)^2 + (NET(i).y-CH(ch).y)^2);
                if NET(i).E>0 && dtsCh <= min([dts_ch(i,:)])
                    NET(i).dest = CH(ch).id;
                    NET(i).dts_ch = min([dts_ch(i,:)]);
                    
                    figure(1);
                    plot([NET(i).x CH(ch).x], [NET(i).y CH(ch).y],'-c','DisplayName','Tx');
                    %legend;
                    %hold off;
                    
                    break;
                end
            end
        end  
    end             
end
%END CLUSTER FORMATION
for i=1:CH_tot
    ncm=1;
    for j=1:n
        if(NET(j).dest == CH(i).id & NET(j).role==0)
            cluster(i,ncm)= NET(j);
            ncm= ncm+1;
        end
    end
end
%------------------- END OF CLUSTERING -----------------------------------



%COMMUNICATION PHASE------------------------------------------------------
dead=0;
while(op_nodes>0 && rnd<5000)
    
    %Node nearby the Sink node
    ns=1;
    for l=1:n
        if NET(l).E>0
            if NET(l).dts<=range_C && NET(l).role==0
                Next_sink(ns) = NET(l);     
            end
        end    
    end
   
    for j = 1:CH_tot
        for ns=1:length(Next_sink)
            dts_tmp(j,ns) = sqrt((CH(j).x-Next_sink(ns).x)^2 + (CH(j).y-Next_sink(ns).y)^2);               
        end
    end
     %en Node nearby the sink
    energy=0;
    %INTRACLUSTER MULTIHOP COMMUNICATION
    for i=1:CH_tot
        ncm=1;
        for j =1:length(cluster(i,:))        
            if cluster(i,j).dest == CH(i).id
                if cluster(i,j).E>0
                    maxQ = max([cluster(i,:).Q]);
                    if (cluster(i,j).dts_ch<=range_C | cluster(i,j).Q ==maxQ) 
                        if cluster(i,j).prev==0
                            ETx= Eelec*k + Eamp*k*cluster(i,j).dts_ch^2;
                            cluster(i,j).E = cluster(i,j).E-ETx;
                            NET(cluster(i,j).id).E=cluster(i,j).E;
                            energy=energy+ETx;
                            CH(i).prev = CH(i).prev +1;
                        else
                            ERx=(EDA+Eelec)*k;
                            ETx= Eelec*k + Eamp*k*cluster(i,j).dts_ch^2;
                            NET(cluster(i,j).id).E=NET(cluster(i,j).id).E-ETx-ERx;
                            cluster(i,j).E = NET(cluster(i,j).id).E;
                            cluster(i,j).prev=0;
                            energy=energy+ETx+ERx;
                            CH(i).prev = CH(i).prev +1;
                        end
                        %Compute the reward
                        Q_old = cluster(i,j).Q;
                        R= (p*(cluster(i,j).E - min_E)/(max_E-min_E)+(q1/cluster(i,j).hop));
                        
                        %update Q_value
                        cluster(i,j).Q =Q_old + alpha*(R+ gamma * maxQ -Q_old) ;
                        NET(cluster(i,j).id).Q = cluster(i,j).Q;
                        
                    else
                        for nex = 1:length(cluster(i,:))
                            if(cluster(i,nex).E>0)
                                if(cluster(i,nex).Q ==maxQ)
                                    next = cluster(i,nex);
                                    cluster(i,nex).prev=1;
                                    nextID=nex;
                                    break;
                                end
                            else
                                cluster(i,nex).Q = -100;
                            end
                           
                        end
                        dts_cm = sqrt((next.x-cluster(i,j).x)^2 + (next.y-cluster(i,j).y)^2);   
                        if cluster(i,j).prev==0
                            ETx= Eelec*k + Eamp*k*dts_cm^2;
                            NET(cluster(i,j).id).E=NET(cluster(i,j).id).E-ETx;
                            cluster(i,j).E = cluster(i,j).E-ETx;
                            energy=energy+ETx;
                        else
                            ERx=(EDA+Eelec)*k;
                            ETx= Eelec*k + Eamp*k*dts_cm^2;
                            NET(cluster(i,j).id).E=NET(cluster(i,j).id).E-ETx-ERx;
                            cluster(i,j).E = cluster(i,j).E-ETx-ERx;
                            cluster(i,j).prev=0;
                            energy=energy+ETx;
                        end
                        %Compute the reward
                        Q_old = cluster(i,j).Q;
                        R= (p*(cluster(i,j).E - min_E)/(max_E-min_E)+(q1/cluster(i,j).hop));
                        
                        %update Q_value
                        cluster(i,j).Q =Q_old + alpha*(R+ gamma * maxQ -Q_old) ;
                        NET(cluster(i,j).id).Q = cluster(i,j).Q;
                        
                        Q_old = NET(next.id).Q;
                        cluster(i,nextID).Q  =Q_old + alpha*(R+ gamma * maxQ -Q_old) ;
                        NET(cluster(i,nextID).id).Q = cluster(i,nextID).Q;
                    end
                
                else
                    cluster(i,j).Q = -100;
                end
                    
            end
        end
    end

    %END OF INTRACLUSTER MULTIHOP COMMUNICATION

    %INTERCLUSTER COMMUNICATION
    for j =1:CH_tot
        thres = NET(CH(j).id).Eo * 0.4;
        if CH(j).E >thres && thres>0
            if(CH(j).dts<=range_C)
                if CH(j).prev ==0
                    ETx= Eelec*k + Eamp*k*CH(j).dts^2;
                    NET(CH(j).id).E=NET(CH(j).id).E-ETx;
                    CH(j).E = CH(j).E-ETx;

                    energy=energy+ETx;
                else
                    %ERx=(EDA+Eelec)*k;
                    %ETx= Eelec*k + Eamp*k*CH(j).dts^2;
                    %Edis = (k*CH(j).prev*(Eelec + EDA) + k*(Eelec+Eamp*(CH(j).dts^2)));
                    Edis = (k*(Eelec + EDA) + k*(Eelec+Eamp*(CH(j).dts^2)));
                    NET(CH(j).id).E=NET(CH(j).id).E-Edis
                    CH(j).E = CH(j).E-Edis;
                    energy=energy+Edis;
                    CH(j).prev =0;
                end
                
            else
                for ns=1:length(Next_sink)
                    if dts_tmp(j,ns) == min(dts_tmp(j,:))
                        if CH(j).prev ==0
                            ETx= Eelec*k + Eamp*k*dts_tmp(j,ns)^2;
                            NET(CH(j).id).E=NET(CH(j).id).E-ETx;
                            CH(j).E = CH(j).E-ETx;

                            energy=energy+ETx;
                        else
                            %ERx=(EDA+Eelec)*k;
                            %ETx= Eelec*k + Eamp*k*CH(j).dts^2;
                            %Edis = (k*CH(j).prev*(Eelec + EDA) + k*(Eelec+Eamp*(CH(j).dts^2)));
                            Edis = (k*(Eelec + EDA) + k*(Eelec+Eamp*(dts_tmp(j,ns)^2)));
                            NET(CH(j).id).E=NET(CH(j).id).E-Edis
                            CH(j).E = CH(j).E-Edis;
                            energy=energy+Edis;
                            CH(j).prev =0;
                        end  
                        NET(Next_sink(ns).id).prev = 1;
                        break;
                    end
                    
                end
            end
            
            %Compute the reward
            Q_old = CH(j).Q;
            R= (p*(CH(j).E - min_E)/(max_E-min_E)+(q1/CH(j).hop));
                        
            %update Q_value
            CH(j).Q =Q_old + alpha*(R+ gamma * maxQ -Q_old) ;
            NET(CH(j).id).Q = CH(j).Q;
        elseif CH(j).E <= thres || thres<=0
            
            %------------------- BEGINNING OF RECLUSTERING --------------
            %CLUSTER HEAD ELECTION
            aln =0; %Alive nodes before reclustering
            for i=1:n
                NET(i).dest =0;
                NET(i).dts_ch =0;
                NET(i).role=0;
                if NET(i).E>0
                    NET(i).Eo = NET(i).E;
                    CM(i) = NET(i);
                    aln = aln+1; 
                else
                    NET(i).Eo = NET(i).E;
                    NET(i).cond = 0;
                    
                    
                end    
            end
            CH_tot = ceil(aln/10)
            %disp("NA ="+CH_tot+" ALN="+aln+" and N="+n)
            tot = 1;
            while(tot<=CH_tot)
                for i=1:n
                    %maxx= max([CM.Q]);
                    %disp(maxx);
                    if(CM(i).Q == max([CM.Q]) && CM(i).Q>=0)
                        if tot == 1 && CM(i).dts>=range_C 
                            NET(i).role=1;
                            CH(tot) = NET(i);
                            %plot(x,y,xm,ym,NET(i).x,NET(i).y,'Or');
                            CM(i).Q = 0;
                            tot =tot+1;
                        elseif tot>1 &&  CM(i).dts>=range_C  
                            cl=0;
                            for t = 1:tot-1
                                dts = sqrt((CM(i).x-CH(t).x)^2 + (CM(i).y-CH(t).y)^2);
                                if(dts <range_C )
                                    cl= cl+1;
                                end
                            end
                            if cl==0
                                NET(i).role=1;
                                CH(tot) = NET(i);
                                %plot(x,y,xm,ym,NET(i).x,NET(i).y,'Or');

                                CM(i).Q = 0;
                                tot =tot+1;  
                            else
                                CM(i).Q = 0;
                            end
                        else
                            CM(i).Q = 0;
                        end

                    end
                       if tot>CH_tot
                           break;
                       end
                end
            end
            %END CLUSTER HEAD ELECTION

            %CLUSTER FORMATION
            for i=1:n
                for ch=1:CH_tot
                    dts_ch(i,ch) = sqrt((NET(i).x-CH(ch).x)^2 + (NET(i).y-CH(ch).y)^2);
                    dts_ch(dts_ch==0)=NaN;
                end
                if NET(i).dts<=range_C
                    NET(i).dest = 0;
                    NET(i).dts_ch = 0;
                else
                    if NET(i).role==0
                        for ch = 1:CH_tot
                            dtsCh = sqrt((NET(i).x-CH(ch).x)^2 + (NET(i).y-CH(ch).y)^2);
                            if NET(i).E>0 && dtsCh == min([dts_ch(i,:)])
                                NET(i).dest = CH(ch).id;
                                NET(i).dts_ch = min([dts_ch(i,:)]);
%                                 figure(1);
%                                 plot([NET(i).x CH(ch).x], [NET(i).y CH(ch).y])
%                                 hold on;
                            end
                        end
                    end  
                end             
            end
            %END CLUSTER FORMATION
            for i=1:CH_tot
                ncm=1;
                for j=1:n
                    if NET(j).E>0
                        if(NET(j).dest == CH(i).id & NET(j).role==0)
                            %cluster(i,ncm)= [];
                            cluster(i,ncm)= NET(j);
                            ncm= ncm+1;
                        end
                    end
                    
                end
            end
            %------------------- END OF RECLUSTERING ---------------------

        end
        CH(j).prev=0;
    end
    %END INTERCLUSTER COMMUNICATION
    
    %Nodes around the sink node
    for l=1:n
        if NET(l).E>0
            if NET(l).dts<=range_C && NET(l).role==0
                if NET(l).prev==0
                    ETx= Eelec*k + Eamp*k*NET(l).dts^2;
                    NET(l).E=NET(l).E-ETx;
                    energy=energy+ETx;
                else
                    Edis = (k*(Eelec + EDA) + k*(Eelec+Eamp*(NET(l).dts^2)));
                    NET(l).E = NET(l).E-Edis;
                    energy=energy+Edis;
                    NET(l).prev =0;
                end
                
                %Compute the reward
                Q_old = NET(l).Q;
                R= (p*(NET(l).E - min_E)/(max_E-min_E)+(q1/NET(l).hop));

                %update Q_value
                NET(l).Q =Q_old + alpha*(R+ gamma * maxQ -Q_old) ;
                NET(l).Q = NET(l).Q;
            end
        end
        
        
    end
    
    %Compute round, Energy consumed per round and ...
    rnd = rnd+1;
    E_round(rnd) = energy;
    
    disp(rnd);
    dead=0;
    for i =1:n
        if NET(i).E<=0 || NET(i).cond==0
            dead = dead+1;
            NET(i).Q= -100;
            NET(i).cond = 0;
            op_nodes = n-dead;
            
        end
    end
    dead_rnd(rnd)=dead;
    op_nodes_rnd(rnd)=op_nodes;
    disp(op_nodes);
end
% END COMMUNICATION PHASE ------------------------------------------------

% Plotting Simulation Results "Operating Nodes per Transmission" %
figure(2)
plot(1:rnd,op_nodes_rnd(1:rnd),'-','Linewidth',1);
%legend('RL-CEBRP');
title ({'Operating Nodes per Round';'' })
xlabel 'Rounds';
ylabel 'Operating Nodes';
hold on;

% Plotting Simulation Results "Energy consumed per Round" %
figure(3)
plot(1:rnd,E_round(1:rnd),'-','Linewidth',1);
%legend('RL-CEBRP')
title ({'Energy consumed per Round';'' })
xlabel 'Rounds';
ylabel 'Energy consumed in J';
hold on;

% % Plotting Simulation Results "Energy consumed per Round" %
figure(4)
plot(1:rnd,E_round(1:rnd),'-r','Linewidth',2);
%legend('RL-EBRP')
title ({'EER-RL'; 'Energy consumed per Round';})
xlabel 'Rounds';
ylabel 'Energy consumed in J';
hold on;
% 
% % Plotting Simulation Results "Cumulated dead nodes per Round" %
figure(5)
plot(1:rnd,dead_rnd(1:rnd),'-r','Linewidth',2);
%legend('RL-EBRP');
title ({'EER-RL'; 'Total dead nodes per Round';})
xlabel 'Rounds';
ylabel 'Dead Nodes';
hold on;