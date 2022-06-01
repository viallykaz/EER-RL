%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%                              RL-EBRP                                 %
%        Reinforcement Learning based Energy Balancing                 %
%                       Routing Protocol for WSN                       %
%                           Conference Paper                           %
%                             ACM-RACS2020                             %
%                                                                      %
% (c) Vially KAZADI MUTOMBO, PhD Student                               %
% Soongsil University                                                  %
% Department of Computer Science                                       %
% mutombo.kazadi@gmail.com                                             %
% July 2020                                                            %
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
Eo=2; % Initial Energy of nodes (in Joules)
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
range_C = 20; %Communication range
alpha=1; %Learning Rate
gamma = 0.8; % Discount Factor
p=0.7 % Energy's Probabilistic parameter
q1=0.3 % Hop count probabilistic parameter
%%%%%%%%%%%%%%%%%%%%%%%%%%% End of Network settings %%%%%%%%%%%%%%%%%%%%%%%%%%%%

%%%%%%%%%%%%%%%%%%%%%%%%% WSN Creattiom %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plotting the WSN %
for i=1:n
    
    
    NET(i).id=i;	% sensor's ID number
    NET(i).x=rand(1,1)*xm;	% X-axis coordinates of sensor node
    NET(i).y=rand(1,1)*ym;	% Y-axis coordinates of sensor node
    NET(i).E=Eo;     % nodes energy levels (initially set to be equal to "Eo"
    %NET(i).E = randi([2,6]); % For heterogeneous WNET
    NET(i).cond=1;   % States the current condition of the node. when the node is operational its value is =1 and when dead =0
    %NET(i).dts=0;    % nodes distance from the sink
    NET(i).dts= sqrt((sinkx-NET(i).x)^2 + (sinky-NET(i).y)^2);
    NET(i).hop=ceil(NET(i).dts/range_C); %Hop count estimate to the sink
    %NET(i).role=0;   % node acts as normal if the value is '0', if elected as a cluster head it  gets the value '1' (initially all nodes are normal)
    %NET(i).pos=0;
    %NET(i).first=0;  %Initial route available. If it the first time a node send a packet its value is 0, otherwise it's 1
    NET(i).closest=0;
    NET(i).prev=0;
    %NET(i).next=0;
    %NET(i).dis=0;	% distance between two nodes headin towards to the cluster head from position 1
    NET(i).sel=0;    % states if the node has already operated for this round or not (if 0 then no, if 1 then yes) 
    NET(i).rop=0;    % number of rounds node was operational
    %NET(i).tel=0;    % states how many times the node was elected as a Cluster Head
    %order(i)=0;

    hold on;
    figure(1);
    plot(x,y,xm,ym,NET(i).x,NET(i).y,'ob',sinkx,sinky,'*r');
    
    title 'RL-EBRP';
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
for i = 1:n
    %neig = 0;
    m=1;
    for j=1:n
        d(i,j) = sqrt((NET(i).x-NET(j).x)^2 + (NET(i).y-NET(j).y)^2);
        %disp(d(i,j))
        if(i~=j && d(i,j)<=range_C)
            
            dis(i,j) = d(i,j)+NET(j).dts; % Total distance to the sink through neighbord j
            Nhop(i,j) = ceil(dis(i,j)/range_C); % Estimation of hop count to the sink through neighbor j
            %if(NET(i).hop >Nhop(i,j))
            neig(i,m) =NET(j).id;
            Q_value(i,m) = Q(j);
            m=m+1;
            %end
        end
         
    end
   % Neighb(i) = neig; %Neighbor
end

%In case of clustering
%for i =1:length(Q)
%    if ismember(Q(i),CH)
%        NET(i).role = 1;
%    end
%end

%% Data transmission

%Chosing the closest node to the sink with high energy level
%source =1;
%source =1;

while op_nodes>0 || rnd>tot_rnd
    energy=0;
    max_E = max([NET.E]);
    if min([NET.E])<0
        min_E = 0; 
    end
    for source=1:n  
        if NET(source).E>0 %A node can send a packet only if it's alive
            maxQ = max(Q_value(source,:));   %Find the neighbor with the maximum Q-Value
            Num_neig = 1;
            for i=1:n
                if ismember(i,neig(source,:)) && Q(i)>=0
                    %Compute the distance between the two nodes
                    D_betw = sqrt((NET(source).x-NET(i).x)^2 + (NET(source).y-NET(i).y)^2);
                    if(Q_value(source,Num_neig)==maxQ && NET(source).dts>range_C)
                        next = i;
                        if NET(source).prev~=0 && NET(source).prev~=source
                            ERx=(EDA+Eelec)*k;
                            ETx= (EDA+Eelec)*k + Eamp*k*D_betw^2; 
                            NET(source).E=NET(source).E-ETx-ERx;
                            energy=energy+ETx+ERx;
                            NET(source).prev=0;
                        else
                            ETx= Eelec*k + Eamp*k*D_betw^2;
                            NET(source).E=NET(source).E-ETx;
                            energy=energy+ETx;
                        end
                        break;   

                    else
                        if NET(source).prev~=0
                            ERx=(EDA+Eelec)*k;
                            ETx= (EDA+Eelec)*k + Eamp*k*NET(source).dts^2; 
                            NET(source).E=NET(source).E-ETx-ERx;
                            energy=energy+ETx+ERx;
                            NET(source).prev=0;
                        else
                            ETx= Eelec*k + Eamp*k*NET(source).dts^2;
                            NET(source).E=NET(source).E-ETx;
                            energy=energy+ETx;
                        end
                        next = source;
                        %Compute the immediate reward
                        Q_old = Q(i);
                        if NET(i).E<0
                            Q(i) = -Q_value(source,Num_neig);
                        else
                            Q(i)= (p*(NET(i).E - min_E)/(max_E-min_E)+(q1/NET(i).hop));
                        end

                        %Update the Q-value
                        %Q_value(source,Num_neig) = Q(i)+ maxQ;
                        Q_value(source,Num_neig) =Q_old + alpha*(Q(i)+ gamma * maxQ -Q_old) ;
                        Num_neig = Num_neig+ 1;
                        
                        break;
                        
                    end
                    
                elseif(ismember(i,neig(source,:)) && Q(i)<0)
                    
                    if NET(source).prev~=0
                        ERx=(EDA+Eelec)*k;
                        ETx= (EDA+Eelec)*k + Eamp*k*NET(source).dts^2; 
                        NET(source).E=NET(source).E-ETx-ERx;
                        energy=energy+ETx+ERx;
                        NET(source).prev=0;
                    else
                        ETx= Eelec*k + Eamp*k*NET(source).dts^2;
                        NET(source).E=NET(source).E-ETx;
                        energy=energy+ETx;
                    end
                    %Compute thE reward
                    Q_old = Q(i);
                    if NET(i).E<0 % If the residual energy is less than 0 the node get a negative reward
                        Q(i) = -Q_value(source,Num_neig);
                    else
                        Q(i)= (p*(NET(i).E - min_E)/(max_E-min_E)+(q1/NET(i).hop));
                    end
                    
                    %Update the Q-value 
                    %Q_value(source,Num_neig) = Q(i)+ maxQ;
                    Q_value(source,Num_neig) =Q_old + alpha*(Q(i)+ gamma * maxQ -Q_old) ;
                    Num_neig = Num_neig+ 1;
                
                end
                
            end

            NET(source).closest=1;
            %NET(source).dis=min_dis;
            NET(next).sel=1;
            NET(next).prev=NET(next).prev+1;
            NET(next).dis2=sqrt((NET(source).x-NET(next).x)^2 + (NET(source).y-NET(next).y)^2);
            %figure(1);
            plot([NET(source).x NET(next).x], [NET(source).y NET(next).y])
            hold on;
            %source=next;
            %source=source+1;
            %temp=temp+1;
            %order(temp)=source;
        end
    end
    rnd=rnd+1;
    
    disp(rnd);
    %E_round(rnd) = energy;
    dead=0;
    for i =1:n
        NET(i).prev=0;
        if NET(i).E<=0
            NET(i).cond=0;
            dead =dead+1;
            op_nodes=100-dead;
            
            Q(i) = -1000;
            
            if dead==100 || op_nodes==0
                
                op_nodes=0;
                break;
            end
        end
    end
    dead_rnd(rnd) = dead; 
    E_round(rnd) = energy;
    op_nodes_rnd(rnd)= op_nodes;
    disp(op_nodes);
    
end
% Plotting Simulation Results "Operating Nodes per Transmission" %
figure(2)
plot(1:rnd,op_nodes_rnd(1:rnd),'-r','Linewidth',2);
legend('RL-EBRP');
title ({'RL-EBRP'; 'Operating Nodes per Round';})
xlabel 'Rounds';
ylabel 'Operating Nodes';
hold on;

% Plotting Simulation Results "Energy consumed per Round" %
figure(3)
plot(1:rnd,E_round(1:rnd),'-r','Linewidth',2);
legend('RL-EBRP')
title ({'RL-EBRP'; 'Energy consumed per Round';})
xlabel 'Rounds';
ylabel 'Energy consumed in J';
hold on;

% Plotting Simulation Results "Cumulated dead nodes per Round" %
figure(4)
plot(1:rnd,dead_rnd(1:rnd),'-r','Linewidth',2);
legend('RL-EBRP');
title ({'RL-EBRP'; 'Total dead nodes per Round';})
xlabel 'Rounds';
ylabel 'Dead Nodes';
hold on;
