clear
clc
%% Variable Definition

%Range of simulation
x=0:0.01:1;
y=0:0.01:0.4;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Map save 2-dimension data of environment as a density function
%Layer save 1-dimension data of environment in order to build up difference
%equation of WSRN.
%
%Static layer is
%Dynamic layer is
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
ImportanceMap=zeros(length(x),length(y));%
StaticMap=zeros(length(x),length(y));% 
DynamicMap=zeros(length(x),length(y));%
StaticLayer=zeros(length(x)*length(y),1);%
DynamicLayer=zeros(length(x)*length(y),1);%

%Number of tasks and robots
TaskPosition=ones(5,2);
RobotPosition=ones(5,2);


ActuatorData=zeros(length(RobotPosition),1);
ALLPosition=ones(length(RobotPosition)+length(TaskPosition),2);
ALLData=zeros(length(ALLPosition),1);

simulationTime=100;
cost=10000*zeros(simulationTime+1,5);

robotR=0.08;
taskR=0.02;


%% Initialization
%Position of station
StationPosition=[0.1,0.1];
%Position of known tasks
TaskPosition=[0.2,0.1;0.4,0.1;0.6,0.2;0.8,0.15;0.4,0.3];
%Initial position of robots
RobotPosition=[0.11,0.18;0.09,0.19;0.1,0.20;0.11,0.21;0.09,0.22];

ALLPosition=[RobotPosition;TaskPosition];

%令總覆蓋能力為5

%Static Map
for i=1:1:length(x)
    for j=1:1:length(y)
        q=[x(i) y(j)];
        StaticMap(i,j)=0;
    end
end

%Dyanamic Map
for i=1:1:length(x)
    for j=1:1:length(y)
        q=[x(i) y(j)];
        %Task
        for task=1:1:length(TaskPosition(:,1))
        DynamicMap(i,j)=DynamicMap(i,j)+10*(exp(-norm(q-TaskPosition(task,:))^2/(2*taskR^2)));
        end
        %Station
        DynamicMap(i,j)=DynamicMap(i,j)+(5-length(TaskPosition(:,1)))*10*(exp(-norm(q-StationPosition(1,:))^2/(2*0.02^2)));
    end
end

%for迴圈把環境抓到控制空間
for i=1:1:length(x)
    for j=1:1:length(y)
        DynamicLayer(i+(j-1)*length(x))=DynamicMap(i,j);
    end
end

ImportanceMap=DynamicMap;
% ImportanceMap=DynamicMap+StaticMap;




%%
for t=1:1:simulationTime

    
    % plot
    figure('Position',[100 100 900 400])
    subplot(2,1,1)
    plot(TaskPosition(:,1),TaskPosition(:,2),'^')
    hold on
    plot(RobotPosition(:,1),RobotPosition(:,2),'*')
    hold on
    %  for texti=1:1:length(EnvironData(:,1))
    %      for textj=1:1:length(EnvironData(1,:))
    %         t=num2str (EnvironData(texti,textj));
    %         text(x(texti)-0.45,y(textj)-0.45, t);
    %      end
    %  end
    
%     for num=1:1:length(SensorData(:,1))
%         t=num2str (SensorData(num));
%         text(SensorPosition(num,2)+0.05,SensorPosition(num,1)+0.05, t);
%     end
    axis([0 1 0 0.4])
%     voronoi(RobotPosition(:,1),RobotPosition(:,2))
    grid on
    
    subplot(2,1,2)
    [X,Y] = meshgrid(x,y);
  
    surf(x,y,ImportanceMap','LineStyle','none');
    title('\phi_{en}')
    colorbar
    set(gca,'Clim',[-10,20])
    view([0 0 1])
    axis([0 1 0 0.4])
   
%save files
    picname=sprintf('%d.jpg',t);
    saveas(gcf,picname);
    %% Mobile control
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % 這裡假定actuator每個迴圈只能移動0.2
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    phi=zeros(length(RobotPosition(:,1)),1);
    MobileControl=zeros(length(RobotPosition(:,1)),2);
    
    
    for i=1:1:length(ImportanceMap(:,1))
        for j=1:1:length(ImportanceMap(1,:))
            
            q=[x(i), y(j)];
            d=0;
            dphi=ImportanceMap(i,j);
            %Assumption of SDF
            if dphi<0
                dphi=0;
            end
            %penalty function
            for coverbot=1:1:length(RobotPosition(:,1))
                
                d=d+dphi/length(RobotPosition(:,1))-5*exp(-norm(q-RobotPosition(coverbot,:))^2/(2*robotR^2));
                
            end
            [d,z]=max([0 d]);
             h=d^2;
             dh=2*d;
             if z==1
                    h=0;
                    dh=0;
             end
             
             cost(t)=cost(t)+h*dphi;
             
            for whichrobot=1:1:length(RobotPosition(:,1))
                MobileControl(whichrobot,:)= MobileControl(whichrobot,:)+(RobotPosition(whichrobot,:)-q)*dh*dphi*exp(-norm(q-RobotPosition(whichrobot,:))^2/(2*robotR^2));
                 
            end
        end
        
    end
    Km=0.0008;
% Km=100;
    MobileControl=-Km*MobileControl;
    RobotPosition=RobotPosition+MobileControl;%control law
    
    
%avoid robots go out of environment    
    for m=1:1:length(RobotPosition(:,1))
        if RobotPosition(m,1)>1
            RobotPosition(m,1)=1;
        end
        if RobotPosition(m,2)>0.4
            RobotPosition(m,2)=0.4;
        end
        if RobotPosition(m,1)<0
            RobotPosition(m,1)=0;
        end
        if RobotPosition(m,2)<0
            RobotPosition(m,2)=0;
        end
    end
    


    %Environment Data update



end
