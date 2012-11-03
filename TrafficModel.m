clear all;
clc;

%% Properties of the road
RoadLength=200;
Width=12;
LaneWidth=4;
NumberofLanes=Width/LaneWidth;
UnitRoadLength=6;
RoadResolution=RoadLength/UnitRoadLength;

x=0:LaneWidth:Width;
y=0:UnitRoadLength:RoadLength;
[X Y] = meshgrid(x,y);
plot(X,Y);
daspect([1 1 1]);
hold on;

%% Properties of the car
CarWidth=2;
CarLength=3;
CarX=zeros(30,1);
CarY=zeros(30,1);

%% Initializing the car positions
for i=1:30
    if(mod(i,3)==1)
        CarX(i,1)=2;
    elseif(mod(i,3)==2)
        CarX(i,1)=6;
    else
        CarX(i,1)=10;
    end
    
    CarY(i,1)=3+6*floor((i-1)/3);
end

%% Displaying how the initial positions of the cars look 
[numberofcars nothing]=size(CarX);%nothing = junk value
for i=1:numberofcars
    Xdata = [CarX(i)-CarWidth/2,CarX(i)+CarWidth/2,CarX(i)+CarWidth/2,CarX(i)-CarWidth/2]';
    Ydata = [CarY(i)-CarLength/2,CarY(i)-CarLength/2,CarY(i)+CarLength/2,CarY(i)+CarLength/2]';
    Zdata = ones(4,1);
    patch(Xdata,Ydata,Zdata,'g');
    daspect([1 1 1]);
    i
%     pause(0.5);
    hold on;
end

%% Assume initial velocities
CarXvel=zeros(30,1);%assume 0 velocity in the x direction
CarYvel=2*ones(30,1);%assume starting velocity to be 2meters/sec in y direcion

%% Define a car coming in the reverse direction

ReverseCarX = 2;%assuming that the illegal traffic flows in lane1
ReverseCarY= 80;%assuming starting position of the illegal traffic
ReverseCarXvel=0;
ReverseCarYvel=-1;%the velocity of the illegal traffic is negative as it is moving in the oppostite direction
accelerationFlag=1;%used to indicate if the velocity is to be increased or decreased
min_Dist=2;%minimum distance between 2 vehicles taken to be 2 meters
acceleration=0.5;%assume that there is either an acceleration or deceleration of 0.5meteres/sec squared
sze=(35-1)/0.1%size of the time frame
SaveCarXPos=zeros(30,sze);%save values of distance and velocity in the following 3 matrices
SaveCarYPos=zeros(30,sze);
SaveCarYvel=zeros(30,sze);
count=1;
for t=1:0.1:35
    clf;
    
    x=0:LaneWidth:Width;
    y=0:UnitRoadLength:RoadLength;
    [X Y] = meshgrid(x,y);
    plot(X,Y);
    xlabel('Lane');
    ylabel('Distance');
    str=sprintf('Time = %d',t);
    title(str);
    axis([0 13 0 Inf]);
    daspect([1 1 1]);
    hold on;

    ReverseCarX=ReverseCarX+ReverseCarXvel*0.1;
    ReverseCarY=ReverseCarY+ReverseCarYvel*0.1;
    
    Xdata = [ReverseCarX-CarWidth/2,ReverseCarX+CarWidth/2,ReverseCarX+CarWidth/2,ReverseCarX-CarWidth/2]';
    Ydata = [ReverseCarY-CarLength/2,ReverseCarY-CarLength/2,ReverseCarY+CarLength/2,ReverseCarY+CarLength/2]';
    Zdata = ones(4,1);

    patch(Xdata,Ydata,Zdata,'r');
    daspect([1 1 1]);
    
%     pause(0.5);
    hold on; 

    for i=1:1:30
        
        accelerationFlag=1;%initially assume that the car should accelerate away
        LaneChange=0;%assume that Lane change is not required
        CarX(i,1)=CarX(i,1)+CarXvel(i,1)*0.1;%modify the positions
        CarY(i,1)=CarY(i,1)+CarYvel(i,1)*0.1;
        
        
        if(CarX(i,1)==ReverseCarX)%This means that we are bothered only abt the cars in the lane in which the reverse car is comming
            if(ReverseCarY-CarY(i,1)<=min_Dist)%check to avoid collision
                 LaneChange=1;%Change lane because the min distance condition is violated
                accelerationFlag=-1;%also reduce speed
            end
        end
        
        
        PresentLaneCars=find(CarX==CarX(i,1));%find out cars in the present lane
        PresentLaneCarsY=zeros(size(PresentLaneCars,1),1);
        for j=1:size(PresentLaneCars,1);
            PresentLaneCarsY(j,1)=CarY(PresentLaneCars(j),1);%find out Y positions of cars in present lane
        end
         
         AboveLaneCars=find(PresentLaneCarsY>CarY(i,1));%we are interested in cars which are ahead of the present car
         AboveCarsY=zeros(size(AboveLaneCars,1),1);
        for j=1:size(AboveLaneCars,1)
            AboveCarsY(j,1)=PresentLaneCarsY(AboveLaneCars(j));%find out Y positions of cars in present lane
        end
         CompareCarY=min(AboveCarsY);%get the minimum Y postion. which is essentially the car immediately infront of the present car.
         if(CompareCarY-CarY(i,1)<=min_Dist)%check if the distance between the car immediately ahead is less than minimum distance
            if(CarYvel(i,1)>0);
             LaneChange=1;
            accelerationFlag=-1;%should the distance be less than min_Dist, decellerate
            end
         end
        
        %some gimmicks for showing blips moving on the screen
        Xdata = [CarX(i)-CarWidth/2,CarX(i)+CarWidth/2,CarX(i)+CarWidth/2,CarX(i)-CarWidth/2]';
        Ydata = [CarY(i)-CarLength/2,CarY(i)-CarLength/2,CarY(i)+CarLength/2,CarY(i)+CarLength/2]';
        Zdata = ones(4,1);
        patch(Xdata,Ydata,Zdata,'b');
        daspect([1 1 1]);
        hold on; 

        CarYvel(i,1)=CarYvel(i,1)+accelerationFlag*acceleration*0.1;%using the v=u+at formula of kinematics
        
        if(CarYvel(i,1)<0)
            CarYvel(i,1)=0;%Critical low velocity
        end
        
        if(CarYvel(i,1)>3)%critical high velocity is 3m/s
            CarYvel(i,1)=3;
        end
        
        
        if LaneChange==1%execute this section if LaneChange has specifically been changed to 1
            if CarX(i,1)==2%car in lane 1 changes to lane 2
                CarX(i,1)=6;
            elseif CarX(i,1)==6%car in lane 2 changes to lane 3
                    CarX(i,1)=10;
            elseif CarX(i,1)==10% car in lane 3 changes to lane 2
                CarX(i,1)=6;
            end
            CarYvel(i,1)=0;
        end
             
        
    end
    SaveCarXPos(:,count)=CarX(:,1);%save the position profile of all the cars at the time t
    SaveCarYPos(:,count)=CarY(:,1);
    SaveCarYvel(:,count)=CarYvel(:,1);%save the velocity profile of all the cars at time t
    count=count+1;%increment the counter for the matrices that save the displacement and velocity profiles
        
    pause(0.1);
    
    if(mod(t,4)==0)
        str=sprintf('View at time = %d.jpg',t);
        saveas(gcf,str);
    end
    
    
end



%% Plotting distance and velocities


for i=1:30
    plot(SaveCarYPos(i,:)');
    xlabel('time');
    ylabel('Distance covered');
    str=sprintf('Distance vs time for car %d',i);
    title(str)
    str=sprintf('Distance vs time for car %d.jpeg',i);
    saveas(gcf,str);
    
    plot(SaveCarYvel(i,:)');
    xlabel('time');
    ylabel('Velocity');
    str=sprintf('Velocity vs time for car %d',i);
    title(str)
    str=sprintf('Velocity vs time for car %d.jpeg',i);
    saveas(gcf,str);
end