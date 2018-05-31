clear;
close all;
clc;


%% gui

hfig = findobj('tag','path');
if (isempty(hfig))

	% create figure
	hfig = figure;
	set(hfig,'numbertitle','off');               % erase figure number
	set(hfig,'name',['Path planning']);
	set(hfig,'MenuBar','none');                  % erase menu
	set(hfig,'doublebuffer','on');               % two graphic buffers
	set(hfig,'CloseRequestFcn',@close)          % close request function (close window)
	set(hfig,'tag','MOSTRARDADES');              % identify figure
    set(hfig,'Units','Normalized','Position',[0.1,0.1,0.75,0.75]);

	drawnow;
	refresh;
	
    % mennu creation
	hmenu = uimenu('Label','&Algorithms','Tag','M');       
		uimenu(hmenu,'Label','&exit','Callback',@close,'separator','on','Accelerator','E');
        uimenu(hmenu,'Label','&improved algorithm','Callback',@start,'separator','on','Accelerator','S');
        uimenu(hmenu,'Label','&non-improved algorithm','Callback',@start_noimprove,'separator','on','Accelerator','S');
    
        
    %the plots
    astar_axes = axes('tag','astar','unit', 'normalized', 'position', [0.025 0.1 0.45 0.8]); 
    nav_axes = axes('tag','nav','unit', 'normalized', 'position', [0.525 0.1 0.45 0.8]);
    % titles for plot
    uicontrol('tag','pla','Parent',hfig,'Style','Text','Units','normalized','Position',[0.15 0.9 0.2 0.07],'string','Planification','fontSize',20);
    uicontrol('tag','nav','Parent',hfig,'Style','Text','Units','normalized','Position',[0.65 0.9 0.2 0.07],'string','Navigation','fontSize',20);
end	

%if a new algorthm is summomed reset GUI
function new()
% crer el figure
	hfig = figure;
	set(hfig,'numbertitle','off');               % erase figure number
	set(hfig,'name',['Path planning']);
	set(hfig,'MenuBar','none');                  % erase menus and buttons
	set(hfig,'doublebuffer','on');               % two graphic buffers
	set(hfig,'CloseRequestFcn',@close)          % function close request
	set(hfig,'tag','MOSTRARDADES');              % identify figure
    set(hfig,'Units','Normalized','Position',[0.1,0.1,0.75,0.75]); %size figure in relative units

	drawnow;
	refresh;
	
    % basic menu on top
	hmenu = uimenu('Label','&Algorithms','Tag','M');       
		uimenu(hmenu,'Label','&exit','Callback',@close,'separator','on','Accelerator','E');
        uimenu(hmenu,'Label','&improved algorithm','Callback',@start,'separator','on','Accelerator','S');
        uimenu(hmenu,'Label','&non-improved algorithm','Callback',@start_noimprove,'separator','on','Accelerator','S');
    
    %the two plots
    astar_axes = axes('tag','astar','unit', 'normalized', 'position', [0.025 0.1 0.45 0.8]); 
    nav_axes = axes('tag','nav','unit', 'normalized', 'position', [0.525 0.1 0.45 0.8]);
    
    %the two titles for each plot
    uicontrol('tag','pla','Parent',hfig,'Style','Text','Units','normalized','Position',[0.15 0.9 0.2 0.07],'string','Planification','fontSize',20);
    uicontrol('tag','nav','Parent',hfig,'Style','Text','Units','normalized','Position',[0.65 0.9 0.2 0.07],'string','Navigation','fontSize',20);

end  

%function for closing the GUI
function close(hco,eventStruct)
closereq;
end
%*****************************************************************************************
%                                     NON IMPROVED ALGORITHM
%*****************************************************************************************
%start algorithm without straight line analysis, dynamic speed limits
function start_noimprove(hco,eventStruct)
%erase possible changes to figure
new()
close all;
%restart ros node
rosshutdown
rosinit
%resolution (scale factor)
res=1;

%assign axes for astar
hfig = findobj('tag','path');
axes_astar = findobj('tag','astar');
axes(axes_astar)

%%read the csv file 
[file, path]=uigetfile('*.csv');
fullpath=[path file];
map=csvread(fullpath);
dmap=size(map);

%DEFINE THE 2-D MAP ARRAY FROM THE CSV
% Initialize the MAP with input values
% Obstacle=-1,Target = 0,Robot=1,Space=2
MAX_X=dmap(1);
MAX_Y=dmap(2);
%This array stores the coordinates of the map and the 
%Objects in each coordinate
MAP=2*(ones(MAX_X,MAX_Y));

axis([1 MAX_X+1 1 MAX_Y+1])
grid on;
grid minor;
hold on;
n=0;%Number of Obstacles

%detect obstacles in the CSV and store in the map also checks for
%corners and puts an * in their sorroundings, all following the criteria
%specified upwards
for x=1:1:MAX_X
        for y=1:1:MAX_Y
            %corner placement
            if x<MAX_X && y<MAX_Y
                if map(x,y)==1 && map(x+1,y)==0 && map(x+1,y+1)==0 && map(x,y+1)==0
                    MAP(x+1,y+1)=-1;
                    MAP(x,y+1)=-1;
                    MAP(x+1,y)=-1;
                    plot(x+1+.5,y+1+.5,'b*');
                    plot(x+.5,y+1+.5,'b*');
                    plot(x+1+.5,y+.5,'b*');
                end
                if map(x,y)==0 && map(x+1,y)==1 && map(x+1,y+1)==0 && map(x,y+1)==0
                    MAP(x,y)=-1;
                    MAP(x,y+1)=-1;
                    MAP(x+1,y+1)=-1;
                    plot(x+.5,y+.5,'b*');
                    plot(x+.5,y+1+.5,'b*');
                    plot(x+1+.5,y+1+.5,'b*');          
                end
                if map(x,y)==0 && map(x+1,y)==0 && map(x+1,y+1)==1 && map(x,y+1)==0
                    MAP(x,y)=-1;
                    MAP(x,y+1)=-1;
                    MAP(x+1,y)=-1;
                    plot(x+1+.5,y+.5,'b*');
                    plot(x+.5,y+1+.5,'b*');
                    plot(x+.5,y+.5,'b*');        
                end
                if map(x,y)==0 && map(x+1,y)==0 && map(x+1,y+1)==0 && map(x,y+1)==1
                    MAP(x+1,y+1)=-1;
                    MAP(x,y)=-1;
                    MAP(x+1,y)=-1;
                    plot(x+1+.5,y+1+.5,'b*');
                    plot(x+.5,y+.5,'b*');
                    plot(x+1+.5,y+.5,'b*');          
                end
            end
           %obstacle placement
           if map(x,y)==1
                xval=x;
                yval=y;
                MAP(xval,yval)=-1;
                plot(xval+.5,yval+.5,'ro');
           end
            
        end
    end
    

% BEGIN Interactive Obstacle, Target, Start Location selection
pause(1);
h=msgbox('Please Select the Target using the Left Mouse button');
uiwait(h,5);
if ishandle(h) == 1
    delete(h);
end
but=0;
while (but ~= 1) %Repeat until the Left button is not clicked
    [xval,yval,but]=ginput(1);
end

xval=floor(xval);
yval=floor(yval);
xTarget=xval;%X Coordinate of the Target
yTarget=yval;%Y Coordinate of the Target

MAP(xval,yval)=0;%Initialize MAP with location of the target
plot(xval+.5,yval+.5,'gd');
text(xval+1,yval+.5,'Target')

%ask the user for the current position of the robot
prompt1='enter the start x position';
title1='pos_x';
pos_x = inputdlg(prompt1,title1);

prompt2='enter the start y position';
title2='pos_y';
pos_y = inputdlg(prompt2,title2);

%save and plot the start position
xStart=floor(str2double(pos_x));%Starting Position
yStart=floor(str2double(pos_y));%Starting Position
MAP(xStart,yStart)=1;
plot(xStart+.5,yStart+.5,'bo');
xval=xStart;
yval=yStart;

%End of map composition

%starts the A* and navigation phase so we start counting time
tic
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%LISTS USED FOR ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%OPEN LIST STRUCTURE
%--------------------------------------------------------------------------
%IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
%--------------------------------------------------------------------------
OPEN=[];
%CLOSED LIST STRUCTURE
%--------------
%X val | Y val |
%--------------
% CLOSED=zeros(MAX_VAL,2);
CLOSED=[];

%Put all obstacles on the Closed list
k=1;%Dummy counter
for i=1:MAX_X
    for j=1:MAX_Y
        if(MAP(i,j) == -1)
            CLOSED(k,1)=i; 
            CLOSED(k,2)=j; 
            k=k+1;
        end
    end
end
CLOSED_COUNT=size(CLOSED,1);
%set the starting node as the first node
xNode=xval;
yNode=yval;
OPEN_COUNT=1;
path_cost=0;
%distance with minsq
goal_distance=distance(xNode,yNode,xTarget,yTarget);
%all possible nodes with costs (similar a dikstra)
OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,path_cost,goal_distance,goal_distance);
OPEN(OPEN_COUNT,1)=0;
%nodes
CLOSED_COUNT=CLOSED_COUNT+1;
CLOSED(CLOSED_COUNT,1)=xNode;
CLOSED(CLOSED_COUNT,2)=yNode;
NoPath=1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)
%  plot(xNode+.5,yNode+.5,'go');
 exp_array=expand_array(xNode,yNode,path_cost,xTarget,yTarget,CLOSED,MAX_X,MAX_Y);
 exp_count=size(exp_array,1);
 %UPDATE LIST OPEN WITH THE SUCCESSOR NODES
 %OPEN LIST FORMAT
 %--------------------------------------------------------------------------
 %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
 %--------------------------------------------------------------------------
 %EXPANDED ARRAY FORMAT
 %--------------------------------
 %|X val |Y val ||h(n) |g(n)|f(n)|
 %--------------------------------
 for i=1:exp_count
    flag=0;
    for j=1:OPEN_COUNT
        if(exp_array(i,1) == OPEN(j,2) && exp_array(i,2) == OPEN(j,3) )
            OPEN(j,8)=min(OPEN(j,8),exp_array(i,5)); %#ok<*SAGROW>
            if OPEN(j,8)== exp_array(i,5)
                %UPDATE PARENTS,gn,hn
                OPEN(j,4)=xNode;
                OPEN(j,5)=yNode;
                OPEN(j,6)=exp_array(i,3);
                OPEN(j,7)=exp_array(i,4);
            end;%End of minimum fn check
            flag=1;
        end;%End of node check
%         if flag == 1
%             break;
    end;%End of j for
    if flag == 0
        OPEN_COUNT = OPEN_COUNT+1;
        OPEN(OPEN_COUNT,:)=insert_open(exp_array(i,1),exp_array(i,2),xNode,yNode,exp_array(i,3),exp_array(i,4),exp_array(i,5));
     end;%End of insert new element into the OPEN list
 end;%End of i for
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %END OF WHILE LOOP
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %Find out the node with the smallest fn 
  index_min_node = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget);
  if (index_min_node ~= -1)    
   %Set xNode and yNode to the node with minimum fn
   xNode=OPEN(index_min_node,2);
   yNode=OPEN(index_min_node,3);
   path_cost=OPEN(index_min_node,6);%Update the cost of reaching the parent node
  %Move the Node to list CLOSED
  CLOSED_COUNT=CLOSED_COUNT+1;
  CLOSED(CLOSED_COUNT,1)=xNode;
  CLOSED(CLOSED_COUNT,2)=yNode;
  OPEN(index_min_node,1)=0;
  else
      %No path exists to the Target!!
      NoPath=0;%Exits the loop!
  end;%End of index_min_node check
end;%End of While Loop
%Once algorithm has run The optimal path is generated by starting of at the
%last node(if it is the target node) and then identifying its parent node
%until it reaches the start node.This is the optimal path

i=size(CLOSED,1);
Optimal_path=[];
xval=CLOSED(i,1);
yval=CLOSED(i,2);
i=1;
Optimal_path(i,1)=xval;
Optimal_path(i,2)=yval;
i=i+1;

if ( (xval == xTarget) && (yval == yTarget))
    inode=0;
   %Traverse OPEN and determine the parent nodes
   parent_x=OPEN(node_index(OPEN,xval,yval),4);%node_index returns the index of the node
   parent_y=OPEN(node_index(OPEN,xval,yval),5);
   
   while( parent_x ~= xStart || parent_y ~= yStart)
           Optimal_path(i,1) = parent_x;
           Optimal_path(i,2) = parent_y;
           %Get the grandparents:-)
           inode=node_index(OPEN,parent_x,parent_y);
           parent_x=OPEN(inode,4);%node_index returns the index of the node
           parent_y=OPEN(inode,5);
           i=i+1;
    end;
 j=size(Optimal_path,1);
 %Plot the Optimal Path!
 p=plot(Optimal_path(j,1)+.5,Optimal_path(j,2)+.5,'bo');
 j=j-1;
 %draw each step
 for i=j:-1:1
  pause(.25);
  set(p,'XData',Optimal_path(i,1)+.5,'YData',Optimal_path(i,2)+.5);
 drawnow ;
 end;
 plot(Optimal_path(:,1)+.5,Optimal_path(:,2)+.5);
else
 pause(1);
 h=msgbox('Sorry, No path exists to the Target!','warn');
 uiwait(h,5);
end

%we need to flip the outputed path as its inverse and apply resolution
Optimal_path=flip(Optimal_path);
Optimal_path=Optimal_path/res;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%NAVIGATION PHASE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%current location of the robot to use with odometry
robotCurrentLocation=[xStart yStart];

%subscribers for odometry and laser topics
handles.odomSub = rossubscriber('/odom', 'BufferSize', 25);
handles.laserSub = rossubscriber('/scan', 'BufferSize', 5);
%publisher for velocity
handles.velPub = rospublisher('/mobile_base/commands/velocity');

%DEFINE PURE PURSUIT CONTROLER
%Based on the path defined above and a robot motion model, you need a path
%Following controller to drive the robot along the path. Create the path
controller = robotics.PurePursuit;
controlRate = robotics.Rate(10);
controller.Waypoints = Optimal_path;
lin=1.25;
controller.DesiredLinearVelocity = lin;
controller.MaxAngularVelocity = lin*3;
controller.LookaheadDistance = lin/4;
%PLOT THE ACTUAL MAP RECORDED BY THE ROBOT
%in the second axis the map is continuosly plot, here is just initialized
%with a blank ocupancy grid and the first plot of seen data as well as
%robot pose and the path obtained with A*
map_slam = robotics.BinaryOccupancyGrid(dmap(1,1)+5,dmap(1,2)+5,5);
plotobj = TurtleBotVisualizer([0,dmap(1,1)+2,0,dmap(1,2)+2]);
plot(Optimal_path(:,1), Optimal_path(:,2),'k--d')
pose = readPose(handles.odomSub.LatestMessage,robotCurrentLocation);
laserMsg = receive(handles.laserSub,3);
laserdata = readLaserData(laserMsg);
plotData(plotobj,pose,laserdata);
[dataWorld]=plotData(plotobj,pose,laserdata);
if ~isempty(laserdata)
    setOccupancy(map_slam,dataWorld,1);
end 

%NAVIGATION STARTS
while (pose(1)~=xTarget && pose(2)~=yTarget)
    %adquire new lectures of laser and odometry
    laserMsg = receive(handles.laserSub,3);
    odomMsg = handles.odomSub.LatestMessage;
    laserdata = readLaserData(laserMsg);
    pose = readPose(odomMsg,robotCurrentLocation);
    % Re-compute speeds
    [v, w] = step(controller,pose);
    %%send new speed comands
    exampleHelperTurtleBotSetVelocity(handles.velPub, v, w);
    waitfor(controlRate);
    %draw slam map
    plotData(plotobj,pose,laserdata);
    hold on 
    h= plot(Optimal_path(:,1), Optimal_path(:,2),'k--d');
    [dataWorld]=plotData(plotobj,pose,laserdata);
    if ~isempty(laserdata)
        setOccupancy(map_slam,dataWorld,1);
    end 
    %check if we arrived, as planner doesnt take into account the 0.5 we
    %need to check
    if pose(1)>(xTarget-0.1) && pose(2)>(yTarget-0.1) && pose(1)<(xTarget+0.1) && pose(2)<(yTarget+0.1)
        exampleHelperTurtleBotSetVelocity(handles.velPub, 0, 0);
        break;
    end
end
%end of the navigation, so we end counting time
toc
exampleHelperTurtleBotSetVelocity(handles.velPub, 0, 0);
rosshutdown;
end


%*****************************************************************************************
%                                     IMPROVED ALGORITHM
%*****************************************************************************************
function start(hco,eventStruct)
%close all previous windows
close all;
%start the GUI another time
new();
%close possible previous ROS ros matlab nodes
rosshutdown;
%start a new node
rosinit;

%resolution (scale factor)
res=1;
%assign axes for astar
hfig = findobj('tag','path');
axes_astar = findobj('tag','astar');
axes(axes_astar)

%%read the csv file 
[file, path]=uigetfile('*.csv');
fullpath=[path file];
map=csvread(fullpath);
dmap=size(map);


%DEFINE THE 2-D MAP ARRAY FROM THE CSV
MAX_X=dmap(1);
MAX_Y=dmap(2);
%This array stores the coordinates of the map and the 
%Objects in each coordinate
MAP=2*(ones(MAX_X,MAX_Y));

% Obtain Obstacle, Target and Robot Position
% Initialize the MAP with input values
% Obstacle=-1,Target = 0,Robot=1,Space=2
axis([1 MAX_X+1 1 MAX_Y+1])
grid on;
grid minor;
hold on;

%detect obstacles in the CSV and store them in the A* map also checks for
%corners and puts an * in their sorroundings
for x=1:1:MAX_X
        for y=1:1:MAX_Y
            %corner detection
            if x<MAX_X && y<MAX_Y
                if map(x,y)==1 && map(x+1,y)==0 && map(x+1,y+1)==0 && map(x,y+1)==0
                    MAP(x+1,y+1)=-1;
                    MAP(x,y+1)=-1;
                    MAP(x+1,y)=-1;
                    plot(x+1+.5,y+1+.5,'b*');
                    plot(x+.5,y+1+.5,'b*');
                    plot(x+1+.5,y+.5,'b*');
                end
                if map(x,y)==0 && map(x+1,y)==1 && map(x+1,y+1)==0 && map(x,y+1)==0
                    MAP(x,y)=-1;
                    MAP(x,y+1)=-1;
                    MAP(x+1,y+1)=-1;
                    plot(x+.5,y+.5,'b*');
                    plot(x+.5,y+1+.5,'b*');
                    plot(x+1+.5,y+1+.5,'b*');          
                end
                if map(x,y)==0 && map(x+1,y)==0 && map(x+1,y+1)==1 && map(x,y+1)==0
                    MAP(x,y)=-1;
                    MAP(x,y+1)=-1;
                    MAP(x+1,y)=-1;
                    plot(x+1+.5,y+.5,'b*');
                    plot(x+.5,y+1+.5,'b*');
                    plot(x+.5,y+.5,'b*');        
                end
                if map(x,y)==0 && map(x+1,y)==0 && map(x+1,y+1)==0 && map(x,y+1)==1
                    MAP(x+1,y+1)=-1;
                    MAP(x,y)=-1;
                    MAP(x+1,y)=-1;
                    plot(x+1+.5,y+1+.5,'b*');
                    plot(x+.5,y+.5,'b*');
                    plot(x+1+.5,y+.5,'b*');          
                end
            end    
           %wall detection
           if map(x,y)==1
                xval=x;
                yval=y;
                MAP(xval,yval)=-1;
                plot(xval+.5,yval+.5,'ro');
           end
            
        end
    end
    

% BEGIN Interactive Obstacle, Target, Start Location selection

%ask for a target
pause(1);
h=msgbox('Please Select the Target using the Left Mouse button');
uiwait(h,5);
if ishandle(h) == 1
    delete(h);
end
xlabel('Please Select the Target using the Left Mouse button','Color','black');
but=0;
while (but ~= 1) %Repeat until the Left button is not clicked
    [xval,yval,but]=ginput(1);
end
%transform the coordenates to an int and save it
xval=floor(xval);
yval=floor(yval);
xTarget=xval;%X Coordinate of the Target
yTarget=yval;%Y Coordinate of the Target

MAP(xval,yval)=0;%Initialize MAP with location of the target
plot(xval+.5,yval+.5,'gd');
text(xval+1,yval+.5,'Target')

%ask for the current position (x,y)
prompt1='enter the start x position';
title1='pos_x';
pos_x = inputdlg(prompt1,title1);

prompt2='enter the start y position';
title2='pos_y';
pos_y = inputdlg(prompt2,title2);

%transform the current position to an int and save it
xStart=floor(str2double(pos_x));%Starting Position
yStart=floor(str2double(pos_y));%Starting Position
MAP(xStart,yStart)=1;
plot(xStart+.5,yStart+.5,'bo');
xval=xStart;
yval=yStart;

%End of map composition and start timing
tic

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%LISTS USED FOR ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%OPEN LIST STRUCTURE
%--------------------------------------------------------------------------
%IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
%--------------------------------------------------------------------------
OPEN=[];
%CLOSED LIST STRUCTURE
%--------------
%X val | Y val |
%--------------
% CLOSED=zeros(MAX_VAL,2);
CLOSED=[];

%Put all obstacles on the Closed list
k=1;%Dummy counter
for i=1:MAX_X
    for j=1:MAX_Y
        if(MAP(i,j) == -1)
            CLOSED(k,1)=i; 
            CLOSED(k,2)=j; 
            k=k+1;
        end
    end
end
CLOSED_COUNT=size(CLOSED,1);
%set the starting node as the first node
xNode=xval;
yNode=yval;
OPEN_COUNT=1;
path_cost=0;
%distance with minsq
goal_distance=distance(xNode,yNode,xTarget,yTarget);
%all possible nodes with costs (similar a dikstra)
OPEN(OPEN_COUNT,:)=insert_open(xNode,yNode,xNode,yNode,path_cost,goal_distance,goal_distance);
OPEN(OPEN_COUNT,1)=0;
%nodes
CLOSED_COUNT=CLOSED_COUNT+1;
CLOSED(CLOSED_COUNT,1)=xNode;
CLOSED(CLOSED_COUNT,2)=yNode;
NoPath=1;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% START ALGORITHM
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
while((xNode ~= xTarget || yNode ~= yTarget) && NoPath == 1)
%  plot(xNode+.5,yNode+.5,'go');
 exp_array=expand_array(xNode,yNode,path_cost,xTarget,yTarget,CLOSED,MAX_X,MAX_Y);
 exp_count=size(exp_array,1);
 %UPDATE LIST OPEN WITH THE SUCCESSOR NODES
 %OPEN LIST FORMAT
 %--------------------------------------------------------------------------
 %IS ON LIST 1/0 |X val |Y val |Parent X val |Parent Y val |h(n) |g(n)|f(n)|
 %--------------------------------------------------------------------------
 %EXPANDED ARRAY FORMAT
 %--------------------------------
 %|X val |Y val ||h(n) |g(n)|f(n)|
 %--------------------------------
 for i=1:exp_count
    flag=0;
    for j=1:OPEN_COUNT
        if(exp_array(i,1) == OPEN(j,2) && exp_array(i,2) == OPEN(j,3) )
            OPEN(j,8)=min(OPEN(j,8),exp_array(i,5)); %#ok<*SAGROW>
            if OPEN(j,8)== exp_array(i,5)
                %UPDATE PARENTS,gn,hn
                OPEN(j,4)=xNode;
                OPEN(j,5)=yNode;
                OPEN(j,6)=exp_array(i,3);
                OPEN(j,7)=exp_array(i,4);
            end;%End of minimum fn check
            flag=1;
        end;%End of node check
%         if flag == 1
%             break;
    end;%End of j for
    if flag == 0
        OPEN_COUNT = OPEN_COUNT+1;
        OPEN(OPEN_COUNT,:)=insert_open(exp_array(i,1),exp_array(i,2),xNode,yNode,exp_array(i,3),exp_array(i,4),exp_array(i,5));
     end;%End of insert new element into the OPEN list
 end;%End of i for
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %END OF WHILE LOOP
 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 %Find out the node with the smallest fn 
  index_min_node = min_fn(OPEN,OPEN_COUNT,xTarget,yTarget);
  if (index_min_node ~= -1)    
   %Set xNode and yNode to the node with minimum fn
   xNode=OPEN(index_min_node,2);
   yNode=OPEN(index_min_node,3);
   path_cost=OPEN(index_min_node,6);%Update the cost of reaching the parent node
  %Move the Node to list CLOSED
  CLOSED_COUNT=CLOSED_COUNT+1;
  CLOSED(CLOSED_COUNT,1)=xNode;
  CLOSED(CLOSED_COUNT,2)=yNode;
  OPEN(index_min_node,1)=0;
  else
      %No path exists to the Target!!
      NoPath=0;%Exits the loop!
  end;%End of index_min_node check
end;%End of While Loop
%Once algorithm has run The optimal path is generated by starting of at the
%last node(if it is the target node) and then identifying its parent node
%until it reaches the start node.This is the optimal path

i=size(CLOSED,1);
Optimal_path=[];
xval=CLOSED(i,1);
yval=CLOSED(i,2);
i=1;
Optimal_path(i,1)=xval;
Optimal_path(i,2)=yval;
i=i+1;

if ( (xval == xTarget) && (yval == yTarget))
    inode=0;
   %Traverse OPEN and determine the parent nodes
   parent_x=OPEN(node_index(OPEN,xval,yval),4);%node_index returns the index of the node
   parent_y=OPEN(node_index(OPEN,xval,yval),5);
   
   while( parent_x ~= xStart || parent_y ~= yStart)
           Optimal_path(i,1) = parent_x;
           Optimal_path(i,2) = parent_y;
           %Get the grandparents:-)
           inode=node_index(OPEN,parent_x,parent_y);
           parent_x=OPEN(inode,4);%node_index returns the index of the node
           parent_y=OPEN(inode,5);
           i=i+1;
   end
 j=size(Optimal_path,1);
 %Plot the Optimal Path!
 p=plot(Optimal_path(j,1)+.5,Optimal_path(j,2)+.5,'bo');
 j=j-1;
 %draw each step
 for i=j:-1:1
  pause(.25);
  set(p,'XData',Optimal_path(i,1)+.5,'YData',Optimal_path(i,2)+.5);
 drawnow ;
 end
 plot(Optimal_path(:,1)+.5,Optimal_path(:,2)+.5);
else
 pause(1);
 h=msgbox('Sorry, No path exists to the Target!','warn');
 uiwait(h,5);
end

%INITIAL CORRECTIONS
%we need to flip the outputed path as its inverse
Optimal_path=flip(Optimal_path);
%reescale path, target and current position if its necessary
Optimal_path=Optimal_path/res;
xTarget=xTarget/res;
yTarget=yTarget/res;
xStart=xStart/res;
yStart=yStart/res;

%STRAIGHT LINES DETECTION
%arrays to save staright lines
straight=[];
straight_end=[];
%flag to control that we just save the first position of an straigth line
first=0;

%read and store large straight lines in the planning, if they are really
%long we can speed up in them
for i=1:1:size(Optimal_path,1)-4
    %we analyse the current point of the path and for more to see if
    %there id a straight line in x
    if (Optimal_path(i,1)==Optimal_path(i+1,1)) && (Optimal_path(i,1)==Optimal_path(i+2,1)) && (Optimal_path(i,1)==Optimal_path(i+3,1))
        %first time we see the straight line we save the point
        if first==0
            straight=[straight Optimal_path(i,1) Optimal_path(i,2) 1];
            first=1;
        end
        %if there is a change in a detected straight line, it ends
        if(Optimal_path(i+3,1)~=Optimal_path(i+4,1))
            straight_end=[straight_end Optimal_path(i+3,1) Optimal_path(i+3,2) 1];
            first=0;
        end
    end
    %we analyse the current point of the path and for more to see if
    %there id a straight line in y 
    if (Optimal_path(i,2)==Optimal_path(i+1,2)) && (Optimal_path(i,2)==Optimal_path(i+2,2)) && (Optimal_path(i,2)==Optimal_path(i+3,2))   
    %first time we see the straight line we save the point    
        if first==0
            straight=[straight Optimal_path(i,1) Optimal_path(i,2) 2];
            first=1;
        end
        %if there is a change in a detected straight line, it ends
        if(Optimal_path(i+3,2)~=Optimal_path(i+4,2))
            straight_end=[straight_end Optimal_path(i+3,1) Optimal_path(i+3,2) 2];
            first=0;
        end
    end
end
%DISTANCE MEASURMENTS FOR ODOMETRY CORRECTION
%initialize vector to save distances for odometry correction
t_dist=[];
for i=1:3:length(straight_end)
    %initial value for distance
    dis=0;
    %distance for x straight lines
    if straight_end(i+2)==1
        count=straight_end(i+1)*res;
        %while there is no obstacle in the original map from the final point
        %of the straight line following x direction...
        while map(straight_end(i)*res,count)~=1
            %we move to the closest wall (1 is right 2 is left)
            if count>size(map,2)/2
                count=count+1;
                dis=dis+1;
                dir=1;
            else
                count=count-1;
                dis=dis+1;
                dir=2;
            end
        end
        %finally when the obstacle is detected we save the distance to it
        %and the right or left direction
        t_dist=[t_dist dis dir];
    end
    %distance for y straight lines
    if straight_end        
        %while there is no obstacle in the original map from the final point
        %of the straight line following x direction...
        count=straight_end(i)*res;
        while map(count,straight_end(i+1)*res)~=1
            %we move to the closest wall (1 is right 2 is left)
            if count>size(map,1)/2
                count=count+1;
                dis=dis+1;
                dir=1;
            else
                count=count-1;
                dis=dis+1;
                dir=2;
            end
        end
        %finally when the obstacle is detected we save the distance to it
        %and the right or left direction
        t_dist=[t_dist dis dir];
    end
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%NAVIGATION PHASE
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

%DEFINITION OF PUBLISHERS AND SUBSCRIBERS 
%save robot current position to update pose
robotCurrentLocation=[xStart yStart];
%subscrive to odometry and laser topics
handles.odomSub = rossubscriber('/odom', 'BufferSize', 25);
handles.laserSub = rossubscriber('/scan', 'BufferSize', 5);
%create a publisher for velocity topic 
handles.velPub = rospublisher('/mobile_base/commands/velocity');

%DEFINITION OF THE CONTROLLER
% Based on the path defined above and a robot motion model, you need a path
% following controller to drive the robot along the path. Create the path
controller = robotics.PurePursuit;
controlRate = robotics.Rate(10);
controller.Waypoints = Optimal_path;
lin=1;
controller.DesiredLinearVelocity = lin;
controller.MaxAngularVelocity = lin*3;
controller.LookaheadDistance = lin/4;%following matlab recommendation

%INITIAL PLOT OF THE ROBOT VIEW
%initialize a blank ocupancy grid and add to the plot the first data of current
%pose, the saw scan data plus the calculated path. After the scan data is replicated to the occupancy
%grid
map_slam = robotics.BinaryOccupancyGrid(dmap(1,1)+5,dmap(1,2)+5,5);
plotobj = TurtleBotVisualizer([0,dmap(1,1)+2,0,dmap(1,2)+2],hfig);
plot(Optimal_path(:,1), Optimal_path(:,2),'k--d')
pose = readPose(handles.odomSub.LatestMessage,robotCurrentLocation);
laserMsg = receive(handles.laserSub,3);
laserdata = readLaserData(laserMsg);
plotData(plotobj,pose,laserdata);
[dataWorld]=plotData(plotobj,pose,laserdata);
if ~isempty(laserdata)
    setOccupancy(map_slam,dataWorld,1);
end 

%initial configuration for finnish, start and straight states
start_part=1;
finnish_part=0;
recta=0;
%counter for straight lines in path
c=1;
%max of straight lines possible
c_max=length(straight);
%counter for points to correct odometry
c_tdist=1;

%NAVIGATION STARTS HERE
%while we dont hit the target navigation continues
while (pose(1)~=xTarget && pose(2)~=yTarget)
    %when it starts we dont let the controler to have all its speed
    if (pose(1)<xStart+1 && pose(2)<yStart+1)
        lin=1;
        controller.DesiredLinearVelocity = lin;
        controller.MaxAngularVelocity = lin*3;
        controller.LookaheadDistance = lin/4;
    else
        start_part=0;
    end
    %in the middle of the map as fast as we can
     if finnish_part==0 && start_part==0 && recta==0     
         lin=1.25;
         controller.DesiredLinearVelocity = lin;
         controller.MaxAngularVelocity = lin*3;
         controller.LookaheadDistance = lin/4;
     end
    %if there are straight lines and we didnt reached the maximum of them
    %we can consider checking the straight lines array
    if isempty(straight)==false && c<c_max
        %first we update pose
        odomMsg = handles.odomSub.LatestMessage;
        pose = readPose(odomMsg,robotCurrentLocation);
        %then we compare the current pose with those Nodes save in the
        %straight line vector if they match enough we are in an straight
        %line
        if pose(1)>(straight(c)-0.25) && pose(1)<(straight(c)+0.25) && pose(2)>(straight(c+1)-0.25) && pose(2)<(straight(c+1)+0.25)
            %we display such information and update the speed limits
            disp('recta');
            recta=1;
            lin=2;
            controller.DesiredLinearVelocity = lin;
            controller.MaxAngularVelocity = lin*3;
            controller.LookaheadDistance = lin/4;
        end
        %when we are on the point of leaving the straight line correct the
        %odometry using the stored distances
        if pose(1)>(straight_end(c)-0.25) && pose(1)<(straight_end(c)+0.25) && pose(2)>(straight_end(c+1)-0.25) && pose(2)<(straight_end(c+1)+0.25 && recta==1)
            %read info from the laser
            laserMsg = receive(handles.laserSub,3);
            laserdata = readLaserData(laserMsg);
            if isempty(laserdata)==false
                dist_laser=mean(laserdata(2,:));
            end
            %if we are close enough to a wall theoretically (less than 5
            %squares) then we correct, we obtain the new pose by
            %substracting the measured distance by the laser to the
            %theoretical one then this number is added or substracted to
            %the current location depending on the direction left or right
            if t_dist(c_tdist)<5
                 if straight_end(c+2)==1
                     disp('corrigiendo y')
                     correct=(dist_laser-t_dist(c_tdist))/2;
                     disp(correct)
                     if t_dist(c_tdist+1)==1
                         robotCurrentLocation(2)=robotCurrentLocation(2)-correct;
                     else
                         robotCurrentLocation(2)=robotCurrentLocation(2)+correct;
                     end
                 end
                 if straight_end(c+2)==2
                     disp('corrigiendo x')
                     correct=(dist_laser-t_dist(c_tdist))/2;
                     disp(correct)
                     if t_dist(c_tdist+1)==1
                         robotCurrentLocation(1)=robotCurrentLocation(1)+correct;
                     else
                         robotCurrentLocation(1)=robotCurrentLocation(1)-correct;
                     end
                 end
                 c_tdist=c_tdist+2;         
            end
            %we increment the end counter when we are about to leave the
            %straight line and by 3 cause the vector contains (x,y,dir) per
            %straight line
            if recta==1
                c=c+3;
            end
            recta=0;
        end
        
    end
    %as in the non improved mode we update odometry and laser, if we dont
    %do so and we are out of an straight line, sart or finish zone we dont
    %have info
    laserMsg = receive(handles.laserSub,3);
    odomMsg = handles.odomSub.LatestMessage;
    laserdata = readLaserData(laserMsg);
    pose = readPose(odomMsg,robotCurrentLocation);
    % Re-compute speeds
    [v, w] = step(controller,pose);
    %send them
    exampleHelperTurtleBotSetVelocity(handles.velPub, v, w);
    waitfor(controlRate);
    %draw slam map
    plotData(plotobj,pose,laserdata);
    hold on 
    h= plot(Optimal_path(:,1), Optimal_path(:,2),'k--d');
    [dataWorld]=plotData(plotobj,pose,laserdata);
    if ~isempty(laserdata)
        setOccupancy(map_slam,dataWorld,1);
    end 
    %if we are near arriving 
    if pose(1)>(xTarget-2) && pose(2)>(yTarget-2) && pose(1)<(xTarget+2) && pose(2)<(yTarget+2)
        finnish_part=1;
        lin=1;
        controller.DesiredLinearVelocity = lin;
        controller.MaxAngularVelocity = lin*3;
        controller.LookaheadDistance = lin/4;
    end
    %check if we arrived
    if pose(1)>(xTarget-0.1) && pose(2)>(yTarget-0.1) && pose(1)<(xTarget+0.1) && pose(2)<(yTarget+0.1)
        exampleHelperTurtleBotSetVelocity(handles.velPub, 0, 0);
        break;
    end
    
end 
%when we end the while the algorithm is finnished so timing stops and the
%node is closed
toc
exampleHelperTurtleBotSetVelocity(handles.velPub, 0, 0);
rosshutdown;
end



