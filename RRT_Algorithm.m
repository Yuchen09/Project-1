clear all
clc
%% Define map parameters

% Map grid Resolution 
grid_r = 1;

% Set the size of the map
down  = 0;
up    = 30;
left  = 0;
right = 30;

% Set plot
set(gca,'XLim',[left right]); % x axis range
set(gca,'XTick',[left:grid_r:right]); % x axis tick
set(gca,'YLim',[down up]); % y axis range
set(gca,'YTick',[down:grid_r:up]); % y axis tick
grid on
axis equal
title('RRT Alogrithm');
xlabel('X');
ylabel('Y','Rotation',0);
hold on

% Set wall obstacle 
wall(1,:) = [left, right-1, right-1, left];
wall(2,:) = [down, down, down+1, down+1];
wall(3,:) = [right-1, right, right , right-1];
wall(4,:) = [down, down, up-1, up-1];
wall(5,:) = [right, right, left+1 , left+1];
wall(6,:) = [up-1, up, up, up-1];
wall(7,:) = [left+1, left, left , left+1];
wall(8,:) = [up, up, down+1, down+1];

% Set blcok obstacle
block(1,:) = [left+5, left+10, left+10, left+5];
block(2,:) = [down+5, down+5, down+10, down+10];
block(3,:) = [left+15, left+25, left+25, left+15];
block(4,:) = [down+10, down+10, down+12, down+12];
block(5,:) = [left+7, left+10, left+10, left+7];
block(6,:) = [down+15, down+15, down+25, down+25];

% Generate the wall and block obstacles in plot
fill(wall(1,:),wall(2,:),'k');
fill(wall(3,:),wall(4,:),'k');
fill(wall(5,:),wall(6,:),'k');
fill(wall(7,:),wall(8,:),'k');
fill(block(1,:),block(2,:),'k');
fill(block(3,:),block(4,:),'k');
fill(block(5,:),block(6,:),'k');

%% Initialize parameters

% starting point
Sx = 2;
Sy = 2;

% Target ponit
Tx = 26;
Ty = 26;

% Distance between new node and the existing nodes
Distance = 1;
% If the node reaches this distance, it is considered the ndoe has reached the target point
T_radius = 2;

%% Draw the starting point & target ponit
plot(Sx,Sy,'ro','MarkerFaceColor','r','MarkerSize',9*grid_r);
plot(Tx,Ty,'bo','MarkerFaceColor','b','MarkerSize',9*grid_r);

% Draw the target circle
theta = linspace(0,2*pi);
circle_x = T_radius*cos(theta) + Tx;
circle_y = T_radius*sin(theta) + Ty;
plot(circle_x, circle_y,'g','LineWidth',0.5*grid_r);

%% main body
% Initialize the random tree
Tree_new = []; % current node
Tree_old = []; % current node's parent
Tree_distance = 0;

Tree_new(1,:) = Sx;
Tree_new(2,:) = Sy;
Tree_old(1,:) = Sx;
Tree_old(2,:) = Sy;

node_x = Sx;
node_y = Sy;
node_number = 1;
Target_distance = sqrt((Tx - node_x)^2 + (Tx - node_y)^2);

% Main loop
while(Target_distance > T_radius)

    random_x = (right - left) * rand() + left; % random x value between x axis range limit
    random_y = (up - down) * rand() + down; % random y value between y axis range limit

    [angle, min_node] = find_node(random_x ,random_y,Tree_new);

    pause(0.01);
  
    new_x = Tree_new(1,min_node) + Distance*cos(angle);
    new_y = Tree_new(2,min_node)+ Distance*sin(angle);

    % Check if the new node is inside the wall 
    flag = 0;
    if (new_x > left+1 && new_x< right-1 )

        if (new_y > down+1 && new_y < up-1)

            flag = 1;

        end
    end

    % Check if the new node is inside block obstacles
    for i=1:1:length(block)/2
        if (new_x >min(block(i,:)) && new_x < max(block(i,:)))
            if (new_y >min(block(i+1,:)) && new_y < max(block(i+1,:)))
            flag = 0;
            end
        end           
    end

    % Draw the new node 
    if(flag ==1)
        Target_distance = sqrt((Tx - new_x)^2 + (Ty - new_y)^2);
        plot(new_x,new_y,'.r','MarkerFaceColor','r','MarkerSize',10*grid_r);     
        plot([Tree_new(1,min_node),new_x],[Tree_new(2,min_node),new_y],'-k','LineWidth',0.8*grid_r);      
        
        % Store nodes
        node_number = node_number+1;
        Tree_old(1,node_number) = Tree_new(1,min_node);
        Tree_old(2,node_number) = Tree_new(2,min_node);
        Tree_new(1,node_number) = new_x;
        Tree_new(2,node_number) = new_y;
        
    end      
end

%% Draw the path

current_index = length(Tree_new);
while current_index ~= 1
    pause(0.01)
    plot([Tree_new(1,current_index),Tree_old(1,current_index)],[Tree_new(2,current_index),Tree_old(2,current_index)],'-','LineWidth',0.8*grid_r,'Color','g');    
    plot(Tree_new(1,current_index),Tree_new(2,current_index),'.b','MarkerFaceColor','b','MarkerSize',10*grid_r);
    for i=1:length(Tree_new)
        if Tree_new(1,i) == Tree_old(1,current_index)
            if Tree_new(2,i) == Tree_old(2,current_index)
                current_index = i;
                break
            end
        end
    end
end

%% Function used to find the cloest node
function [angle, min_node] = find_node(random_x ,random_y,Tree_new)

    for i=1:1:length(Tree_new(1,:))
        dx = random_x - Tree_new(1,i);
        dy = random_y - Tree_new(2,i);
        distance(i) = sqrt(dx^2+dy^2);
    end

    [~,min_node] = min(distance);
    angle = atan2(random_y - Tree_new(2,min_node), random_x - Tree_new(1,min_node));
end








































