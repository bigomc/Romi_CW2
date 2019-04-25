%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Exploration Algorithm Simulation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

load('MAP.mat')

%DEFINE THE 2-D MAP ARRAY
MAX_X=25;
MAX_Y=25;
MAX_VAL=25;

%This array stores the coordinates of the map and the 
%Objects in each coordinate

%MAP=2*(ones(MAX_X,MAX_Y));
MAP_R=2*(ones(MAX_X,MAX_Y));

% Obtain Obstacle, Target and Robot Position
% Initialize the MAP with input values
% Obstacle=-1,Target = 0,Robot=1,Space=2
j=0;
x_val = 1;
y_val = 1;
axis([1 MAX_X+1 1 MAX_Y+1])
grid on;
grid minor;
hold on;
n=0;%Number of Obstacles

%%

[m,n] = size(MAP);
 
 for i=1:1:m
     for j=1:1:n
         if MAP(i,j)==-1
             plot(i+0.5,j+0.5,'ro');
         end
     end
 end
 
x_init = 2;
y_init = 2;

p = plot(x_init+0.5,y_init+0.5,'bo');

pause(2)

%%
heading = 1;
x = 2;
y = 2;
while x<24
    sensors = check_sensors(x,y,heading,MAP);
    MAP_R = mapping(x,y,heading,sensors,MAP_R);
    [x,y,heading] = move(x,y,heading,sensors);
    %pause(.25);
    set(p,'XData',x+0.5,'YData',y+0.5);
    xlabel(heading,'Color','blue');
    drawnow ;
end

figure(2)
[m,n] = size(MAP);
 
 for i=1:1:m
     for j=1:1:n
         if MAP_R(i,j)==0
             plot(i+0.5,j+0.5,'bo');
         end
         if MAP_R(i,j)==1
             plot(i+0.5,j+0.5,'b*');
         end
         if MAP_R(i,j)==2
             plot(i+0.5,j+0.5,'r*');
         end
         hold on;
     end
 end

% switch heading 
%    case 1   %UP
%       
%    case 2   %DOWN
%       statements
%    case 3   %LEFT
%       statements
%    case 4   %RIGHT
%       statements
%    otherwise
%       statements
% end 

% j=size(exploration_path,1);
%  Plot the Optimal Path!
%  p=plot(exploration_path(j,1),exploration_path(j,2),'bo');
%  j=j-1;
%  for i=j:-1:1
%   pause(.25);
%   set(p,'XData',exploration_path(i,1),'YData',exploration_path(i,2));
%  drawnow ;
%  end
%  plot(Optimal_path(:,1),Optimal_path(:,2));

