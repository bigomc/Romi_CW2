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
axis([1 MAX_X 1 MAX_Y])
grid on;
grid minor;
hold on;
n=0;%Number of Obstacles

%%
% BEGIN Interactive Obstacle, Target, Start Location selection
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
xval=floor(xval);
yval=floor(yval);
xTarget=xval;%X Coordinate of the Target
yTarget=yval;%Y Coordinate of the Target

MAP(xval,yval)=0;%Initialize MAP with location of the target
plot(xval+.5,yval+.5,'gd');
text(xval+1,yval+.5,'Target')

pause(2);
h=msgbox('Select Obstacles using the Left Mouse button,to select the last obstacle use the Right button');
  xlabel('Select Obstacles using the Left Mouse button,to select the last obstacle use the Right button','Color','blue');
uiwait(h,10);
if ishandle(h) == 1
    delete(h);
end
while but == 1
    [xval,yval,but] = ginput(1);
    xval=floor(xval);
    yval=floor(yval);
    MAP(xval,yval)=-1;%Put on the closed list as well
    plot(xval+.5,yval+.5,'ro');
 end%End of While loop
 
%  pause(1);
% 
% h=msgbox('Please Select the Vehicle initial position using the Left Mouse button');
% uiwait(h,5);
% if ishandle(h) == 1
%     delete(h);
% end
% xlabel('Please Select the Vehicle initial position ','Color','black');
% but=0;
% while (but ~= 1) %Repeat until the Left button is not clicked
%     [xval,yval,but]=ginput(1);
%     xval=floor(xval);
%     yval=floor(yval);
% end
% xStart=xval;%Starting Position
% yStart=yval;%Starting Position
% MAP(xval,yval)=1;
%  plot(xval+.5,yval+.5,'bo');
%End of obstacle-Target pickup

x_init = 2;
y_init = 2;

p = plot(x_init+0.5,y_init+0.5,'bo');

pause(2)

exploration_path=[2 0;2 1;2 2;2 3;2 4;2 5];
%%
heading = 1;
x = 2;
y = 2;
while true
    sensors = check_sensors(x,y,heading,MAP);
    MAP_R = mapping(x,y,heading,sensors,MAP_R);
    [x,y,heading] = move(x,y,heading,sensors);
    pause(.25);
    set(p,'XData',x+0.5,'YData',y+0.5);
    xlabel(heading,'Color','blue');
    drawnow ;
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

