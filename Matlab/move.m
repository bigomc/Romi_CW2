function [x,y,heading,flag] = move(x,y,heading,sensors,MAP_R, flag)

cells = zeros(1,4);
i_x = x;
i_y = y;
i_h = heading;
%Go to the next unexplored cell
%Try to mantain the up or down heading
%My previous was up or down?
%Can I continue with my previous? not? head to right and move one step
%Can I go up? Was explored?
%Can I go down? Was explored?
%Can I move to the right? not? move to the previous position -> that means
%rotate 180 and move until a non-explored area, then right


%Where am I
%Chech suround> Whera can I go?
%From where can I go? they are already explored?
%YES: look for another NO:go there
%If all are explored return previous cell, and repeat the process

%zero:not available -> obstacle
%one:explored - avaliable
%two:not explored
%three:visited

cells = check_suround(x,y,MAP_R);

av = find(cells(:,3)==1,1);
ne = find(cells(:,3)==2,1);
if av~=0
   if cells(2,3) == 1
        i = 2;
    elseif cells(4,3) == 1
        i = 4;
    else
       i = av;
   end
    x = cells(i,1);
    y = cells(i,2); 
    heading = i;  
elseif ne~=0
    heading = ne;
end

if (x==i_x)&&(y==i_y)&&(heading==i_h)
    [x,y]=check_nearest_available(x,y,MAP_R);
    pause(1);
end
        
        
end