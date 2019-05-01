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
% k = find(cells(:,3)==1||cells(:,3)==0);
% if k ~=0
%     flag = 0;
% end
% if flag == 0
    for i=1:1:size(cells)
        if cells(i,3) == 2 %Just chance heading dont move
            heading = i;
            break;
        elseif cells(i,3) == 1
            x = cells(i,1);
            y = cells(i,2);  
            heading = i;
            break;
        elseif cells(i,3) == 0

        end
    end
% elseif flag == 1
%     if heading==1
%         x = x - 1;
%     elseif heading==2
%         y = y + 1;
%     elseif heading==3
%         x = x + 1;
%     elseif heading==4
%         y = y - 1;
%    end
% %end

if (x==i_x)&&(y==i_y)&&(heading==i_h)
    [x,y]=check_nearest_available(x,y,MAP_R);
    pause(2);
end

% if (x==i_x)&&(y==i_y)&&(heading==i_h)
%     [x_n,y_n]=check_nearset_available(x,y,MAP_R)
%     flag = 1;
%    if heading==1
%         heading = 3;
%         x = x + 1;
%     elseif heading==2
%         heading = 4;
%         y = y - 1;
%     elseif heading==3
%         heading = 1;
%         x = x - 1;
%     elseif heading==4
%         heading = 2;
%         y = y + 1;
%    end
% end

end