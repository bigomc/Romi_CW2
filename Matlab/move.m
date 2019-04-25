function [x,y,heading] = move(x,y,heading,sensors)
%Try to mantain the up or down heading
%My previous was up or down?
%Can I continue with my previous? not? head to right and move one step
%Can I go up? Was explored?
%Can I go down? Was explored?
%Can I move to the right? not? move to the previous position -> that means
%rotate 180 and move until a non-explored area, then right


if heading == 1 %UP
    if sensors(2,1) == 0
        y = y + 1;
    elseif sensors(3,1) == 0
        heading = 4;    %RIGHT
        x = x + 1;
    else
        heading = 2;    %DOWN
        y = y - 1;
    end
elseif heading == 2 %DOWN
    if sensors(2,1) == 0
        y = y - 1;
    elseif sensors(1,1) == 0
        heading = 4;    %RIGHT
        x = x + 1;
    else
        heading = 1;    %UP
        y = y + 1;
    end
elseif heading == 3
    if sensors(1,1) == 0
    elseif sensors(3,1) == 0
    elseif sensors(2,1) == 0
    else
    end
elseif heading == 4
    if sensors(1,1) == 0
        heading = 1;
        y = y + 1;
    elseif sensors(3,1) == 0
        heading = 2;
        y = y - 1;
    elseif sensors(2,1) == 0
        x = x + 1;
    else
        heading = 3;
        x = x - 1;
    end
end


end