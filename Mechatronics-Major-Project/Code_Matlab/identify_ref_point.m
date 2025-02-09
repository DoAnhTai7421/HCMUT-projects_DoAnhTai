function ref_point = identify_ref_point(S, C, color, segment)
% S: tọa độ cảm biến giữa
% C: tọa độ điểm chính giữa 2 bánh xe
% ref_point: tọa độ giao điểm giữa đường đi qua tâm cảm biến vuông góc
% đường tâm xe với đường ref

cx1 = -2000 + 800/tand(45+135/2);
cy1 = 1000 + 800;
cx2 = -1500 + 800/tand(135/2);
cy2 = 1500 - 800;
cx1_b = -2000 + 800/tand(45+135/2);
cy1_b = 1000 - 800;
cx2_b = -1500 + 800/tand(135/2);
cy2_b = 500 + 800;
syms t;

x = S(1,1) + (C(2,1)-S(2,1))*t;
y = S(2,1) + (S(1,1)-C(1,1))*t;

if color == "red"
    switch segment
        case 1
            t = double(solve(y,t));
        case 2
            t = double(solve(x+2500+sqrt(500^2-(y-500)^2),t));
        case 3
            t = double(solve(y-1000,t));
        case 4
            t = double(solve(y-cy1+sqrt(800^2-(x-cx1)^2),t));
        case 5
            t = double(solve(y-x-(cy1-800*cosd(45))+(cx1+800*sind(45)),t));
        case 6
            t = double(solve(y-cy2-sqrt(800^2-(x-cx2)^2),t));
        otherwise
            t = double(solve(y-1500,t));
    end
end

if color == "green"
    switch segment
        case 1
            t = double(solve(y,t));
        case 2
            t = double(solve(x+2500+sqrt(500^2-(y-500)^2),t));
        case 3
            t = double(solve(y-1000,t));
        case 4
            t = double(solve(y-cy1_b-sqrt(800^2-(x-cx1_b)^2),t));
        case 5
            t = double(solve(y+x-(cy1_b+800*cosd(45))-(cx1_b+800*sind(45)),t));
        case 6
            t = double(solve(y-cy2_b+sqrt(800^2-(x-cx2_b)^2),t));
        otherwise
            t = double(solve(y-500,t));
    end
end
i=1;
while isempty(t)
   t = identify_ref_point(S, C, color, i);
   i=i+1;  
end
if length(t)>1
    id=0;
    for i=1:length(t)  
        x = S(1,1) + (C(2,1)-S(2,1))*t(i);
        y = S(2,1) + (S(1,1)-C(1,1))*t(i);
        distance = sqrt((x-S(1,1))^2+(y-S(2,1))^2);
        if i==1 
            min=distance; 
            id = i;
        end
        if distance<min
            min=distance;
            id = i;
        end
    end
    if id~=0 
        t=t(id); 
    end
end

x = S(1,1) + (C(2,1)-S(2,1))*t;
y = S(2,1) + (S(1,1)-C(1,1))*t;
ref_point = [x;y];
end