function draw_path

    % đường ngang số 1 dưới cùng
    x1 = linspace(0,-2500,251);
%     x1 = x1(1:length(x1)-1);
    y1 = 0*x1;
 

    % nửa đường tròn r500
    t = linspace(-90,-270,181);
    x2 = -2500+500*cosd(t);
    y2 = 500+500*sind(t);
    

    % cung tròn r800 ở trên số 1
    cx1_a = -2000 + 800/tand(45+135/2);
    cy1_a = 1000 + 800;
    t = linspace(-90,-45,450);
    curve_1xa = cx1_a + 800*cosd(t);
    curve_1ya = cy1_a + 800*sind(t);
 

    % đường ngang tiếp tuyến 3 đường tròn
    x3 = linspace(-2500,cx1_a,50); 
%     x3 = x3(2:length(x3));
    y3 = 1000+0*x3;


    % cung tròn r800 ở trên số 2
    cx2_a = -1500 + 800/tand(135/2);
    cy2_a = 1500 - 800;
    t = linspace(135,90,451);
    curve_2xa = cx2_a + 800*cosd(t);
    curve_2ya = cy2_a + 800*sind(t);


    % đường xiên 45 độ ở trên (tiếp tuyến với 2 cung r800)
    x4 = linspace(curve_1xa(length(curve_1xa)), curve_2xa(1), 5);
%     x4 = x4(2:length(x4));
    y4 = 3000+x4;
    n4 = length(x4);

    % đường ngang ở trên cùng
    x5 = linspace(cx2_a, 0, 100);
    y5 = 1500 + 0*x5;
    n5 = length(x5);
    
    % cung tròn r800 ở dưới số 1
    cx1_b = -2000 + 800/tand(45+135/2);
    cy1_b = 1000 - 800;
    t = linspace(90,45,450);
    curve_1xb = cx1_b + 800*cosd(t);
    curve_1yb = cy1_b + 800*sind(t);
    
    % cung tròn r800 ở dưới số 2
    cx2_b = -1500 + 800/tand(135/2);
    cy2_b = 500 + 800;
    t = linspace(-135,-90,451);
    curve_2xb = cx2_b + 800*cosd(t);
    curve_2yb = cy2_b + 800*sind(t);


    % đường xiên 45 độ ở dưới (tiếp tuyến với 2 cung r800)
    x4_b = linspace(curve_1xb(length(curve_1xb)), curve_2xb(1), 5);
%     x4_b = x4_b(2:length(x4_b));
    y4_b = -1000-x4_b;
 

    % đường ngang ở giữa
    x5_b = linspace(cx2_b, 0, 100);
    y5_b = 500 + 0*x5_b;
    
    % Chỗ nhận hàng
    t = linspace(-25,25,100);
    x6 = -2000 + 0*t;
    y6 = t;
        
    plot(x1, y1, 'b', x2, y2, 'b', x3, y3, 'b',...
        curve_1xa, curve_1ya, 'r', x4, y4, 'r', curve_2xa, curve_2ya, 'r', x5, y5, 'r',...
        curve_1xb, curve_1yb, 'g', x4_b, y4_b, 'g', curve_2xb, curve_2yb, 'g', x5_b, y5_b, 'g',...
        x6, y6, 'b');
    
    axis equal;
%     hold off;
end