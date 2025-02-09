clear all

%% Khai báo biến
cum_error_L=0;
cum_error_R=0;
error_prev_L=0;
error_prev_R=0;
feedback_time=0.05;
cx1 = -2000 + 800/tand(45+135/2);
cy1 = 1000 + 800;
cx2 = -1500 + 800/tand(135/2);
cy2 = 1500 - 800;

cx1_b = -2000 + 800/tand(45+135/2);
cy1_b = 1000 - 800;
cx2_b = -1500 + 800/tand(135/2);
cy2_b = 500 + 800;
% d: khoang cach tu tam cam bien den diem chinh giua hai banh xe
% w: vtoc goc cua robot
% vL, vR: van toc dai banh xe trai, phai cua robot
% L: khoang cach giua 2 banh xe
% R: ban kinh banh xe
% half_d_sensor: chieu dai day cam bien
% L_x: chieu dai xe
% W_x: chieu rong xe

L_x = 300;
W_x = 200;
L = 180;
wR = 0;
wL = 0;
w = 0;
v = 0.0;
d = 115;
half_d_sensor = 17*2;
R = 80/2;
x1 = [0, 0, pi]; % vi tri ban dau: x=0, y=0, phi=180 do
X = x1;

x1_prev = x1;
tSamp = 0.05;
tSpan = [0 tSamp];
T = 0:tSamp:22;

e_vel(1:length(T))=0;
e1(1:length(T))=0;
e2(1:length(T))=0;
e3(1:length(T))=0;
wL(1:length(T))=0;
wR(1:length(T))=0;
error(1:length(T))=0;
vRef = 200;

x_s=0;
y_s=0;
S=[x_s;y_s];
x_c=x_s-d*cos(x1(3));

y_c=y_s-d*sin(x1(3));
Xc=x_c; Yc=y_c;
C=[x_c;y_c];
int=0;
intw=0;

% Random màu
if rand(1)<0.5
    color = "green";
else
    color = "red";
end

current_index=1000;
load = 0;
count=0;
time_delay=3.6; %th�?i gian tải hàng lên

% G=a/(s+b) ->
% v=a*duty_cycle*exp(-b*t)+v_prev*exp(-b*t)
kp=0.47; ki=5.26; kd=-0.004;
k1 = 0.05/200*0.5;
k2 = 0.12/200*0.5;
k3 = 2/200*0.5;
pL=0.475; iL=6; dL=-0.003;
pR=0.475; iR=6; dR=-0.003;

duty_cycle_L = 0;
duty_cycle_R = 0;
v_prev = 0;
vR_prev=0;
vL_prev=0;
a=56.39;
b=21.32;
stop=0;
freq_samp = 2;
%% Vòng lặp
for i=2:length(T)
    %% B�? qua để trừ ra th�?i gian lấy hàng
    if (i>current_index && i<=current_index+time_delay/tSamp)
       continue;
    end
    
    %% Tính toán vận tốc hiện tại
    wL(i) = duty_cycle_L*a/b*(1-exp(-b*tSamp))+wL(i-1)*exp(-b*tSamp);
    wR(i) = duty_cycle_R*a/b*(1-exp(-b*tSamp))+wR(i-1)*exp(-b*tSamp);
    
    if (x_s>0)
       wR(i)=0;
       wL(i)=0;
       stop = 1;
    end
    
    vL = wL(i)/60*(2*pi*R);
    vR = wR(i)/60*(2*pi*R);
    
    v_fb = (vL+vR)/2;
    v_avg = (v_fb+v_prev)/2;
    
    w_fb = 0.5*((vR-vL)/L+(vR_prev-vL_prev)/L);

    % Giai tim toa do moi cua tam cam bien
%     [t,y] = ode45(@kinematic,tSpan,x1);
%     x1 = y(length(y),:);
    %% Cập nhật vị trí mới
    dx1 = tSamp*[v_avg*cos(x1(3))-w_fb*d*sin(x1(3)),v_avg*sin(x1(3))+w_fb*d*cos(x1(3)),w_fb];
    x1 = x1+dx1;
    x_s = x1(1);
    y_s = x1(2);
    S=[x_s;y_s];
    C=[x_c;y_c];
    vR_prev=vR;
    vL_prev=vL;
    v_prev = v_fb;
    
    %% Tìm t�?a độ điểm ref
    if color == "red"
        if (x_s>-2500 && y_s<100)
            segment=1;
        end
        if(x_s<=-2500 && y_s<1100)
            segment=2;
        end
        if (x_s>-2500 && y_s<1100 && y_s>900)
            segment=3;
        end
        if (x_s > cx1 && y_s<cy1-800*cosd(45) && y_s>900)
            segment=4;
        end
        if (x_s > cx1+800*sind(45) && y_s >= cy1-800*cosd(45) && y_s < 1270)
            segment=5;
        end
        if (x_s > -1737 && y_s >= 1270 && x_s<=-1160)
            segment=6;
        end
        if (x_s > -1160 && y_s >=1400)
            segment=7;
        end
        
        ref_p = identify_ref_point(S,C, "red",segment);
        if isempty(ref_p)
            break;
        end   
    end
    
    if color == "green"
        if (x_s>-2500 && y_s<100)
            segment=1;
        end
        if(x_s<=-2500 && y_s<1100)
            segment=2;
        end
        if (x_s>-2500 && y_s<1100 && y_s>900)
            segment=3;
        end
        if (x_s > cx1_b && y_s>cy1_b+800*cosd(45) && y_s<1100)
            segment=4;
        end
        if (x_s > cx1_b+800*sind(45) && y_s <= cy1_b+800*cosd(45) && y_s >730)
            segment=5;
        end
        if (x_s > -1737 && y_s >= 400 && x_s<=-1160)
            segment=6;
        end
        if (x_s > -1160 && y_s >=400)
            segment=7;
        end
        
        ref_p = identify_ref_point(S,C, "green",segment);
        if isempty(ref_p)
            break;
        end   
    end
    
    %% Tính sai số
    distance_S = sqrt((x1(1)-x1_prev(1))^2+(x1(2)-x1_prev(2))^2);
    ex = ref_p(1,1) - S(1,1);
    ey = ref_p(2,1) - S(2,1);
    error(i) = sqrt(ex^2+ey^2);
    e1(i) = ex*cos(x1(3))+ey*sin(x1(3));
    noise = 1*rand(1);
    sign_ = rand(1);
    if (sign_<0.5)
        sign_ = -1;
    else
        sign_ = 1;
    end
    e2(i) = (-ex*sin(x1(3))+ey*cos(x1(3))) + sign_*noise;
    if mod(i,1)==0
        e2(i) = e2(i) + noise*sign_*(1-stop);
    end
    if abs(e2(i))>34
        break;
    end

    if (v_fb==0) 
        e3(i)=0;
    else
        e3(i) =  (e2(i)-e2(i-1))/(v_fb*tSamp);
    end

    vRef = 400;
    wRef = e3(i)/tSamp; 
%     if abs(wRef)>pi
%         wRef = pi/2*sign(wRef);
%     end
    %% Tính vận tốc dài và vận tốc góc mong muốn
    v = vRef*cos(e3(i)) + k1*e1(i);
    w =  k2*vRef*e2(i) + 0.05*wRef+ k3*sin(e3(i));%+
    
    %% Vận tốc mong muốn từng động cơ
    expected_wR = 1/2*(w*L+2*v)/(pi*2*R)*60;
    expected_wL = 1/2*(2*v-w*L)/(pi*2*R)*60;
    
    
    %%  PID L motor
    error_rpm_L = expected_wL-wL(i);		
    cum_error_L = cum_error_L + error_rpm_L*feedback_time;
    rate_error = (error_rpm_L-error_prev_L)/feedback_time;

    duty_cycle_L = pL*error_rpm_L + iL*cum_error_L+ dL*rate_error;
    error_prev_L = error_rpm_L;

    if duty_cycle_L<-100
        duty_cycle_L=-100;
    end
    if duty_cycle_L>100
        duty_cycle_L=100;
    end
    
    %%  PID R motor
    error_rpm_R = expected_wR-wR(i);		
    cum_error_R = cum_error_R + error_rpm_R*feedback_time;
    rate_error = (error_rpm_R-error_prev_R)/feedback_time;

    duty_cycle_R = pR*error_rpm_R + iR*cum_error_R+ dR*rate_error;
    error_prev_R = error_rpm_R;

    if duty_cycle_R<-100
        duty_cycle_R=-100;
    end
    if duty_cycle_R>100
        duty_cycle_R=100;
    end
    
    %% Cập nhật mảng t�?a độ cảm biến
    x1_prev = x1; 
    X=[X;x1];
    
    %% Cập nhật t�?a độ điểm chính giữa 2 bánh xe
    x_c=x_s-d*cos(x1(3));
    y_c=y_s-d*sin(x1(3));
   
    Xc=[Xc;x_c];
    Yc=[Yc;y_c];
    
    %% Vẽ 
    if mod(i,freq_samp)==0
        wR_(i/freq_samp) = wR(i);
        wL_(i/freq_samp) = wL(i);
        T_(i/freq_samp) = T(i);
        e2_(i/freq_samp) = e2(i);
        plot(x_s, y_s,'*');
        c_x = Xc(i);
        c_y = Yc(i);

        a1 = -(X(i,2)-Yc(i));
        b1 = X(i,1) - Xc(i);
        syms t_sensor; %tham số phương trình dãy cảm biến

        x_s1 = X(i,1) + a1*t_sensor;
        y_s1 = X(i,2) + b1*t_sensor;
        t_sensor = double(solve(sqrt((X(i,1)-x_s1)^2+(X(i,2)-y_s1)^2)-half_d_sensor, t_sensor));
    %         if ~isempty(t_sensor)
        t_sensor = t_sensor(1);
        x_s1 = X(i,1) + a1*t_sensor;
        y_s1 = X(i,2) + b1*t_sensor;
        x_s2 = X(i,1) - a1*t_sensor;
        y_s2 = X(i,2) - b1*t_sensor;

        % truc banh xe
        x_w1 = Xc(i) + L/2/half_d_sensor*a1*t_sensor;
        y_w1 = Yc(i) + L/2/half_d_sensor*b1*t_sensor;
        x_w2 = Xc(i) - L/2/half_d_sensor*a1*t_sensor;
        y_w2 = Yc(i) - L/2/half_d_sensor*b1*t_sensor;

        % 2 banh xe
        x_w1a = x_w1 + (X(i,1)-Xc(i))*R/d; 
        y_w1a = y_w1 + (X(i,2)-Yc(i))*R/d;
        x_w1b = x_w1 - (X(i,1)-Xc(i))*R/d;
        y_w1b = y_w1 - (X(i,2)-Yc(i))*R/d;

        x_w2a = x_w2 + (X(i,1)-Xc(i))*R/d;
        y_w2a = y_w2 + (X(i,2)-Yc(i))*R/d;
        x_w2b = x_w2 - (X(i,1)-Xc(i))*R/d;
        y_w2b = y_w2 - (X(i,2)-Yc(i))*R/d;

        % Khung bao cua xe
        x1_a = Xc(i) + W_x/2/half_d_sensor*a1*t_sensor + (X(i,1)-Xc(i))/d*L_x/2;
        y1_a = Yc(i) + W_x/2/half_d_sensor*b1*t_sensor + (X(i,2)-Yc(i))/d*L_x/2;
        x1_b = Xc(i) + W_x/2/half_d_sensor*a1*t_sensor - (X(i,1)-Xc(i))/d*L_x/2;
        y1_b = Yc(i) + W_x/2/half_d_sensor*b1*t_sensor - (X(i,2)-Yc(i))/d*L_x/2;

        x2_a = Xc(i) - W_x/2/half_d_sensor*a1*t_sensor + (X(i,1)-Xc(i))/d*L_x/2;
        y2_a = Yc(i) - W_x/2/half_d_sensor*b1*t_sensor + (X(i,2)-Yc(i))/d*L_x/2;
        x2_b = Xc(i) - W_x/2/half_d_sensor*a1*t_sensor - (X(i,1)-Xc(i))/d*L_x/2;
        y2_b = Yc(i) - W_x/2/half_d_sensor*b1*t_sensor - (X(i,2)-Yc(i))/d*L_x/2;

        % 2 banh mat trau
        cx_caster1 = Xc(i) + (X(i,1)-Xc(i))*80/100;
        cy_caster1 = Yc(i) + (X(i,2)-Yc(i))*80/100;

        cx_caster2 = Xc(i) - (X(i,1)-Xc(i))*80/100;
        cy_caster2 = Yc(i) - (X(i,2)-Yc(i))*80/100;

        angle = 0:pi/18:2*pi;
        clf('reset');
        hold on;
        draw_path;   

        plot([X(i,1), Xc(i)], [X(i,2), Yc(i)], 'red'); %Doan SC

        plot([x1_a, x1_b, x2_b, x2_a, x1_a], [y1_a, y1_b, y2_b, y2_a, y1_a], 'b');
        % Day cam bien
        plot([x_s1, x_s2], [y_s1, y_s2]); %sensor
        % Truc noi 2 banh xe
        plot([x_w1, x_w2], [y_w1, y_w2]); %

        %Hai banh xe
        plot([x_w1a, x_w1b], [y_w1a, y_w1b]); %
        plot([x_w2a, x_w2b], [y_w2a, y_w2b]);

        %Hai banh mat trau
        plot(cx_caster1+10*cos(angle), cy_caster1+10*sin(angle));
        plot(cx_caster2+10*cos(angle), cy_caster2+10*sin(angle));
        plot(X(:,1), X(:,2),'.');

        %% Khối hàng
        if (load ==0)
            fill([-2100, -1900, -1900, -2100], [-300, -300, -150, -150], color);
        end
        if (load==1)
            x1g_a = Xc(i) + 150/2/half_d_sensor*a1*t_sensor + (X(i,1)-Xc(i))/d*200/2;
            y1g_a = Yc(i) + 150/2/half_d_sensor*b1*t_sensor + (X(i,2)-Yc(i))/d*200/2;
            x1g_b = Xc(i) + 150/2/half_d_sensor*a1*t_sensor - (X(i,1)-Xc(i))/d*200/2;
            y1g_b = Yc(i) + 150/2/half_d_sensor*b1*t_sensor - (X(i,2)-Yc(i))/d*200/2;

            x2g_a = Xc(i) - 150/2/half_d_sensor*a1*t_sensor + (X(i,1)-Xc(i))/d*200/2;
            y2g_a = Yc(i) - 150/2/half_d_sensor*b1*t_sensor + (X(i,2)-Yc(i))/d*200/2;
            x2g_b = Xc(i) - 150/2/half_d_sensor*a1*t_sensor - (X(i,1)-Xc(i))/d*200/2;
            y2g_b = Yc(i) - 150/2/half_d_sensor*b1*t_sensor - (X(i,2)-Yc(i))/d*200/2;
            fill([x1g_a, x1g_b, x2g_b,x2g_a], [y1g_a, y1g_b, y2g_b,y2g_a], color);
        end
        drawnow;

        %% Khi dừng lấy hàng
        if (load==0 && x_s<-2000)
            current_index=i;
            pause(time_delay);
            for j=current_index+1:(current_index+time_delay/tSamp)
                X=[X;x1];
                Xc=[Xc;x_c];
                Yc=[Yc;y_c];
                e1(j)=e1(current_index);
                e2(j)=e2(current_index);
                e3(j)=e3(current_index);
                wR(j) = 0;
                wL(j) = 0;
                if (mod(j,freq_samp)==0)
                    wR_(j/freq_samp) = wR(j);
                    wL_(j/freq_samp) = wL(j);
                    T_(j/freq_samp) = T(j);
                    e2_(j/freq_samp) = e2(j);
                end
                v=0;
                w=0;
            end
            load=1;
        end
        hold off;   
    end

end

%% Vẽ đồ thị vận tốc, sai số

tiledlayout(2,1)
nexttile
plot(T_, wL_, 'blue', T_, wR_, 'red'); 
xlabel('Time (s)');
xlim([0, 22]);
ylabel('Velocity (rpm)');
legend({'wL','wR'});
nexttile
plot(T_,e2_); title('e2');
xlabel('Time (s)');
xlim([0, 22]);
ylabel('Error (mm)');

% figure; 
% plot(T, wL, 'blue', T, wR, 'red'); 
% xlabel('Time (s)');
% ylabel('Velocity (rpm)');
% legend({'wL','wR'});
% figure; 
% xlim([0, 30]);
% plot(T,e2); title('e2');
% xlabel('Time (s)');
% ylabel('Error (mm)');
