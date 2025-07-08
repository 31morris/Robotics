clc
clear all
close all

% 變數
init_p = [];end_p = [];
target_p = [];start_point = [];end_point = [];
init_px = [];init_py = [];init_pz = [];
end_px = [];end_py = [];end_pz = [];

%位置
p1 = [];p2 = [];p3 = [];p4 = [];p5 = [];p6 = [];

%速度
v1 = [];v2 = [];v3 = [];v4 = [];v5 = [];v6 = [];

%加速度
acc1 = [];acc2 = [];acc3 = [];acc4 = [];acc5 = [];acc6 = [];

% 參數 
sampling_time = 0.002;

%利用project 1的逆向運動學得到ABC關節角度
A = [26.5651 72.6476 -58.9929 -13.6546 -153.4349   90.0000];
B = [-36.8699 25.7996 -79.2267 53.4272 -126.8699 180.0000];
C = [45.0000 12.2384 -88.9084 -13.33 -90.0000 135.0000];

% 定義變數
del_CB = C - B;
del_BA = B - A;

%計算即時狀態
for t = 0.5*-1:sampling_time:0.5
    if t < 0.2*-1
        t = t + 0.5;
        target_p = del_BA*(t/0.5) + A;
        start_point = target_p;
        [X_i, Y_i, Z_i, X_f, Y_f, Z_f] = FK(target_p);

        % 更新初始和最終位置
        init_px = [init_px X_i];init_py = [init_py Y_i];init_pz = [init_pz Z_i];
        end_px = [end_px X_f];end_py = [end_py Y_f];end_pz = [end_pz Z_f];

        % 記錄每個關節的角度值
        p1 = [p1 target_p(1)];p2 = [p2 target_p(2)];p3 = [p3 target_p(3)];
        p4 = [p4 target_p(4)];p5 = [p5 target_p(5)];p6 = [p6 target_p(6)];

        % 計算關節的速度
        joint_v = del_BA/0.5;
        v1 = [v1 joint_v(1)];v2 = [v2 joint_v(2)];v3 = [v3 joint_v(3)];
        v4 = [v4 joint_v(4)];v5 = [v5 joint_v(5)];v6 = [v6 joint_v(6)];

        % 計算關節的加速度
        joint_a = zeros(6,1);
        acc1 = [acc1 joint_a(1)];acc2 = [acc2 joint_a(2)];acc3 = [acc3 joint_a(3)];
        acc4 = [acc4 joint_a(4)];acc5 = [acc5 joint_a(5)];acc6 = [acc6 joint_a(6)];

    elseif t <= 0.2 && t >= 0.2*-1
        del_BA = start_point - B;
        h = (t + 0.2) / (2 * 0.2);
        target_p = ((del_CB * (0.2/0.5) + del_BA)*(2-h)*(h^2) - (2 * del_BA)) * h + start_point;
        end_point = target_p;
        [X_i, Y_i, Z_i, X_f, Y_f, Z_f] = FK(target_p);

        % 更新初始和最終位置
        init_px = [init_px X_i];init_py = [init_py Y_i];init_pz = [init_pz Z_i];
        end_px = [end_px X_f];end_py = [end_py Y_f];end_pz = [end_pz Z_f];
        
        % 記錄每個關節的角度值
        p1 = [p1 target_p(1)];p2 = [p2 target_p(2)];p3 = [p3 target_p(3)];
        p4 = [p4 target_p(4)];p5 = [p5 target_p(5)];p6 = [p6 target_p(6)];
        
        % 計算關節的速度
        joint_v = ((del_CB * (0.2/0.5) + del_BA)*(1.5-h)*2*(h^2) - del_BA)/0.2;
        v1 = [v1 joint_v(1)];v2 = [v2 joint_v(2)];v3 = [v3 joint_v(3)];
        v4 = [v4 joint_v(4)];v5 = [v5 joint_v(5)];v6 = [v6 joint_v(6)];
        
        % 計算關節的加速度
        joint_a = ((del_CB * (0.2/0.5) + del_BA)*(1-h))*(3*h)/(0.2^2);
        acc1 = [acc1 joint_a(1)];acc2 = [acc2 joint_a(2)];acc3 = [acc3 joint_a(3)];
        acc4 = [acc4 joint_a(4)];acc5 = [acc5 joint_a(5)];acc6 = [acc6 joint_a(6)];

    elseif t > 0.2
        t = (t-0.2);
        target_p = del_CB*(t/0.5) + end_point;
        [X_i, Y_i, Z_i, X_f, Y_f, Z_f] = FK(target_p);

        % 更新初始和最終位置
        init_px = [init_px X_i];init_py = [init_py Y_i];init_pz = [init_pz Z_i];
        end_px = [end_px X_f];end_py = [end_py Y_f];end_pz = [end_pz Z_f];
        
        % 記錄每個關節的角度值
        p1 = [p1 target_p(1)];p2 = [p2 target_p(2)];p3 = [p3 target_p(3)];
        p4 = [p4 target_p(4)];p5 = [p5 target_p(5)];p6 = [p6 target_p(6)];

        joint_v = del_CB/0.5;
        v1 = [v1 joint_v(1)];v2 = [v2 joint_v(2)];v3 = [v3 joint_v(3)];
        v4 = [v4 joint_v(4)];v5 = [v5 joint_v(5)];v6 = [v6 joint_v(6)];

        joint_a = zeros(6,1);
        acc1 = [acc1 joint_a(1)];acc2 = [acc2 joint_a(2)];acc3 = [acc3 joint_a(3)];
        acc4 = [acc4 joint_a(4)];acc5 = [acc5 joint_a(5)];acc6 = [acc6 joint_a(6)];
    end
end

%角度
t = 0:sampling_time:1;
figure
subplot(3,2,1);
plot(t,p1);
title('joint1');xticks(0:0.1:1);yticks(-40:20:40)
grid;

subplot(3,2,2);
plot(t,p2);
title('joint2');xticks(0:0.1:1);yticks(0:20:100)
grid;

subplot(3,2,3);
plot(t,p3);
title('joint3');ylabel('angle(degree)');xticks(0:0.1:1);yticks(-110:10:-50);
grid;

subplot(3,2,4);
plot(t,p4);
title('joint4');xticks(0:0.1:1);yticks(-40:20:60);
grid;

subplot(3,2,5);
plot(t,p5);
title('joint5');xlabel('time(sec)');xticks(0:0.1:1);yticks(-180:20:-40);
grid;

subplot(3,2,6);
plot(t,p6);
title('joint6');xlabel('time(sec)');xticks(0:0.1:1);yticks(80:20:180);
grid;

%角速度
t = 0:sampling_time:1;
figure
subplot(3,2,1);
plot(t,v1);
title('joint1');xticks(0:0.1:1);yticks(-110:100:210);
grid;

subplot(3,2,2);
plot(t,v2);
title('joint2');xticks(0:0.1:1);yticks(-100:20:0);
grid;

subplot(3,2,3);
plot(t,v3);
title('joint3');ylabel('angular velocity(degree)');xticks(0:0.1:1);yticks(-40:5:-20);
grid;

subplot(3,2,4);
plot(t,v4);
title('joint4');xticks(0:0.1:1);yticks(-200:100:200);
grid;

subplot(3,2,5);
plot(t,v5);
title('joint5');xlabel('time(sec)');xticks(0:0.1:1);yticks(30:5:80);
grid;

subplot(3,2,6);
plot(t,v6);
title('joint6');xlabel('time(sec)');xticks(0:0.1:1);yticks(-100:100:200);
grid;

%角加速度
t = 0:sampling_time:1;
figure
subplot(3,2,1);
plot(t,acc1);
title('joint1');xticks(0:0.1:1);yticks(0:200:1400);
grid;

subplot(3,2,2);
plot(t,acc2);
title('joint2');xticks(0:0.1:1);yticks(0:50:300);
grid;

subplot(3,2,3);
plot(t,acc3);
title('joint3');ylabel('angular acceleration(degree)');xticks(0:0.1:1);yticks(0:20:80);
grid;

subplot(3,2,4);
plot(t,acc4);
title('joint4');xticks(0:0.1:1);yticks(-1200:200:0);
grid;

subplot(3,2,5);
plot(t,acc5);
title('joint5');xlabel('time(sec)');xticks(0:0.1:1);yticks(0:20:100);
grid;

subplot(3,2,6);
plot(t,acc6);
title('joint6');xlabel('time(sec)');xticks(0:0.1:1);yticks(-1200:200:0);
grid;

%繪製Joint move
figure
plot3(init_px, init_py, init_pz)
xlabel('x(mm)');ylabel('y(mm)');zlabel('z(mm)');
grid;
hold on;
%A點
plot3(400,200,-300,'r*')
text(400,200,-300,'A (400,200,-300)')
%B點
plot3(400,-300,100,'r*')
text(400,-300,100,'B (400,-300,100)')
%C點
plot3(300,300,200,'r*')
text(300,300,200,'C (300,300,200)')

%繪製加速度方向的箭頭
quiver3(init_px, init_py, init_pz, end_px, end_py, end_pz, 'Color', [0, 1, 1])

%順向運動學 
function [init_x,init_y,init_z,end_x,end_y,end_z] = FK(theta)
    theta1 = theta(1);
    theta2 = theta(2);
    theta3 = theta(3);
    theta4 = theta(4);
    theta5 = theta(5);
    theta6 = theta(6);
    
    A1 = [  cosd(theta1),               0,   -sind(theta1),    120*cosd(theta1);
        sind(theta1),               0,    cosd(theta1),    120*sind(theta1);
                       0,              -1,               0,               0;
                       0,               0,               0,               1]; 

    A2 = [  cosd(theta2),   -sind(theta2),               0,    250*cosd(theta2);
            sind(theta2),    cosd(theta2),               0,    250*sind(theta2);
                           0,               0,               1,               0;
                           0,               0,               0,               1];
    
    A3 = [  cosd(theta3),   -sind(theta3),               0,    260*cosd(theta3);
            sind(theta3),    cosd(theta3),               0,    260*sind(theta3);
                           0,               0,               1,               0;
                           0,               0,               0,               1];
    
    A4 = [  cosd(theta4),               0,   -sind(theta4),               0;
            sind(theta4),               0,    cosd(theta4),               0;
                           0,              -1,               0,               0;
                           0,               0,               0,               1];
    
    A5 = [  cosd(theta5),               0,    sind(theta5),               0;
            sind(theta5),               0,   -cosd(theta5),               0;
                           0,               1,               0,               0;
                           0,               0,               0,               1];
    
    A6 = [  cosd(theta6),   -sind(theta6),               0,               0;
            sind(theta6),    cosd(theta6),               0,               0;
                           0,               0,               1,               0;
                           0,               0,               0,               1];
   
    T6 = A1*A2*A3*A4*A5*A6;
    init_x = T6(13);
    init_y = T6(14);
    init_z = T6(15);
    end_x = init_x + T6(9) * 2;
    end_y = init_y + T6(10) * 2;
    end_z = init_z + T6(11) * 2;
end

% 儲存joint_angles.txt
fileID = fopen('joint_angles.txt', 'w');
fprintf(fileID, '\tjoint1\t\tjoint2\t\tjoint3\t\tjoint4\t\tjoint5\t\tjoint6\t\t if out of range\n');
for i = 1:length(p1)
    fprintf(fileID, '%d\t', i);
    fprintf(fileID, '%f\t', p1(i), p2(i), p3(i), p4(i), p5(i), p6(i));
    fprintf(fileID, '\n');
end
fclose(fileID);