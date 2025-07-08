clc
clear all
close all

%ABC三點(單位mm)

A = [ 0  1  0      400 ;
      0 0  -1    200 ; 
      -1 0    0    -300 ; 
      0  0     0      1 ];  

B = [ 0 0   -1  400 ;
      -1  0  0 -300 ; 
      0   1  0 100; 
      0     0     0      1 ];  

C = [ 1 0  0  300 ;
      0  -1 0  300 ; 
      0   0  -1 200 ; 
      0     0     0      1 ]; 

%逆向運動學
function theta_list = IK(T6)

    % 設置輸出格式為 format long
    format long;

    % DH參數
    a1 = 120; a2 = 250; a3 = 260;

    % 關節的限制
    joint1_limit = [-150, 150];
    joint2_limit = [-30, 100];
    joint3_limit = [-120, 0];
    joint4_limit = [-110, 110];
    joint5_limit = [-180, 180];
    joint6_limit = [-180, 180];

    % 輸入矩陣解析
    nx = rad2deg(T6(1,1)); ny = rad2deg(T6(2,1)); nz = rad2deg(T6(3,1));
    ox = rad2deg(T6(1,2)); oy = rad2deg(T6(2,2)); oz = rad2deg(T6(3,2));
    ax = rad2deg(T6(1,3)); ay = rad2deg(T6(2,3)); az = rad2deg(T6(3,3));
    px = T6(1,4);  py = T6(2,4); pz = T6(3,4);

    % 限制角度範圍的內嵌函數
    function angle = limit_to_range(angle)
        if angle > 180
            angle = angle - 360;
        elseif angle < -180
            angle = angle + 360;
        end
    end

    % 計算各個關節角度
    theta1 = atan2d(py, px);
    theta3 = -acosd(((px*cosd(theta1) + py*sind(theta1) - a1)^2 + pz^2 - a2^2 - a3^2) / (2*a2*a3));
    theta2 = atan2d((a2*cosd(theta3) + a3), (-a2*sind(theta3))) - ...
             atan2d((px*cosd(theta1) + py*sind(theta1) - a1), -pz) - theta3;

    theta4 = atan2d(-sind(theta2+theta3)*(ax*cosd(theta1) + ay*sind(theta1)) - (az*cosd(theta2+theta3)), ...
                      cosd(theta2+theta3)*(ax*cosd(theta1) + ay*sind(theta1)) - az*sind(theta2+theta3)) + 180;
    theta6 = -atan2d((nx*cosd(theta1)*sind(theta2+theta3+theta4) + ny*sind(theta1)*sind(theta2+theta3+theta4) + nz*cosd(theta2+theta3+theta4)), ...
                    -(ox*cosd(theta1)*sind(theta2+theta3+theta4) + oy*sind(theta1)*sind(theta2+theta3+theta4) + oz*cosd(theta2+theta3+theta4)));
    theta5 = atan2d((ax*cosd(theta1)*cosd(theta2+theta3+theta4) + ay*sind(theta1)*cosd(theta2+theta3+theta4) - az*sind(theta2+theta3+theta4)), ...
                    (ax*(-sind(theta1)) + ay*cosd(theta1)));

    % 限制所有角度到 [-180, 180] 範圍內
    theta1 = limit_to_range(theta1);
    theta2 = limit_to_range(theta2);
    theta3 = limit_to_range(theta3);
    theta4 = limit_to_range(theta4);
    theta5 = limit_to_range(theta5);
    theta6 = limit_to_range(theta6);

    % 各關節角度
    theta_list = [theta1, theta2, theta3, theta4, theta5, theta6];

    % 定義關節範圍和角度
    joint_limits = [joint1_limit; joint2_limit; joint3_limit; joint4_limit; joint5_limit; joint6_limit];
    thetas = theta_list;

    % 檢查範圍
    for i = 1:length(thetas)
        if thetas(i) < joint_limits(i, 1) || thetas(i) > joint_limits(i, 2)
            %fprintf('joint%d is out of range!\n', i);
        end
    end

end

% 取得6D位姿的參數（位置和姿態）
[xA ,yA ,zA ,RA ,PA, YA] = find_6D_poses(A);
[xB ,yB ,zB ,RB ,PB, YB] = find_6D_poses(B);
[xC ,yC ,zC ,RC ,PC, YC] = find_6D_poses(C);

% 採樣時間
sampling_time = 0.002;

%計算從A到B1的位移與姿態變化
indexA = 0;   
x = xB - xA; y = yB - yA; z = zB - zA;
R = RB - RA; P = PB - PA; Y = YB - YA;

%使用採樣時間計算A到B1之間每個點的座標與姿態
for t=-0.5:sampling_time:-0.2
    indexA = indexA+1;
    h = (t+0.5)/0.5;
    dx = x*h; dy = y*h; dz = z*h;
    dR = R*h; dP = P*h; dY = Y*h;
    x_A(:,indexA) = xA+dx; y_A(:,indexA) = yA+dy; z_A(:,indexA) =zA+dz; 
    R_A(:,indexA) = RA+dR; P_A(:,indexA) = PA+dP; Y_A(:,indexA) =YA+dY;
end

% 計算A到B1之間每個點的方向
for i = 1:indexA
    orientation_A(:,i) = Orientation(R_A(:,i),P_A(:,i),Y_A(:,i));
end

%計算B2到C之間每個點的座標與姿態
indexC = 0;  
x = xC - xB; y = yC - yB; z = zC - zB;
R = RC - RB; P = PC - PB; Y = YC - YB;

% 使用採樣時間計算B2到C之間每個點的座標與姿態
for t=0.2:sampling_time:0.5
    indexC = indexC+1;
    h = (t)/0.5;
    dx = x*h; dy = y*h; dz = z*h;
    dR = R*h; dP = P*h; dY = Y*h;
    x_C(:,indexC) = xB+dx; y_C(:,indexC) = yB+dy; z_C(:,indexC) = zB+dz; 
    R_C(:,indexC) = RB+dR; P_C(:,indexC) = PB+dP; Y_C(:,indexC) = YB+dY;
end

%計算B2到C之間每個點的方向
for i = 1:indexC
    orientation_C(:,i) = Orientation(R_C(:,i),P_C(:,i),Y_C(:,i));
end

% 曲線B
xA = x_A(:,indexA)-xB; yA = y_A(:,indexA)-yB; zA = z_A(:,indexA)-zB;
RA = R_A(:,indexA)-RB; PA = P_A(:,indexA)-RB; YA = Y_A(:,indexA)-RB;
xC = x_C(:,1)-xB; yC = y_C(:,1)-yB; zC = z_C(:,1)-zB;
RC = R_C(:,1)-RB; PC = P_C(:,1)-PB; YC = Y_C(:,1)-YB;
indexB = 0; 

% 使用採樣時間計算曲線B的每個點
for t=(-0.2+sampling_time):sampling_time:(0.2-sampling_time)
    indexB = indexB+1;
    h = (t+0.2)/(2*0.2);
    x_B(:,indexB) = xB + ((xC+xA)*(2-h)*h^2-2*xA)*h+xA;
    y_B(:,indexB) = yB + ((yC+yA)*(2-h)*h^2-2*yA)*h+yA;
    z_B(:,indexB) = zB + ((zC+zA)*(2-h)*h^2-2*zA)*h+zA;
    R_B(:,indexB) = RB + ((RC+RA)*(2-h)*h^2-2*RA)*h+RA;
    P_B(:,indexB) = PB + ((PC+PA)*(2-h)*h^2-2*PA)*h+PA;
    Y_B(:,indexB) = YB + ((YC+YA)*(2-h)*h^2-2*YA)*h+YA;   
end
% 計算曲線B的每個點的方向
for i = 1:indexB
    orientation_B(:,i) = Orientation(R_B(:,i),P_B(:,i),Y_B(:,i));
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% 位置
X=[x_A x_B x_C]; Y=[y_A y_B y_C]; Z=[z_A z_B z_C];
X1=[x_A x_B]; Y1=[y_A y_B]; Z1=[z_A z_B];
t=0:sampling_time:1;
% 繪製位置曲線
figure
subplot(3,1,1);plot(t,X);
xlabel('time(sec)');ylabel('Position(mm)');
title('x');grid;
subplot(3,1,2);plot(t,Y);
xlabel('time(sec)');ylabel('Position(mm)');
title('y');grid;
subplot(3,1,3);plot(t,Z);
xlabel('time(sec)');ylabel('Position(mm)');
title('z');grid;

% 速度
dt=t(2:501);
dX=diff(X)/sampling_time;
dY=diff(Y)/sampling_time;
dZ=diff(Z)/sampling_time;
% 繪製速度曲線
figure
subplot(3,1,1);plot(dt,dX);
xlabel('time(sec)');ylabel('Velocity(mm/s)');
title('x');grid;
subplot(3,1,2);plot(dt,dY);
xlabel('time(sec)');ylabel('Velocity(mm/s)');
title('y');grid;
subplot(3,1,3);plot(dt,dZ);
xlabel('time(sec)');ylabel('Velocity(mm/s)');
title('z');grid;

% 加速度
dt2=t(3:501);
dX2=diff(dX)/sampling_time;
dY2=diff(dY)/sampling_time;
dZ2=diff(dZ)/sampling_time;
% 繪製加速度曲線
figure
subplot(3,1,1);plot(dt2,dX2);
xlabel('time(sec)');ylabel('Acceleration(mm/s^2)');
title('x');grid;
subplot(3,1,2);plot(dt2,dY2);
xlabel('time(sec)');ylabel('Acceleration(mm/s^2)');
title('y');grid;
subplot(3,1,3);plot(dt2,dZ2);
xlabel('time(sec)');ylabel('Acceleration(mm/s^2)');
title('z');grid;

%3D 路徑視覺化
figure
x_all = [x_A x_B x_C]; y_all = [y_A y_B y_C]; z_all = [z_A z_B z_C]; 
orientation_all = [orientation_A orientation_B orientation_C];
quiver3(x_all,y_all,z_all,orientation_all(1,:),orientation_all(2,:),orientation_all(3,:),'Color', [0, 1, 1]);
xlabel('x(mm)');ylabel('y(mm)');zlabel('z(mm)');

hold on;

scatter3(x_all,y_all,z_all,'b','filled', 'SizeData', 0.5);
plot3(  400, 200, -300, 'r*');  % A
plot3( 400, -300,  100, 'r*');  % B
plot3( 300,  300, 200, 'r*');  % C
hold off;

text(   400, 200, -300,'A( 400,200,-300)');
text(  400, -300,  100,'B(400,-300, 100)');
text( 300,  300, 200,'C(300, 300,200)');
title('3D path of Cartesion Move')

% 將位置和方向數據組合成 4x4 齊次變換矩陣
num_frames = length(x_all);
homogeneous_matrices = zeros(4, 4, num_frames);

% 儲存輸出角度
output_angles = [];

for i = 1:num_frames
    % 取得當前的 X, Y, Z 和 Orientation
    pos = [x_all(i); y_all(i); z_all(i)];
    orient = [orientation_all(1,i), orientation_all(2,i), orientation_all(3,i)];
    
    % 將方向向量正規化為旋轉矩陣的一部分
    z_axis = orient(:) / norm(orient); % 確保方向向量正規化
    % 假設 X 軸與 Z 軸正交，並計算其他軸 (簡化計算)
    arbitrary_vector = [1; 0; 0]; % 任意參考向量
    if abs(dot(z_axis, arbitrary_vector)) > 0.9
        arbitrary_vector = [0; 1; 0]; % 避免平行
    end
    x_axis = cross(arbitrary_vector, z_axis); 
    x_axis = x_axis / norm(x_axis);
    y_axis = cross(z_axis, x_axis); % 確保正交
    
    % 組成旋轉矩陣
    R = [x_axis, y_axis, z_axis];
    
    % 組成齊次變換矩陣
    T = eye(4);
    T(1:3, 1:3) = R;
    T(1:3, 4) = pos;
    homogeneous_matrices(:,:,i) = T;
    

    % 提取 n, o, a, p 作為逆運動學輸入
    n = T(1:3, 1); % 第一列
    o = T(1:3, 2); % 第二列
    a = T(1:3, 3); % 第三列
    p = T(1:3, 4); % 第四列

    % 將 n, o, a, p 組成 4x4 齊次變換矩陣
    T6 = [n, o, a, p; 0, 0, 0, 1]; % 將位置和方向組合成 4x4 齊次變換矩陣

    % 執行逆運動學計算
    theta = IK(T6);  % 傳遞 T6

    % 將結果存儲
    output_angles = [output_angles; theta]; % 去掉轉置操作，確保橫向追加
    %disp(size(output_angles)); % 顯示 output_angles 的行和列數
end

% 將角度結果存成txt檔案
output_filename = 'cartesian_angles.txt';
fileID = fopen('cartesian_angles.txt', 'w');

% 寫入標題
fprintf(fileID, '     joint1    joint2     joint3     joint4     joint5     joint6\n');

% 按行寫入，每行添加行號
for i = 1:size(output_angles, 1)
    fprintf(fileID, '%-4d %.6f %.6f %.6f %.6f %.6f %.6f\n', i, output_angles(i, :));
end

disp(['cartesian_angles已儲存到檔案: ', output_filename]);

% 關閉文件
fclose(fileID);

% 儲存為txt檔案
output_filename = 'homogeneous_matrices.txt';
fileID = fopen(output_filename, 'w');
for i = 1:num_frames
    fprintf(fileID, 'Frame %d:\n', i);
    fprintf(fileID, '%f\t%f\t%f\t%f\n', homogeneous_matrices(:,:,i)');
    fprintf(fileID, '\n');
end
fclose(fileID);

disp(['所有 4x4 齊次變換矩陣已儲存到檔案: ', output_filename]);

% find_6D_poses: find x,y,z,roll,pitch,yaw from a T6 matrix
function [x,y,z,phi,theta,psi] = find_6D_poses(T6)
    % Degree and Radius Transformation
    D_to_R = pi / 180;
    R_to_D = 180 / pi;
   
    nx = T6(1,1); ny = T6(2,1); nz = T6(3,1);
    ox = T6(1,2); oy = T6(2,2); oz = T6(3,2);
    ax = T6(1,3); ay = T6(2,3); az = T6(3,3);
    px = T6(1,4); py = T6(2,4); pz = T6(3,4);
    x = px; y = py; z = pz;
    phi = atan2(ay, ax) * R_to_D;
    theta = atan2(sqrt(ax^2 + ay^2), az) * R_to_D;
    psi = atan2(oz, -nz) * R_to_D;
    % p = [ x, y, z, phi,theta, psi];
end

% Orientation: find the orientation of Z axis from raw pitch yaw
function orientation = Orientation(R,P,Y)
    % Degree and Radius Transformation
    D_to_R = pi / 180;
    R_to_D = 180 / pi;
    R = R * D_to_R;
    P = P * D_to_R;
    Y = Y * D_to_R;
    orientation = [cos(R)*sin(P) sin(R)*sin(P) cos(P)];
end

% 儲存六軸角度數據到txt檔案
output_data = [x_all', y_all', z_all', orientation_all(1,:)', orientation_all(2,:)', orientation_all(3,:)'];
output_filename = 'six_axis_angles.txt';  % 設定儲存的檔案名稱
writematrix(output_data, output_filename, 'Delimiter', 'tab');  % 儲存為Tab分隔的txt檔案
disp(['六軸角度數據已儲存到檔案: ', output_filename]);
