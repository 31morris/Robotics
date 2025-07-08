format long

%DH參數
a1 =120; a2 = 250; a3 = 260; 
%關節的限制
joint1_limit = [-150,150];
joint2_limit = [-30,100];
joint3_limit = [-120,0];
joint4_limit = [-110,110];
joint5_limit = [-180,180];
joint6_limit = [-180,180];

fprintf("======================INPUT======================\n");
%輸入矩陣
T6=input('Cartesian point(n o a p):\n');

nx = rad2deg(T6(1,1));ny = rad2deg(T6(2,1));nz = rad2deg(T6(3,1));
ox = rad2deg(T6(1,2));oy = rad2deg(T6(2,2));oz = rad2deg(T6(3,2));
ax = rad2deg(T6(1,3));ay = rad2deg(T6(2,3));az = rad2deg(T6(3,3));
px = T6(1,4);  py = T6(2,4);pz = T6(3,4);

function angle = limit_to_range(angle)
    % 將單一角度限制在 [-180, 180] 範圍內
    if angle > 180
        angle = angle - 360;
    elseif angle < -180
        angle = angle + 360;
    end
end

fprintf("======================OUTPUT======================\n");

%逆向運動學
theta1_1 = atan2d(py, px) ; 
theta3_1 = acosd(((px*cosd(theta1_1)+py*sind(theta1_1)-a1)^2+pz^2-a2^2-a3^2)/(2*a2*a3));
theta2 = atan2d((a2*cosd(theta3_1)+a3),(-250*sind(theta3_1)))-atan2d((px*cosd(theta1_1)+py*sind(theta1_1)-120),(-pz))-theta3_1;
theta4_1 = atan2d(-sind(theta2+theta3_1)*(ax*cosd(theta1_1)+ay*sind(theta1_1))-(az*cosd(theta2+theta3_1)),(cosd(theta2+theta3_1))*(ax*cosd(theta1_1)+ay*sind(theta1_1))-az*sind(theta2+theta3_1));
theta6 = atan2d((nx*cosd(theta1_1)*sind(theta2+theta3_1+theta4_1)+ny*sind(theta1_1)*sind(theta2+theta3_1+theta4_1)+nz*cosd(theta2+theta3_1+theta4_1)), -(ox*cosd(theta1_1)*sind(theta2+theta3_1+theta4_1)+oy*sind(theta1_1)*sind(theta2+theta3_1+theta4_1)+oz*cosd(theta2+theta3_1+theta4_1)));
theta5 = atan2d( (ax*cosd(theta1_1)*cosd(theta2+theta3_1+theta4_1)+ay*sind(theta1_1)*cosd(theta2+theta3_1+theta4_1)-az*sind(theta2+theta3_1+theta4_1)) , (ax*(-sind(theta1_1))+ay*cosd(theta1_1)) );

function [theta1_1, theta2, theta3_1, theta4_1, theta5, theta6] = limit_angles(theta1_1, theta2, theta3_1, theta4_1, theta5, theta6)
    % 限制所有角度到 [-180, 180] 範圍內
    theta1_1 = limit_to_range(theta1_1);
    theta2 = limit_to_range(theta2);
    theta3_1 = limit_to_range(theta3_1);
    theta4_1 = limit_to_range(theta4_1);
    theta5 = limit_to_range(theta5);
    theta6 = limit_to_range(theta6);
end

theta_list1 = [theta1_1 theta2 theta3_1 theta4_1 theta5 theta6];
fprintf('your theta_list1 is:\n');

% 定義關節範圍和角度
joint_limits = [joint1_limit; joint2_limit; joint3_limit; joint4_limit; joint5_limit; joint6_limit];
thetas = [theta1_1, theta2, theta3_1, theta4_1, theta5, theta6];

%檢查範圍
for i = 1:length(thetas)
    if joint_limits(i, 1) >= thetas(i) || thetas(i) >= joint_limits(i, 2)
        fprintf('joint%d is out of range!\n', i);
    end
end

fprintf("%.3f  ", theta_list1);
fprintf("\n");

fprintf("================================\n");

theta1_1 = atan2d(py, px) ; 
theta3_1 = acosd(((px*cosd(theta1_1)+py*sind(theta1_1)-a1)^2+pz^2-a2^2-a3^2)/(2*a2*a3));
theta2 = atan2d((a2*cosd(theta3_1)+a3),(-250*sind(theta3_1)))-atan2d((px*cosd(theta1_1)+py*sind(theta1_1)-120),(-pz))-theta3_1;
theta4_2 = atan2d(-sind(theta2+theta3_1)*(ax*cosd(theta1_1)+ay*sind(theta1_1))-(az*cosd(theta2+theta3_1)),(cosd(theta2+theta3_1))*(ax*cosd(theta1_1)+ay*sind(theta1_1))-az*sind(theta2+theta3_1))+180;
theta6 = atan2d((nx*cosd(theta1_1)*sind(theta2+theta3_1+theta4_2)+ny*sind(theta1_1)*sind(theta2+theta3_1+theta4_2)+nz*cosd(theta2+theta3_1+theta4_2)), -(ox*cosd(theta1_1)*sind(theta2+theta3_1+theta4_2)+oy*sind(theta1_1)*sind(theta2+theta3_1+theta4_2)+oz*cosd(theta2+theta3_1+theta4_2)));
theta5 = atan2d( (ax*cosd(theta1_1)*cosd(theta2+theta3_1+theta4_2)+ay*sind(theta1_1)*cosd(theta2+theta3_1+theta4_2)-az*sind(theta2+theta3_1+theta4_2)) , (ax*(-sind(theta1_1))+ay*cosd(theta1_1)) );

    % 限制所有角度到 [-180, 180] 範圍內
    theta1_1 = limit_to_range(theta1_1);
    theta2 = limit_to_range(theta2);
    theta3_1 = limit_to_range(theta3_1);
    theta4_2 = limit_to_range(theta4_2);
    theta5 = limit_to_range(theta5);
    theta6 = limit_to_range(theta6);

 [theta1_1, theta2, theta3_1, theta4_2, theta5, theta6] = limit_angles(theta1_1, theta2, theta3_1, theta4_2, theta5, theta6);

theta_list2 = [theta1_1 theta2 theta3_1 theta4_2 theta5 theta6];
fprintf('your theta_list2 is:\n');

% 定義關節範圍和角度
joint_limits = [joint1_limit; joint2_limit; joint3_limit; joint4_limit; joint5_limit; joint6_limit];
thetas = [theta1_1, theta2, theta3_1, theta4_2, theta5, theta6];

%檢查範圍
for i = 1:length(thetas)
    if joint_limits(i, 1) >= thetas(i) || thetas(i) >= joint_limits(i, 2)
        fprintf('joint%d is out of range!\n', i);
    end
end

fprintf("%.3f  ", theta_list2);
fprintf("\n");

fprintf("================================\n");

theta1_1 = atan2d(py, px) ; 
theta3_2 = -acosd(((px*cosd(theta1_1)+py*sind(theta1_1)-a1)^2+pz^2-a2^2-a3^2)/(2*a2*a3));
theta2 = atan2d((a2*cosd(theta3_2)+a3),(-250*sind(theta3_2)))-atan2d((px*cosd(theta1_1)+py*sind(theta1_1)-120),(-pz))-theta3_2;
theta4_1 = atan2d(-sind(theta2+theta3_2)*(ax*cosd(theta1_1)+ay*sind(theta1_1))-(az*cosd(theta2+theta3_2)),(cosd(theta2+theta3_2))*(ax*cosd(theta1_1)+ay*sind(theta1_1))-az*sind(theta2+theta3_2));
theta6 = atan2d((nx*cosd(theta1_1)*sind(theta2+theta3_2+theta4_1)+ny*sind(theta1_1)*sind(theta2+theta3_2+theta4_1)+nz*cosd(theta2+theta3_2+theta4_1)), -(ox*cosd(theta1_1)*sind(theta2+theta3_2+theta4_1)+oy*sind(theta1_1)*sind(theta2+theta3_2+theta4_1)+oz*cosd(theta2+theta3_2+theta4_1)));
theta5 = atan2d( (ax*cosd(theta1_1)*cosd(theta2+theta3_2+theta4_1)+ay*sind(theta1_1)*cosd(theta2+theta3_2+theta4_1)-az*sind(theta2+theta3_2+theta4_1)) , (ax*(-sind(theta1_1))+ay*cosd(theta1_1)) );

   % 限制所有角度到 [-180, 180] 範圍內
    theta1_1 = limit_to_range(theta1_1);
    theta2 = limit_to_range(theta2);
    theta3_2 = limit_to_range(theta3_2);
    theta4_1 = limit_to_range(theta4_1);
    theta5 = limit_to_range(theta5);
    theta6 = limit_to_range(theta6);
[theta1_1, theta2, theta3_2, theta4_1, theta5, theta6] = limit_angles(theta1_1, theta2, theta3_2, theta4_1, theta5, theta6);

theta_list3 = [theta1_1 theta2 theta3_2 theta4_1 theta5 theta6];
fprintf('your theta_list3 is:\n');

% 定義關節範圍和角度
joint_limits = [joint1_limit; joint2_limit; joint3_limit; joint4_limit; joint5_limit; joint6_limit];
thetas = [theta1_1, theta2, theta3_2, theta4_1, theta5, theta6];

%檢查範圍
for i = 1:length(thetas)
    if joint_limits(i, 1) >= thetas(i) || thetas(i) >= joint_limits(i, 2)
        fprintf('joint%d is out of range!\n', i);
    end
end

fprintf("%.3f  ", theta_list3);
fprintf("\n");

fprintf("================================\n");

theta1_1 = atan2d(py, px) ; 
theta3_2 = -acosd(((px*cosd(theta1_1)+py*sind(theta1_1)-a1)^2+pz^2-a2^2-a3^2)/(2*a2*a3));
theta2 = atan2d((a2*cosd(theta3_2)+a3),(-250*sind(theta3_2)))-atan2d((px*cosd(theta1_1)+py*sind(theta1_1)-120),(-pz))-theta3_2;

theta4_2 = atan2d(-sind(theta2+theta3_2)*(ax*cosd(theta1_1)+ay*sind(theta1_1))-(az*cosd(theta2+theta3_2)),(cosd(theta2+theta3_2))*(ax*cosd(theta1_1)+ay*sind(theta1_1))-az*sind(theta2+theta3_2))+180;
theta6 = atan2d((nx*cosd(theta1_1)*sind(theta2+theta3_2+theta4_2)+ny*sind(theta1_1)*sind(theta2+theta3_2+theta4_2)+nz*cosd(theta2+theta3_2+theta4_2)), -(ox*cosd(theta1_1)*sind(theta2+theta3_2+theta4_2)+oy*sind(theta1_1)*sind(theta2+theta3_2+theta4_2)+oz*cosd(theta2+theta3_2+theta4_2)));
theta5 = atan2d( (ax*cosd(theta1_1)*cosd(theta2+theta3_2+theta4_2)+ay*sind(theta1_1)*cosd(theta2+theta3_2+theta4_2)-az*sind(theta2+theta3_2+theta4_2)) , (ax*(-sind(theta1_1))+ay*cosd(theta1_1)) );

    % 限制所有角度到 [-180, 180] 範圍內
    theta1_1 = limit_to_range(theta1_1);
    theta2 = limit_to_range(theta2);
    theta3_2 = limit_to_range(theta3_2);
    theta4_2 = limit_to_range(theta4_2);
    theta5 = limit_to_range(theta5);
    theta6 = limit_to_range(theta6);
[theta1_1, theta2, theta3_2, theta4_2, theta5, theta6] = limit_angles(theta1_1, theta2, theta3_2, theta4_2, theta5, theta6);

theta_list4 = [theta1_1 theta2 theta3_2 theta4_2 theta5 theta6];
fprintf('your theta_list4 is:\n');

% 定義關節範圍和角度
joint_limits = [joint1_limit; joint2_limit; joint3_limit; joint4_limit; joint5_limit; joint6_limit];
thetas = [theta1_1, theta2, theta3_2, theta4_2, theta5, theta6];

%檢查範圍
for i = 1:length(thetas)
    if joint_limits(i, 1) >= thetas(i) || thetas(i) >= joint_limits(i, 2)
        fprintf('joint%d is out of range!\n', i);
    end
end

fprintf("%.3f  ", theta_list4);
fprintf("\n");

fprintf("================================\n");

theta1_2 = atan2d(py, px)-180 ; 
theta3_1 = acosd(((px*cosd(theta1_2)+py*sind(theta1_2)-a1)^2+pz^2-a2^2-a3^2)/(2*a2*a3));
theta2 = atan2d((a2*cosd(theta3_1)+a3),(-250*sind(theta3_1)))-atan2d((px*cosd(theta1_2)+py*sind(theta1_2)-120),(-pz))-theta3_1;
theta4_1 = atan2d(-sind(theta2+theta3_1)*(ax*cosd(theta1_2)+ay*sind(theta1_2))-(az*cosd(theta2+theta3_1)),(cosd(theta2+theta3_1))*(ax*cosd(theta1_2)+ay*sind(theta1_2))-az*sind(theta2+theta3_1));
theta6 = atan2d((nx*cosd(theta1_2)*sind(theta2+theta3_1+theta4_1)+ny*sind(theta1_2)*sind(theta2+theta3_1+theta4_1)+nz*cosd(theta2+theta3_1+theta4_1)), -(ox*cosd(theta1_2)*sind(theta2+theta3_1+theta4_1)+oy*sind(theta1_2)*sind(theta2+theta3_1+theta4_1)+oz*cosd(theta2+theta3_1+theta4_1)));
theta5 = atan2d( (ax*cosd(theta1_2)*cosd(theta2+theta3_1+theta4_1)+ay*sind(theta1_2)*cosd(theta2+theta3_1+theta4_1)-az*sind(theta2+theta3_1+theta4_1)) , (ax*(-sind(theta1_2))+ay*cosd(theta1_2)) );

    % 限制所有角度到 [-180, 180] 範圍內
    theta1_2 = limit_to_range(theta1_2);
    theta2 = limit_to_range(theta2);
    theta3_1 = limit_to_range(theta3_1);
    theta4_1 = limit_to_range(theta4_1);
    theta5 = limit_to_range(theta5);
    theta6 = limit_to_range(theta6);
[theta1_2, theta2, theta3_1, theta4_1, theta5, theta6] = limit_angles(theta1_2, theta2, theta3_1, theta4_1, theta5, theta6);

theta_list5 = [theta1_2 theta2 theta3_1 theta4_1 theta5 theta6];
fprintf('your theta_list5 is:\n');

% 定義關節範圍和角度
joint_limits = [joint1_limit; joint2_limit; joint3_limit; joint4_limit; joint5_limit; joint6_limit];
thetas = [theta1_2, theta2, theta3_1, theta4_1, theta5, theta6];

%檢查範圍
for i = 1:length(thetas)
    if joint_limits(i, 1) >= thetas(i) || thetas(i) >= joint_limits(i, 2)
        fprintf('joint%d is out of range!\n', i);
    end
end

fprintf("%.3f  ", theta_list5);
fprintf("\n");

fprintf("================================\n");

theta1_2 = atan2d(py, px)-180 ; 
theta3_1 = acosd(((px*cosd(theta1_2)+py*sind(theta1_2)-a1)^2+pz^2-a2^2-a3^2)/(2*a2*a3));
theta2 = atan2d((a2*cosd(theta3_1)+a3),(-250*sind(theta3_1)))-atan2d((px*cosd(theta1_2)+py*sind(theta1_2)-120),(-pz))-theta3_1;
theta4_2 = atan2d(-sind(theta2+theta3_1)*(ax*cosd(theta1_2)+ay*sind(theta1_2))-(az*cosd(theta2+theta3_1)),(cosd(theta2+theta3_1))*(ax*cosd(theta1_2)+ay*sind(theta1_2))-az*sind(theta2+theta3_1))+180;
theta6 = atan2d((nx*cosd(theta1_2)*sind(theta2+theta3_1+theta4_2)+ny*sind(theta1_2)*sind(theta2+theta3_1+theta4_2)+nz*cosd(theta2+theta3_1+theta4_2)), -(ox*cosd(theta1_2)*sind(theta2+theta3_1+theta4_2)+oy*sind(theta1_2)*sind(theta2+theta3_1+theta4_2)+oz*cosd(theta2+theta3_1+theta4_2)));
theta5 = atan2d( (ax*cosd(theta1_2)*cosd(theta2+theta3_1+theta4_2)+ay*sind(theta1_2)*cosd(theta2+theta3_1+theta4_2)-az*sind(theta2+theta3_1+theta4_2)) , (ax*(-sind(theta1_2))+ay*cosd(theta1_2)) );

    % 限制所有角度到 [-180, 180] 範圍內
    theta1_2 = limit_to_range(theta1_2);
    theta2 = limit_to_range(theta2);
    theta3_1 = limit_to_range(theta3_1);
    theta4_2 = limit_to_range(theta4_2);
    theta5 = limit_to_range(theta5);
    theta6 = limit_to_range(theta6);

 [theta1_2, theta2, theta3_1, theta4_2, theta5, theta6] = limit_angles(theta1_2, theta2, theta3_1, theta4_2, theta5, theta6);

theta_list6 = [theta1_2 theta2 theta3_1 theta4_2 theta5 theta6];
fprintf('your theta_list6 is:\n');

% 定義關節範圍和角度
joint_limits = [joint1_limit; joint2_limit; joint3_limit; joint4_limit; joint5_limit; joint6_limit];
thetas = [theta1_2, theta2, theta3_1, theta4_2, theta5, theta6];

%檢查範圍
for i = 1:length(thetas)
    if joint_limits(i, 1) >= thetas(i) || thetas(i) >= joint_limits(i, 2)
        fprintf('joint%d is out of range!\n', i);
    end
end

fprintf("%.3f  ", theta_list6);
fprintf("\n");

fprintf("================================\n");

theta1_2 = atan2d(py, px)-180 ; 
theta3_2 = -acosd(((px*cosd(theta1_2)+py*sind(theta1_2)-a1)^2+pz^2-a2^2-a3^2)/(2*a2*a3));
theta2 = atan2d((a2*cosd(theta3_2)+a3),(-250*sind(theta3_2)))-atan2d((px*cosd(theta1_2)+py*sind(theta1_2)-120),(-pz))-theta3_2;

theta4_1 = atan2d(-sind(theta2+theta3_2)*(ax*cosd(theta1_2)+ay*sind(theta1_2))-(az*cosd(theta2+theta3_2)),(cosd(theta2+theta3_2))*(ax*cosd(theta1_2)+ay*sind(theta1_2))-az*sind(theta2+theta3_2));
theta6 = atan2d((nx*cosd(theta1_2)*sind(theta2+theta3_2+theta4_1)+ny*sind(theta1_2)*sind(theta2+theta3_2+theta4_1)+nz*cosd(theta2+theta3_2+theta4_1)), -(ox*cosd(theta1_2)*sind(theta2+theta3_2+theta4_1)+oy*sind(theta1_2)*sind(theta2+theta3_2+theta4_1)+oz*cosd(theta2+theta3_2+theta4_1)));
theta5 = atan2d( (ax*cosd(theta1_2)*cosd(theta2+theta3_2+theta4_1)+ay*sind(theta1_2)*cosd(theta2+theta3_2+theta4_1)-az*sind(theta2+theta3_2+theta4_1)) , (ax*(-sind(theta1_2))+ay*cosd(theta1_2)) );

   % 限制所有角度到 [-180, 180] 範圍內
    theta1_2 = limit_to_range(theta1_2);
    theta2 = limit_to_range(theta2);
    theta3_2 = limit_to_range(theta3_2);
    theta4_1 = limit_to_range(theta4_1);
    theta5 = limit_to_range(theta5);
    theta6 = limit_to_range(theta6);
[theta1_2, theta2, theta3_2, theta4_1, theta5, theta6] = limit_angles(theta1_2, theta2, theta3_2, theta4_1, theta5, theta6);

theta_list7 = [theta1_2 theta2 theta3_2 theta4_1 theta5 theta6];
fprintf('your theta_list7 is:\n');

% 定義關節範圍和角度
joint_limits = [joint1_limit; joint2_limit; joint3_limit; joint4_limit; joint5_limit; joint6_limit];
thetas = [theta1_2, theta2, theta3_2, theta4_1, theta5, theta6];

%檢查範圍
for i = 1:length(thetas)
    if joint_limits(i, 1) >= thetas(i) || thetas(i) >= joint_limits(i, 2)
        fprintf('joint%d is out of range!\n', i);
    end
end

fprintf("%.3f  ", theta_list7);
fprintf("\n");
fprintf("================================\n");

theta1_2 = atan2d(py, px)-180 ; 
theta3_2 = -acosd(((px*cosd(theta1_2)+py*sind(theta1_2)-a1)^2+pz^2-a2^2-a3^2)/(2*a2*a3));
theta2 = atan2d((a2*cosd(theta3_2)+a3),(-250*sind(theta3_2)))-atan2d((px*cosd(theta1_2)+py*sind(theta1_2)-120),(-pz))-theta3_2;
theta4_2 = atan2d(-sind(theta2+theta3_2)*(ax*cosd(theta1_2)+ay*sind(theta1_2))-(az*cosd(theta2+theta3_2)),(cosd(theta2+theta3_2))*(ax*cosd(theta1_2)+ay*sind(theta1_2))-az*sind(theta2+theta3_2))+180;
theta6 = atan2d((nx*cosd(theta1_2)*sind(theta2+theta3_2+theta4_2)+ny*sind(theta1_2)*sind(theta2+theta3_2+theta4_2)+nz*cosd(theta2+theta3_2+theta4_2)), -(ox*cosd(theta1_2)*sind(theta2+theta3_2+theta4_2)+oy*sind(theta1_2)*sind(theta2+theta3_2+theta4_2)+oz*cosd(theta2+theta3_2+theta4_2)));
theta5 = atan2d( (ax*cosd(theta1_2)*cosd(theta2+theta3_2+theta4_2)+ay*sind(theta1_2)*cosd(theta2+theta3_2+theta4_2)-az*sind(theta2+theta3_2+theta4_2)) , (ax*(-sind(theta1_2))+ay*cosd(theta1_2)) );

    % 限制所有角度到 [-180, 180] 範圍內
    theta1_2 = limit_to_range(theta1_2);
    theta2 = limit_to_range(theta2);
    theta3_2 = limit_to_range(theta3_2);
    theta4_2 = limit_to_range(theta4_2);
    theta5 = limit_to_range(theta5);
    theta6 = limit_to_range(theta6);
[theta1_2, theta2, theta3_2, theta4_2, theta5, theta6] = limit_angles(theta1_2, theta2, theta3_2, theta4_2, theta5, theta6);

theta_list8 = [theta1_2 theta2 theta3_2 theta4_2 theta5 theta6];
fprintf('your theta_list8 is:\n');

% 定義關節範圍和角度
joint_limits = [joint1_limit; joint2_limit; joint3_limit; joint4_limit; joint5_limit; joint6_limit];
thetas = [theta1_2 theta2 theta3_2 theta4_2 theta5 theta6];

%檢查範圍
for i = 1:length(thetas)
    if joint_limits(i, 1) >= thetas(i) || thetas(i) >= joint_limits(i, 2)
        fprintf('joint%d is out of range!\n', i);
    end
end

fprintf("%.3f  ", theta_list8);
fprintf("\n");