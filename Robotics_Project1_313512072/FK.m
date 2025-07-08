% Clear the terminal and variables
clc;
clear;

% Degree and Radius Transformation
% D_to_R = pi/180;
R_to_D = 180/pi;

% DH Model - Puma 560
d = [0, 0, 0, 0, 0, 0];   
a = [120, 250, 260, 0, 0, 0];  
alpha = [-90, 0, 0, -90, 90, 0]; 
theta = [0, 0, 0, 0, 0, 0];   
max_theta = [150, 100, 0, 110, 180, 180];
min_theta = [-150, -30, -120, -110, -180, -180];

% -------------------------------------------------------------------------
% Input joint value
fprintf('Forward kinematics\n');
valid_input = false;
fprintf('Please enter the joint variable (in rad):\n');
input_theta = input('theta1, theta2 , theta3 , theta4 , theta5, theta6 :\n');
while valid_input == false
    out_of_range = 0;
    for dof = 1:length(theta)
        if (min_theta(dof) <= input_theta(dof) && input_theta(dof) <= max_theta(dof))
            theta(dof) = input_theta(dof);
        else
            fprintf('theta%d is out of range\n', dof);
            out_of_range = out_of_range + 1;
        end
    end
    if out_of_range == 0
        valid_input = true;
    end
end

% Initialize transformation matrix
T = eye(4);

% calculate the transformation matrices
for i = 1:6
    A = [cosd(theta(i)), -sind(theta(i)) * cosd(alpha(i)), sind(theta(i)) * sind(alpha(i)), a(i) * cosd(theta(i));
         sind(theta(i)),  cosd(theta(i)) * cosd(alpha(i)), -cosd(theta(i)) * sind(alpha(i)), a(i) * sind(theta(i));
         0,                   sind(alpha(i)),                       cosd(alpha(i)),                       d(i);
         0,                   0,                                          0,                                          1];
    T = T * A;  
end

% Extract rotation and position components
nx = T(1,1); ny = T(2,1); nz = T(3,1);
ox = T(1,2); oy = T(2,2); oz = T(3,2);
ax = T(1,3); ay = T(2,3); az = T(3,3);
px = T(1,4); py = T(2,4); pz = T(3,4);
x = px; y = py; z = pz;

phi = atan2(oz, -nz) * R_to_D;
theta_val = atan2(sqrt(ax^2 + ay^2), az) * R_to_D;
psi = atan2(ay, ax) * R_to_D;
p = [x, y, z, phi, theta_val, psi];


format long;

fprintf('[n o a p]:\n')
disp(T);
fprintf('(x , y , z , phi , theta , psi ): \n');
fprintf('= %.4f %.4f %.4f %.4f %.4f %.4f\n', p(1), p(2), p(3), p(4), p(5), p(6));


