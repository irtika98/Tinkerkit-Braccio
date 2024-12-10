%% Prepare                                                         %%%%%%%
clear
clc


%% Input Joint angles
theta_1 = 0;
theta_2 = pi/2;
theta_3 = pi/2;
theta_4 = 0;
theta_5 = 150;
theta_6 = pi/2;

%% Calculate End Effector Coordinates (my calculations) %%%%%%%

function T_total = six_joint_transformation(DH_params)

    T_total = eye(4);
    
    % Loop through each joint and compute the transformation matrix
    for i = 1:5
        theta = DH_params(i, 1);
        d     = DH_params(i, 2);
        a     = DH_params(i, 3);
        alpha = DH_params(i, 4);
        
        % Compute the transformation matrix for the current joint
        T_i = dh_transformation(theta, d, a, alpha);
        
        % Multiply with the total transformation matrix
        T_total = T_total * T_i;
    end
end

function matrix = dh_transformation(theta_i, d_i, a_i_1, alpha_i_1)

% Define the matrix
matrix = [cos(theta_i), -sin(theta_i), 0, a_i_1; 
          sin(theta_i)*cos(alpha_i_1), cos(theta_i)*cos(alpha_i_1), -sin(alpha_i_1), -sin(alpha_i_1)*d_i;
          sin(theta_i)*sin(alpha_i_1), cos(theta_i)*sin(alpha_i_1), cos(alpha_i_1), cos(alpha_i_1)*d_i; 
          0, 0, 0, 1];


end



% Define DH parameters for 6 joints [theta, d, a, alpha]
DH_params = [theta_1        ,   71.5   ,   0  ,              0;
             theta_2        ,    0     ,   0  ,             pi/2;
             theta_3-(pi/2) ,    0     ,  125 ,              0;
             theta_4        ,    0     ,  125 ,              0;  
             theta_5        ,   190    ,   0  ,             pi/2;
             theta_6        ,    0    ,   0  ,             theta_6;]

% Compute the total transformation matrix
T_total = six_joint_transformation(DH_params);

% Display the resulting total transformation matrix
disp('End Effector Position (Manual):');
disp(T_total)

% Extract x, y, z from the last column of the transformation matrix
x = T_total(1, 4);
y = T_total(2, 4);
z = T_total(3, 4);

% Display x, y, z coordinates
disp(['x: ', num2str(x)])
disp(['y: ', num2str(y)])
disp(['z: ', num2str(z)])

