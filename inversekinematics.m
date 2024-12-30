% --- Inverse Kinematics Code ---
clc;
clear;

% Define symbolic variables for joint angles
syms theta1 theta2 theta3 theta4

% Define symbolic variables for DH parameters
a = [0, 500, 430.17, 160];      % link lengths
alpha = [pi/2, 0, 0, 0];        % link twists
d = [486, 0, 0, 0];             % link offsets

% --- Desired End-Effector Pose (Numeric Example) ---
% Replace these with your desired end-effector pose
Px = 60; % Desired x-coordinate of the end-effector
Py = 119.45; % Desired y-coordinate of the end-effector
Pz = -99.33; % Desired z-coordinate of the end-effector
phi = deg2rad(45); % Desired orientation angle about the z-axis (optional)

% --- Solve for theta1 ---
theta1_sol = atan2(Py, Px);

% --- Solve for theta2 and theta3 using geometric relationships ---
% Position of the wrist center
Px_wrist = Px - a(4)*cos(phi);
Py_wrist = Py - a(4)*sin(phi);
Pz_wrist = Pz - d(1);

% Distance from origin to wrist center (projection on the plane)
D = sqrt(Px_wrist^2 + Py_wrist^2);

% Law of Cosines for theta3
cos_theta3 = (D^2 + Pz_wrist^2 - a(2)^2 - a(3)^2) / (2 * a(2) * a(3));
theta3_sol = atan2(sqrt(1 - cos_theta3^2), cos_theta3); % Elbow-up solution

% Solve for theta2
beta = atan2(Pz_wrist, D);
alpha = atan2(a(3) * sin(theta3_sol), a(2) + a(3) * cos(theta3_sol));
theta2_sol = beta - alpha;

% --- Solve for theta4 (end-effector orientation) ---
theta4_sol = phi - theta2_sol - theta3_sol;

% --- Display Results ---
disp('Inverse Kinematics Joint Angles (Radians):');
disp(['Theta1: ', char(theta1_sol)]);
disp(['Theta2: ', char(theta2_sol)]);
disp(['Theta3: ', char(theta3_sol)]);
disp(['Theta4: ', char(theta4_sol)]);

% --- Convert to Degrees for Easier Interpretation ---
theta1_deg = rad2deg(double(theta1_sol));
theta2_deg = rad2deg(double(theta2_sol));
theta3_deg = rad2deg(double(theta3_sol));
theta4_deg = rad2deg(double(theta4_sol));

disp('Inverse Kinematics Joint Angles (Degrees):');
disp(['Theta1: ', num2str(theta1_deg)]);
disp(['Theta2: ', num2str(theta2_deg)]);
disp(['Theta3: ', num2str(theta3_deg)]);
disp(['Theta4: ', num2str(theta4_deg)]);

% --- Plot the Robotic Arm Configuration ---
% Convert joint angles to numeric values
theta1 = double(theta1_sol);
theta2 = double(theta2_sol);
theta3 = double(theta3_sol);
theta4 = double(theta4_sol);

% Calculate joint positions
origin = [0; 0; 0];
joint1 = origin + [0; 0; d(1)];
joint2 = joint1 + [a(2)*cos(theta1)*cos(theta2); a(2)*sin(theta1)*cos(theta2); a(2)*sin(theta2)];
joint3 = joint2 + [a(3)*cos(theta1)*cos(theta2 + theta3); a(3)*sin(theta1)*cos(theta2 + theta3); a(3)*sin(theta2 + theta3)];
end_effector = joint3 + [a(4)*cos(theta1)*cos(theta2 + theta3 + theta4); a(4)*sin(theta1)*cos(theta2 + theta3 + theta4); a(4)*sin(theta2 + theta3 + theta4)];

% Plot the robotic arm
figure;
hold on;
plot3([origin(1), joint1(1)], [origin(2), joint1(2)], [origin(3), joint1(3)], 'r', 'LineWidth', 2); % Link 1
plot3([joint1(1), joint2(1)], [joint1(2), joint2(2)], [joint1(3), joint2(3)], 'g', 'LineWidth', 2); % Link 2
plot3([joint2(1), joint3(1)], [joint2(2), joint3(2)], [joint2(3), joint3(3)], 'b', 'LineWidth', 2); % Link 3
plot3([joint3(1), end_effector(1)], [joint3(2), end_effector(2)], [joint3(3), end_effector(3)], 'k', 'LineWidth', 2); % Link 4
