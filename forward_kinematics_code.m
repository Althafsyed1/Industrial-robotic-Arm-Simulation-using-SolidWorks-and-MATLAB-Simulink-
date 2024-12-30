clc;
clear;

% Define symbolic variables for joint angles
syms theta1 theta2 theta3 theta4

% Define symbolic variables for DH parameters
a = [0, 500, 430.17, 160];      % link lengths
alpha = [pi/2, 0, 0, 0];        % link twists (converted to radians)
d = [486, 0, 0, 0];             % link offsets
theta = [theta1, theta2, theta3, theta4]; % joint variables (symbolic)

% DH Transformation Matrix Function (Symbolic)
DH = @(a, alpha, d, theta) ...
    [cos(theta), -sin(theta)*cos(alpha), sin(theta)*sin(alpha), a*cos(theta);
     sin(theta), cos(theta)*cos(alpha), -cos(theta)*sin(alpha), a*sin(theta);
     0, sin(alpha), cos(alpha), d;
     0, 0, 0, 1];

% Compute Symbolic Transformation Matrices
T01 = DH(a(1), alpha(1), d(1), theta(1));
T12 = DH(a(2), alpha(2), d(2), theta(2));
T23 = DH(a(3), alpha(3), d(3), theta(3));
T34 = DH(a(4), alpha(4), d(4), theta(4));

% Forward Kinematics: Compute T04 Symbolically
T04 = T01 * T12 * T23 * T34;

% Simplify the final transformation matrix
T04 = simplify(T04);

disp('Forward Kinematics Transformation Matrix (Symbolic):');
disp(T04);

% --- Input Parameters (Joint Angles in Degrees) ---
theta_values_deg = [-108.94, -23.68, -50.52, 32.22]; % Example joint angles in degrees
theta_values_rad = theta_values_deg * pi / 180; % Convert to radians

% Substitute the input values into the symbolic matrix
T04_numeric = double(subs(T04, [theta1, theta2, theta3, theta4], theta_values_rad));

disp('Forward Kinematics Transformation Matrix (Numerical):');
disp(T04_numeric);
