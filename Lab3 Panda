%% ME192 Lab 3 - Franka Emika Panda Modified-DH Forward Kinematics
clear; clc; close all

% Modified DH columns:
% alpha(i-1), a(i-1), d(i), theta(i)
alpha = deg2rad([0 -90 90 90 -90 90 90]);
a = [0 0 0 0.0825 -0.0825 0 0.088];
d = [0.333 0 0.316 0 0.384 0 0.107];

% Recorded Panda configuration from the lab image, in degrees
% Order: [J1 J2 J3 J4 J5 J6 J7]
q_deg = [115 -29 0 -116 -11 88 -22];
q = deg2rad(q_deg);

% Manual modified-DH transformations
T07 = eye(4);
for i = 1:7
    ct = cos(q(i)); st = sin(q(i));
    ca = cos(alpha(i)); sa = sin(alpha(i));

    % A_i = Rx(alpha) Tx(a) Rz(theta) Tz(d)
    Ti = [ct,       -st,       0,  a(i);
          ca*st,    ca*ct,   -sa, -d(i)*sa;
          sa*st,    sa*ct,    ca,  d(i)*ca;
          0,          0,       0,  1];

    fprintf('T_%d%d =\n',i-1,i)
    disp(Ti)

    T07 = T07 * Ti;
end

fprintf('Compound T_07 =\n')
disp(T07)

fprintf('Flange position [m] = [%.6f %.6f %.6f]^T\n', ...
    T07(1,4), T07(2,4), T07(3,4))

% The controller image reports the current hand/tool position.
% Add the common 0.103 m Panda hand/tool offset for comparison.
tool_offset = 0.103;
T_tool = [cos(-pi/4) -sin(-pi/4) 0 0;
          sin(-pi/4)  cos(-pi/4) 0 0;
          0           0          1 tool_offset;
          0           0          0 1];
T_hand = T07 * T_tool;

fprintf('Hand/tool position [m] = [%.6f %.6f %.6f]^T\n', ...
    T_hand(1,4), T_hand(2,4), T_hand(3,4))

% Measured current coordinates from the controller image, in mm
p_measured = [-130.77; 368.48; 541.79] / 1000;
p_calculated = T_hand(1:3,4);
position_error = norm(p_measured - p_calculated);
fprintf('Measured position [m] = [%.6f %.6f %.6f]^T\n', ...
    p_measured(1), p_measured(2), p_measured(3))
fprintf('Position error = %.6f m (%.3f mm)\n', ...
    position_error, 1000*position_error)


