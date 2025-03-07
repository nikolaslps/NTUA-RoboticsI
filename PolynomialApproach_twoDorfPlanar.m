%% *** Robot (kinematic) model parameters *** 
clear; 
close all; 

%% Define distance between links 
l0 = 81;  %% in cm 
l1 = 20;
l2 = 60;
l3 = 0; % For inverse
l4 = 14;
l5 = 55;
l6 = 0; % For inverse
l7 = 10;

%% *** Sampling Period *** 
%% *** for the robot motion, kinematic simulation: 
dt = 0.001; %dt = 0.001; i.e. 1 msec)   

%% *** Create (or load from file) reference signals *** 
%% *** DESIRED MOTION PROFILE - TASK SPACE *** 
Tf = 10.0; 	% 10sec duration of motion 
t = dt:dt:2*Tf;

%xd0,td0,yd1: initial/final end-point position --> desired task-space trajectory  
xd_A = 100;	
xd_B = 90; 
yd_A = 50; 
yd_B = -40;  
h = 50;
accel = [0.5, 0.5]; % Acceleration applied for better approach

% Example of desired trajectory : linear segment (x0,y0)-->(x1,y1); Time duration: Tf; 
disp('Initialising Desired Task-Space Trajectory (Motion Profile) ...'); %% 
disp(' ');   

% Go from A to B
x_forward  = double(trajectory(0, Tf, xd_A, xd_B, accel(1)));
y_forward  = double(trajectory(0, Tf, yd_A, yd_B, accel(2)));
% Return from B to A
reverse_x = double(trajectory(Tf, 2*Tf, xd_B, xd_A, -accel(1)));
reverse_y = double(trajectory(Tf, 2*Tf, yd_B, yd_A, -accel(2)));

[xd, yd, zd, ux, uy, uz] = compute_trajectory(Tf, dt, x_forward, y_forward, reverse_x, reverse_y, h);

% Resize in order to be the same length as t
xd = xd(1:length(t));   
yd = yd(1:length(t));  
zd = zd(1:length(t));
ux = ux(1:length(t)); 
uy = uy(1:length(t));
uz = uz(1:length(t));

% Plot the position xd, yd, zd
fig1 = figure;  
subplot(2,3,1); 
plot(t, xd, 'LineWidth', 1.5); 
ylabel('Position xd (cm)'); 
xlabel('Time t (sec)');  
grid on;

subplot(2,3,2); 
plot(t, yd, 'LineWidth', 1.5); 
ylabel('Position yd (cm)'); 
xlabel('Time t (sec)');  
grid on;

subplot(2,3,3); 
plot(t, zd, 'LineWidth', 1.5); 
ylabel('Position zd (cm)'); 
xlabel('Time t (sec)');  
grid on;

% Plot the linear velocity ux, uy, uz
subplot(2,3,4); 
plot(t, ux, 'LineWidth', 1.5); 
ylabel('Linear Velocity ux (cm/sec)'); 
xlabel('Time t (sec)');  
grid on;

subplot(2,3,5); 
plot(t, uy, 'LineWidth', 1.5); 
ylabel('Linear Velocity uy (cm/sec)'); 
xlabel('Time t (sec)');  
grid on;

subplot(2,3,6); 
plot(t, uz, 'LineWidth', 1.5); 
ylabel('Linear Velocity uz (cm/sec)'); 
xlabel('Time t (sec)');  
grid on;

%% ****** KINEMATIC SIMULATION - Main loop ****** 
disp('Kinematic Simulation ...'); %% 
disp(' '); %%  

%% ***** INVESRE KINEMATICS  -->  DESIRED MOTION - JOINT SPACE ***** 
%% Compute the reference joint-motion vectors: 
%% {qd(k,i), i=1,...,n (num of degrees of freedom), with k=1,..., kmax,} 
%% and reference joint (angular) velocities {qd_1(k,i)} 
% Taken from analysis we performed in Part 1

% First DoF
qd(:,1) = atan2(yd(:), xd(:));
% or qd(:,1) = atan2(-yd(:), -xd(:));

theta = atan2(l5 + l7, l4);
delta = sqrt(l4^2 + (l5 + l7)^2);

% Third DoF
qd(:,3) = acos((((yd(:) ./ sin(qd(:,1))) - l1).^2 + (h - l0)^2 - delta^2 - l2^2) / (2 * l2 * delta)) - theta;
alpha = atan2(delta * abs(sin(qd(:,3) + theta)), l2 + delta * cos(qd(:,3) + theta));

% Second DoF
qd(:,2) = atan2((yd(:) ./ sin(qd(:,1))) - l1, h - l0) - alpha;

q3 = qd(:,3);
q2 = qd(:,2);
q1 = qd(:,1);

% If we want to compute q_doti with Inverse Jacobian 
% qd_dot1 = zeros(length(t),1); 
% qd_dot2 = zeros(length(t),1); 
% qd_dot3 = zeros(length(t),1);

% for k = 1:length(t)
%     j = inverse(l0,l1,l2,l3,l4,l5,l6,l7,q1(k),q2(k),q3(k));
%     qd_dot1(k) = j(1,1)*ux(k) + j(1,2)*uy(k) + j(1,3)*uz(k);
%     qd_dot2(k) = j(2,1)*ux(k) + j(2,2)*uy(k) + j(2,3)*uz(k);
%     qd_dot3(k) = j(3,1)*ux(k) + j(3,2)*uy(k) + j(3,3)*uz(k);
% end

% Else, compute angular velocities like this
qd_dot1 = gradient(qd(:,1), dt);  
qd_dot2 = gradient(qd(:,2), dt); 
qd_dot3 = gradient(qd(:,3), dt); 

fig2 = figure; 
subplot(2,3,1); 
plot(t, q1, 'LineWidth', 1.5);  
ylabel('DoF q1 (rad)'); 
xlabel('Time t (sec)');  
grid on;

subplot(2,3,2); 
plot(t, q2, 'LineWidth', 1.5); 
ylabel('DoF q2 (rad)'); 
xlabel('Time t (sec)'); 
grid on;

subplot(2,3,3); 
plot(t, q3, 'LineWidth', 1.5); 
ylabel('DoF q3 (rad)'); 
xlabel('Time t (sec)'); 
grid on;

% Plot the angular velocity qd1_{dot}, qd2_{dot}, qd3_{dot}
subplot(2,3,4); 
plot(t, qd_dot1, 'LineWidth', 1.5); 
ylabel('Angular Velocity of first DoF (rad/sec)'); 
xlabel('Time t (sec)');  
grid on;

subplot(2,3,5); 
plot(t, qd_dot2, 'LineWidth', 1.5); 
ylabel('Angular Velocity of second DoF (rad/sec)'); 
xlabel('Time t (sec)');  
grid on;

subplot(2,3,6); 
plot(t, qd_dot3, 'LineWidth', 1.5); 
ylabel('Angular Velocity of third DoF (rad/sec)'); 
xlabel('Time t (sec)');  
grid on;


%% ***** FORWARD KINEMATICS  JOINT MOTION -->  CARTESIAN POSITIONS ***** 
% We take and produce the P vectors from matrices A(q)
%%(xd1, yd1) : cartesian position of the 1st link's local reference frame
%%A0->1
xd1 = l1 * cos(qd(:,1));   
yd1 = l1 * sin(qd(:,1)); 
zd1 = l0;
%%(xd2, yd2) : cartesian position of the 2nd link's local reference frame
%%A0->2
xd2 = xd1 + l2 * cos(qd(:,1)) .* sin(qd(:,2)) - l3 * sin(qd(:,1));   
yd2 = yd1 + l2 * sin(qd(:,1)) .* sin(qd(:,2)) + l3 * cos(qd(:,1));
zd2 = zd1 + l2 * cos(qd(:,2));

%%(xd3, yd3) : cartesian position of the 3rd link's local reference frame
%%A0->3
xd3 = xd2 + l4 * cos(qd(:,1)) .* sin(qd(:,2)) .* cos(qd(:,3)) + l4 * cos(qd(:,1)) .* cos(qd(:,2)) .* sin(qd(:,3));   
yd3 = yd2 + l4 * sin(qd(:,1)) .* sin(qd(:,2)) .* cos(qd(:,3)) + l4 * sin(qd(:,1)) .* cos(qd(:,2)) .* sin(qd(:,3));
zd3 = zd2 + l4 * cos(qd(:,2)) .* cos(qd(:,3)) - l4 * sin(qd(:,2)) .* sin(qd(:,3));

%%(xdE, ydE) : cartesian position of the last link's local reference frame
%%A0->E
xdE = xd3 + (l5 + l7) * cos(qd(:,1)) .* cos(qd(:,2) + qd(:,3));
ydE = yd3 + (l5 + l7) * sin(qd(:,1)) .* cos(qd(:,2) + qd(:,3));   
zdE = zd3 - (l5 + l7) * sin(qd(:,2) + qd(:,3));


%% *** SAVE and PLOT output data *** %%** use functions plot(...)  
%save;  %% --> save data to 'matlab.mat' file 

%%*** Stick Diagram --> animate robot motion ... (**optional**) 
%% within a for (or while) loop, use periodic plot(...) functions to draw the geometry (current pos)  
%% of the robot, and thus animate its motion ...
 
fig3 = figure; 
axis on 
hold on 
xlabel('x (cm)'); 
ylabel('y (cm)'); 
zlabel('z (cm)');
grid on;

% Initial robot base position
plot3([0], [0], [0], 'ko', 'MarkerSize', 6, 'DisplayName', 'Base');

dtk = 1000; %% plot robot position every dtk samples, to animate its motion 
kmax = Tf / dt + 1;
for tkn = 1:dtk:2*kmax
   tk = mod(tkn,2*kmax-2);
   pause(0.2);
   hold off % display only the motion of the robot
   
   view(3); % Set the perspective to a default 3D view
   axis equal; % To ensure uniform scaling in all axis
   
   plot3(xd, yd, zd, 'rs', 'DisplayName', 'Desired Trajectory'); 
   hold on;
   
   plot3([0, xd1(tk)], [0, yd1(tk)], [0, zd1], 'b-', 'LineWidth', 1.5); % Link 1				
   plot3([xd1(tk), xd2(tk)], [yd1(tk), yd2(tk)], [zd1, zd2(tk)], 'r-', 'LineWidth', 1.5); % Link 2
   plot3([real(xd2(tk)), real(xd3(tk))], [real(yd2(tk)), real(yd3(tk))], [real(zd2(tk)), real(zd3(tk))], 'g-', 'LineWidth', 1.5); % Link 3
   plot3([real(xd3(tk)), real(xdE(tk))], [real(yd3(tk)), real(ydE(tk))], [real(zd3(tk)), real(zdE(tk))], 'm-', 'LineWidth', 1.5); % End-effector
   
   plot3(xd1(tk), yd1(tk), zd1, 'ro', 'MarkerSize', 4, 'DisplayName', 'Joint 1'); 
   plot3(xd2(tk), yd2(tk), zd2(tk), 'bo', 'MarkerSize', 4, 'DisplayName', 'Joint 2'); 
   plot3(real(xd3(tk)), real(yd3(tk)), real(zd3(tk)), 'go', 'MarkerSize', 4, 'DisplayName', 'Joint 3'); 
   plot3(real(xdE(tk)), real(ydE(tk)), real(zdE(tk)), 'k*', 'MarkerSize', 6, 'DisplayName', 'End-Effector'); 

end

%% 
disp('Desired Start Position (A):');
disp([xd_A, yd_A, h]);
disp('Actual Start Position:');
%disp([xdE(1), ydE(1), zdE(1)]);
disp([abs(xdE(1)), abs(ydE(1)), abs(zdE(1))]);

disp('Desired End Position (B):');
disp([xd_B, yd_B, h]);
disp('Actual End Position:');
disp([xdE(kmax), ydE(kmax), zdE(kmax)]);

%%
function [xd, yd, zd, ux, uy, uz] = compute_trajectory(Tf, dt, x_forward, y_forward, reverse_x, reverse_y, h)
    kmax = Tf / dt + 1;
    num_points = 2 * kmax - 1;
    
    xd = zeros(1, num_points);
    yd = zeros(1, num_points);
    zd = h * ones(1, num_points); % zd = h
    ux = zeros(1, num_points);
    uy = zeros(1, num_points);
    uz = zeros(1, num_points); % uz = 0
    
    for k = 1:num_points
        time = (k - 1) * dt;
        
        % Determine the time of each phase in motion
        if time < Tf * 0.1
            coeff_x = x_forward(6:-1:1);
            coeff_y = y_forward(6:-1:1);
        elseif time < Tf * 0.9
            coeff_x = x_forward(8:-1:7);
            coeff_y = y_forward(8:-1:7);
        elseif time < Tf
            coeff_x = x_forward(14:-1:9);
            coeff_y = y_forward(14:-1:9);
        % Now return back
        elseif time < Tf * 1.1
            coeff_x = reverse_x(6:-1:1);
            coeff_y = reverse_y(6:-1:1);
        elseif time < Tf * 1.9
            coeff_x = reverse_x(8:-1:7);
            coeff_y = reverse_y(8:-1:7);
        else
            coeff_x = reverse_x(14:-1:9);
            coeff_y = reverse_y(14:-1:9);
        end
        
        % Now pass to the computation of positions and velocities
        xd(k) = polyval(coeff_x, time);
        ux(k) = polyval(polyder(coeff_x), time);
        yd(k) = polyval(coeff_y, time);
        uy(k) = polyval(polyder(coeff_y), time);
    end
    
    limit_points = min(20000, num_points);
    xd = xd(1:limit_points);
    yd = yd(1:limit_points);
    zd = zd(1:limit_points);
    ux = ux(1:limit_points);
    uy = uy(1:limit_points);
    uz = uz(1:limit_points);
end


function traj = trajectory(t0, tf, xA, xB, g)
    t1 = t0 + (tf - t0) * 0.1;
    t2 = t0 + (tf - t0) * 0.9;
    
    syms a0 a1 a2 a3 a4 a5 b0 b1 c0 c1 c2 c3 c4 c5
    eqns = [
        a0 + a1 * t0 + a2 * t0^2 + a3 * t0^3 + a4 * t0^4 + a5 * t0^5 == xA; % Acceleration phase
        a1 + 2 * a2 * t0 + 3 * a3 * t0^2 + 4 * a4 * t0^3 + 5 * a5 * t0^4 == 0;
        2 * a2 + 6 * a3 * t0 + 12 * a4 * t0^2 + 20 * a5 * t0^3 == g;
        
        a0 + a1 * t1 + a2 * t1^2 + a3 * t1^3 + a4 * t1^4 + a5 * t1^5 == b0 + b1 * t1; % Transition to constant velocity
        a1 + 2 * a2 * t1 + 3 * a3 * t1^2 + 4 * a4 * t1^3 + 5 * a5 * t1^4 == b1;
        2 * a2 + 6 * a3 * t1 + 12 * a4 * t1^2 + 20 * a5 * t1^3 == 0;
        
        c0 + c1 * t2 + c2 * t2^2 + c3 * t2^3 + c4 * t2^4 + c5 * t2^5 == b0 + b1 * t2; % Transition to deceleration 
        c1 + 2 * c2 * t2 + 3 * c3 * t2^2 + 4 * c4 * t2^3 + 5 * c5 * t2^4 == b1;
        2 * c2 + 6 * c3 * t2 + 12 * c4 * t2^2 + 20 * c5 * t2^3 == 0;
        
        c0 + c1 * tf + c2 * tf^2 + c3 * tf^3 + c4 * tf^4 + c5 * tf^5 == xB; % Deceleration Phase
        c1 + 2 * c2 * tf + 3 * c3 * tf^2 + 4 * c4 * tf^3 + 5 * c5 * tf^4 == 0;
        2 * c2 + 6 * c3 * tf + 12 * c4 * tf^2 + 20 * c5 * tf^3 == g;
        
        b0 + b1 * t1 - (a0 + a1 * t0 + a2 * t0^2 + a3 * t0^3 + a4 * t0^4 + a5 * t0^5) - (0.5*(t1-t0)*b1) == 0; % Just got to final position 
        (c0 + c1 * tf + c2 * tf^2 + c3 * tf^3 + c4 * tf^4 + c5 * tf^5) - (b0 + b1 * t2) - (0.5 * (t1 - t0) * b1) == 0; % Final constant velocity
    ];
    
    % Get the coefficients
    [A, B] = equationsToMatrix(eqns, [a0 a1 a2 a3 a4 a5 b0 b1 c0 c1 c2 c3 c4 c5]);
    %disp('Matrix A:');
    %disp(A);
    %disp('Rank of A:');
    %disp(rank(A));

    if det(A) == 0
        error('The coefficient matrix A is singular. Check input parameters or equations.');
    end

    traj = vpa(linsolve(A, B));
end

%% May not be necessary, if q_doti not computed with gradient ...
% Compute the inverse J from differential analysis using forward
% kinematic's matrices
function J_inverse = inverse(l0,l1,l2,l3,l4,l5,l6,l7,q1,q2,q3)

    % From Forward Kinematic analysis, the computed matrices
    A01 = [cos(q1), 0, -sin(q1), cos(q1)*l1; 
           sin(q1), 0, cos(q1), sin(q1)*l1;
           0, -1, 0, l0;
           0, 0, 0, 1
    ];

    A12 = [sin(q2), cos(q2), 0, sin(q2)*l2;
           -cos(q2), sin(q2), 0, -cos(q2)*l2; 
           0, 0, 1, l3; 
           0, 0, 0, 1
    ];

    A23 = [cos(q3), 0, -sin(q3), cos(q3)*l4;
           sin(q3), 0, cos(q3), sin(q3)*l4; 
           0, -1, 0, 0; 
           0, 0, 0, 1
    ];

    A34 = [1, 0, 0, 0;
           0, 0, -1, 0; 
           0, 1, 0, l5; 
           0, 0, 0, 1
    ];
    
    A45 = [1, 0, 0, 0;
           0, 0, 1, 0; 
           0, -1, 0, l6; 
           0, 0, 0, 1
    ];
    
    A5E = [1, 0, 0, 0;
           0, 1, 0, 0; 
           0, 0, 1, l7; 
           0, 0, 0, 1
    ];

    A02 = A01*A12; 
    A03 = A02*A23;
    A04 = A03*A34;
    A05 = A04*A45;
    A0E = A05*A5E;
    
    % From differential analysis, compute the Jacobian matrix
    p0E = A0E(1:3,4);
    p01 = A01(1:3,4);
    p02 = A02(1:3,4);
    
    b0 = [0; 0; 1];
    r0E = p0E;
    JL1 = cross(b0, r0E);
    
    b1 = [-sin(q1); cos(q1); 0];
    r1E = p0E - p01;
    JL2 = cross(b1, r1E);
    
    b2 = [-sin(q1); cos(q1); 0];
    r2E = p0E - p02;
    JL3 = cross(b2, r2E);
    
    JA1 = b0;
    JA2 = b1; 
    JA3 = b2;
    
    J = [JL1, JL2, JL3; 
        JA1, JA2, JA3
    ]; 
    J_inverse = inv(J(1:3,:));

end
