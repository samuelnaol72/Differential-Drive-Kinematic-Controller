clear;
close all;
clc;

%{ 
TOPIC:      KINEMATIC CONTROLLER FOR THE MOBILE ROBOT.
NAME:       NAOL SAMUEL EREGA.
STUDENT ID: 20210889
%}

%% Initial pose and parameters

initial_poses = [
    50, 35,  0, -35, -50, -35,  0,   35;
    0,  35, 50,  35,  0,  -35, -50, -35;
    0, 0, 0, 0, 0, 0, 0, 0 
]; % Initial x, y, theta (angle in degree)
% Controllers Parameters for two different controllers
k1 = [2; 5; -2]; % Controller 1 gains (k_rho, k_alpha, k_beta)
k2 = [2; 2; -2]; % Controller 2 gains (k_rho, k_alpha, k_beta)

% Simulation time and time step
T = 100;   
dt = 0.001;   

% Final Poses
final_pose_1 = [0; 0; deg2rad(90)]; 
final_pose_2 = [0; 0; deg2rad(180)];

%% SIMULATION FOR THE FIRST CONTROLLER (k1)

% CASE I: Final pose: [0, 0, 90 deg] 
simulate(initial_poses, final_pose_1, k1, dt, T,'First Controller - Case I: 0,0,90');

% CASE II: Final pose: [0, 0, 180 deg] -> converted to radians
simulate(initial_poses, final_pose_2, k1, dt, T, 'First Controller - Case II: 0,0,180');

%% SIMULATION FOR THE SECOND CONTROLLER (k2)
% CASE I: Final pose: [0, 0, 90 deg]
simulate(initial_poses, final_pose_1, k2, dt, T, 'Second Controller - Case I: 0,0,90');

% CASE II: Final pose: [0, 0, 180 deg]
simulate(initial_poses, final_pose_2, k2, dt, T, 'Second Controller - Case II: 0,0,180');

%% Forward Kinematics Function
function x_dot = FMKinematics(x, u)
    % x is current state [rho; alpha; beta]
    % u is input command
    rho = x(1); 
    alpha = x(2); 
    % Motion model
    M = [-cos(alpha), 0;
          sin(alpha) / rho, -1;
         -sin(alpha) / rho, 0];
    x_dot = M * u;
end

%% Backward Kinematics Function
function x_dot = BMKinematics(x, u)
    % x is current state [rho; alpha; beta]
    % u is an input command
    rho = x(1); 
    alpha = x(2); 

    % Motion model
    M = [cos(alpha), 0;
        -sin(alpha) / rho, 1;
         sin(alpha) / rho, 0];
    x_dot = M * u;
end


%% Simulation Function
function simulate(initial_poses, final_pose, k, dt, T, titl)
    figure;
    k_rho = k(1); 
    k_alph = k(2);
    k_bet = k(3);

    num_robots = size(initial_poses,2);
    N = floor(T/dt);

    % Coordinate trace for each robot
    x_trace = zeros(N, num_robots); 
    y_trace = zeros(N, num_robots);
    theta_trace = zeros(N, num_robots);

    % Iterate for each robot
    for robot_idx = 1:num_robots
        hold on;
        % Initial cartesian coordinates and orientation
        x_init = initial_poses(1, robot_idx);
        y_init = initial_poses(2, robot_idx);
        theta_init = deg2rad(initial_poses(3, robot_idx));

        % Initialize trace
        x_trace(1, robot_idx) = x_init;
        y_trace(1, robot_idx) = y_init;
        theta_trace(1, robot_idx) = theta_init;

        % Initial polar coordinate
        rho_init = norm([x_init, y_init]);
        alpha_init = -theta_init + atan2(final_pose(2)-y_init, final_pose(1)-x_init);
        if alpha_init<=-pi
            while alpha_init<=-pi
                alpha_init=alpha_init+ 2*pi;
            end
        end
        if alpha_init>pi
            while alpha_init>pi
                alpha_init=alpha_init- 2*pi;
            end
        end
        % Determine the movement direction (forward or backward)
        if -pi/2 < alpha_init && alpha_init <= pi/2
            f = @FMKinematics;
            sign_flag=1; 
        else
            f = @BMKinematics;
            sign_flag=-1;
        end
         beta_init = -theta_init - alpha_init+final_pose(3);
        if beta_init<=-pi
            while beta_init<=-pi
                beta_init=beta_init+ 2*pi;
            end
        end
        if beta_init>pi
            while beta_init>pi
                beta_init=beta_init- 2*pi;
            end
        end
         % Initial state: [rho; alpha; beta]
        state = [rho_init; alpha_init; beta_init];
        % Alpha desired and Beta desired initilaization
        if sign_flag==-1
            alpha_desired=pi*(alpha_init/norm(alpha_init));
            beta_desired=pi*(beta_init/norm(beta_init));
            if(isnan(beta_desired))
                beta_desired=-pi;
            end
            if(isnan(alpha_desired))
                alpha_desired=-pi;
            end
        else
            alpha_desired=0;
            beta_desired=0;
        end
        % Simulate for the given time
        for t = 2:N
            v = k_rho * state(1);  % Forward speed
            w = k_alph * (alpha_desired+sign_flag*state(2)) + k_bet * (beta_desired+sign_flag*state(3));  % Rotation speed
            u = [v; w];  % Input vector [v; w]
            state = state +dt * f(state,u);  % Update state (rho, alpha, beta)
            % Update Cartesian position based on v and theta
            x_trace(t, robot_idx) = x_trace(t-1, robot_idx) +sign_flag*v*dt* cos(theta_trace(t-1, robot_idx));
            y_trace(t, robot_idx) = y_trace(t-1, robot_idx) +sign_flag*v*dt* sin(theta_trace(t-1, robot_idx));
            theta_trace(t, robot_idx) = theta_trace(t-1, robot_idx)+sign_flag*w*dt;  % Update orientation
        end
        % Plot the trajectory of the robot
        plot(x_trace(:, robot_idx), y_trace(:, robot_idx));
    end
    title(titl);
    xlabel('X Position');
    ylabel('Y Position');
    grid on;
    hold off;
    % Plot the progress of theta (orientation) over time for each robot
    figure;
    time = linspace(0, T, N);  % Time vector
    for robot_idx = 1:num_robots
        hold on;
        plot(time, rad2deg(theta_trace(:, robot_idx)));  
    end
    
    title([titl ' - Theta Progress']);
    xlabel('Time (s)');
    ylabel('Orientation (Degrees)');
    grid on;
    hold off;
end
