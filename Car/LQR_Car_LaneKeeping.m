function tvlqr_lane_keeping_demo()
    % TVLQR_LANE_KEEPING_DEMO - Time-Varying LQR control demonstration for lane keeping
    %
    % This function demonstrates Time-Varying Linear Quadratic Regulator (TVLQR) control
    % applied to a vehicle lane-keeping system with nonlinear bicycle model dynamics.
    %
    % Usage: tvlqr_lane_keeping_demo()
    
    clc; close all; clear;
    
    % =====================================================================
    % SYSTEM PARAMETERS - Vehicle Parameters
    % =====================================================================
    
    % Vehicle parameters (from your original code)
    params = struct();
    params.Tc = 2;                              % Time constant for throttle dynamics
    params.m = 1558;                            % Vehicle mass (kg)
    params.lf = 1.462884;                       % Distance from CG to front axle (m)
    params.lr = 1.405516;                       % Distance from CG to rear axle (m)
    params.l = params.lf + params.lr;           % Wheelbase (m)
    params.Cf = 1.432394487827058e+05;          % Front tire cornering stiffness (N/rad)
    params.Cr = 2.214094963126969e+05;          % Rear tire cornering stiffness (N/rad)
    params.g = 9.80655;                         % Gravitational acceleration (m/s^2)
    
    % Calculate understeer coefficient
    params.kus = (params.m*params.g*params.lr/(params.l*params.Cf) - ...
                  params.m*params.g*params.lf/(params.l*params.Cr));
    
    % Lane parameters
    params.lane_width = 3.7;                   % Standard lane width (m)
    
    % Simulation parameters
    t_final = 10;                              % Simulation time (s)
    dt = 0.01;                                 % Time step (s)
    
    % Visualization parameters
    car_length = 4.5;                          % Vehicle length (m)
    car_width = 2.0;                           % Vehicle width (m)
    
    % =====================================================================
    % USER INTERFACE
    % =====================================================================
    
    fprintf('=== Time-Varying LQR Lane Keeping Demo ===\n\n');
    
    fprintf('Vehicle Parameters:\n');
    fprintf('  Mass: %.0f kg, Wheelbase: %.2f m\n', params.m, params.l);
    fprintf('  Front/Rear axle distances: %.2f/%.2f m\n', params.lf, params.lr);
    fprintf('  Tire stiffness (F/R): %.0f/%.0f N/rad\n', params.Cf, params.Cr);
    fprintf('  Lane width: %.1f m\n', params.lane_width);
    
    % Get TVLQR cost matrices from user
    [Q, R] = get_tvlqr_costs();
    
    % Get trajectory parameters
    [x_des, T_transition] = get_trajectory_params();
    
    % =====================================================================
    % GET USER INPUT FOR INITIAL CONDITIONS
    % =====================================================================
    
    fprintf('\n=== Initial Conditions Setup ===\n');
    fprintf('State vector: [p_long, p_lat, heading, v_long, steering_angle]\n');
    fprintf('Enter initial conditions (press Enter for default values):\n');
    
    % Get initial longitudinal position
    default_p_long = 0;
    p_long0 = input(sprintf('Initial longitudinal position [%.1f m]: ', default_p_long));
    if isempty(p_long0), p_long0 = default_p_long; end
    
    % Get initial lateral position
    default_p_lat = 0;
    p_lat0 = input(sprintf('Initial lateral position [%.1f m]: ', default_p_lat));
    if isempty(p_lat0), p_lat0 = default_p_lat; end
    
    % Get initial heading (in degrees for user convenience)
    default_heading_deg = -30;
    heading0_deg = input(sprintf('Initial heading [%.1f deg]: ', default_heading_deg));
    if isempty(heading0_deg), heading0_deg = default_heading_deg; end
    heading0_rad = heading0_deg * pi / 180;
    
    % Get initial longitudinal velocity
    default_v_long = 10;
    v_long0 = input(sprintf('Initial longitudinal velocity [%.1f m/s]: ', default_v_long));
    if isempty(v_long0), v_long0 = default_v_long; end
    
    % Get initial steering angle
    default_steering = 0;
    steering0_deg = input(sprintf('Initial steering angle [%.1f deg]: ', default_steering));
    if isempty(steering0_deg), steering0_deg = default_steering; end
    steering0_rad = steering0_deg * pi / 180;
    
    % Package initial conditions [p_long, p_lat, heading, v_long, steering_angle]
    x0 = [p_long0; p_lat0; heading0_rad; v_long0; steering0_rad];
    
    fprintf('\nStarting simulation with initial conditions:\n');
    fprintf('  Longitudinal position: %.2f m\n', x0(1));
    fprintf('  Lateral position: %.2f m\n', x0(2));
    fprintf('  Heading: %.1f deg\n', x0(3) * 180/pi);
    fprintf('  Longitudinal velocity: %.2f m/s\n', x0(4));
    fprintf('  Steering angle: %.1f deg\n', x0(5) * 180/pi);
    
    % =====================================================================
    % CREATE LINEARIZED SYSTEM MATRICES
    % =====================================================================
    
    fprintf('\nCreating symbolic linearized system...\n');
    [A_sym, B_sym] = create_symbolic_system(params);
    
    % =====================================================================
    % RUN SIMULATION
    % =====================================================================
    
    fprintf('Running TVLQR simulation...\n');
    
    % Time vector
    tspan = 0:dt:t_final;
    
    % Simulate system with TVLQR control
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
    [t, X] = ode45(@(t, x) tvlqr_dynamics(t, x, A_sym, B_sym, params, Q, R, x_des, T_transition), ...
                   tspan, x0, options);
    
    % Calculate control inputs
    U = zeros(2, length(t));
    for i = 1:length(t)
        [~, u] = tvlqr_dynamics(t(i), X(i,:)', A_sym, B_sym, params, Q, R, x_des, T_transition);
        U(:, i) = u;
    end
    
    % =====================================================================
    % CREATE PLOTS
    % =====================================================================
    
    fprintf('Creating results plots...\n');
    
    % Main results figure
    fig1 = figure('Position', [100, 100, 1200, 800]);
    set(fig1, 'Name', 'TVLQR Lane Keeping Control Results', 'NumberTitle', 'off');
    
    % Plot 1: States vs time
    subplot(2, 3, 1);
    yyaxis left;
    plot(t, X(:, 3) * 180/pi, 'r-', 'LineWidth', 2);
    hold on;
    plot(t, X(:, 5) * 180/pi, 'b-', 'LineWidth', 2);
    ylabel('Angles (deg)', 'Color', 'k');
    yyaxis right;
    plot(t, X(:, 4), 'g-', 'LineWidth', 2);
    plot(t, X(:, 2), 'k-', 'LineWidth', 2);
    ylabel('Velocity (m/s) / Lateral Pos (m)', 'Color', 'k');
    xlabel('Time (s)');
    title('Vehicle States vs Time');
    legend({'Heading', 'Steering Angle', 'Velocity', 'Lateral Position'}, 'Location', 'best');
    grid on;
    
    % Plot 2: Control inputs vs time
    subplot(2, 3, 2);
    plot(t, U(1, :), 'b-', 'LineWidth', 2);
    hold on;
    plot(t, U(2, :) * 180/pi, 'r-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Control Inputs');
    title('Control Inputs vs Time');
    legend({'Throttle', 'Steering Command (deg)'}, 'Location', 'best');
    grid on;
    
    % Plot 3: Vehicle trajectory (top view)
    subplot(2, 3, 3);
    plot(X(:, 1), X(:, 2), 'b-', 'LineWidth', 2);
    hold on;
    plot(X(1, 1), X(1, 2), 'go', 'MarkerSize', 8, 'MarkerFaceColor', 'g');
    plot(X(end, 1), X(end, 2), 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'r');
    plot([0, max(X(:, 1))], [params.lane_width/2, params.lane_width/2], 'k--', 'LineWidth', 1);
    plot([0, max(X(:, 1))], [-params.lane_width/2, -params.lane_width/2], 'k--', 'LineWidth', 1);
    plot([0, max(X(:, 1))], [0, 0], 'k:', 'LineWidth', 1);
    xlabel('Longitudinal Position (m)');
    ylabel('Lateral Position (m)');
    title('Vehicle Trajectory');
    legend({'Trajectory', 'Start', 'End', 'Lane Boundaries', '', 'Center'}, 'Location', 'best');
    grid on;
    axis equal;
    
    % Plot 4: Lateral position vs time
    subplot(2, 3, 4);
    plot(t, X(:, 2), 'b-', 'LineWidth', 2);
    hold on;
    plot(t, ones(size(t)) * params.lane_width/2, 'r--', 'LineWidth', 1);
    plot(t, ones(size(t)) * (-params.lane_width/2), 'r--', 'LineWidth', 1);
    plot(t, zeros(size(t)), 'k:', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Lateral Position (m)');
    title('Lateral Position vs Time');
    legend({'Vehicle Position', 'Lane Boundaries', '', 'Lane Center'}, 'Location', 'best');
    grid on;
    
    % Plot 5: Heading and steering vs time
    subplot(2, 3, 5);
    plot(t, X(:, 3) * 180/pi, 'r-', 'LineWidth', 2);
    hold on;
    plot(t, X(:, 5) * 180/pi, 'b-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Angle (deg)');
    title('Heading and Steering Angle');
    legend({'Heading', 'Steering Angle'}, 'Location', 'best');
    grid on;
    
    % Plot 6: Velocity vs time
    subplot(2, 3, 6);
    plot(t, X(:, 4), 'g-', 'LineWidth', 2);
    hold on;
    plot(t, ones(size(t)) * x_des(4), 'r--', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Velocity (m/s)');
    title('Longitudinal Velocity');
    legend({'Actual Velocity', 'Target Velocity'}, 'Location', 'best');
    grid on;
    
    sgtitle('TVLQR Lane Keeping Control Analysis', 'FontSize', 16, 'FontWeight', 'bold');
    
    % =====================================================================
    % ANIMATION
    % =====================================================================
    
    fprintf('\nStarting animation...\n');
    fprintf('Press any key to stop the animation.\n');
    
    % Create animation figure
    fig2 = figure('Position', [200, 200, 1200, 600]);
    set(fig2, 'Name', 'TVLQR Animation - Press any key to stop', 'NumberTitle', 'off');
    
    % Set up animation
    view_window = 40;
    max_lateral = max(abs(X(:, 2))) + params.lane_width;
    
    xlim([0, view_window]);
    ylim([-max_lateral, max_lateral]);
    xlabel('Longitudinal Distance (m)', 'FontSize', 12);
    ylabel('Lateral Distance (m)', 'FontSize', 12);
    title('TVLQR Vehicle Lane Keeping - Top View', 'FontSize', 14);
    grid on;
    hold on;
    axis equal;
    
    % Draw lane markings
    lane_center = 0;
    lane_left = params.lane_width/2;
    lane_right = -params.lane_width/2;
    
    plot([0, view_window], [lane_left, lane_left], 'r-', 'LineWidth', 3);
    plot([0, view_window], [lane_right, lane_right], 'r-', 'LineWidth', 3);
    plot([0, view_window], [lane_center, lane_center], 'y--', 'LineWidth', 1);
    
    % Create vehicle shape
    vehicle_corners = create_vehicle_shape(car_length, car_width);
    vehicle_handle = patch('XData', [], 'YData', [], 'FaceColor', 'blue', ...
                          'EdgeColor', 'black', 'LineWidth', 2);
    
    % Set up key press detection
    set(fig2, 'KeyPressFcn', @(src, event) set(src, 'UserData', 'stop'));
    set(fig2, 'UserData', 'running');
    
    % Animation loop
    skip_frames = 5;
    
    while strcmp(get(fig2, 'UserData'), 'running') && ishandle(fig2)
        for i = 1:skip_frames:length(t)
            if ~ishandle(fig2) || ~strcmp(get(fig2, 'UserData'), 'running')
                break;
            end
            
            % Get current state
            car_x = X(i, 1);
            car_y = X(i, 2);
            car_heading = X(i, 3);
            
            % Update vehicle visualization
            rotated_vehicle = rotate_vehicle(vehicle_corners, car_x, car_y, car_heading);
            set(vehicle_handle, 'XData', rotated_vehicle(:, 1), 'YData', rotated_vehicle(:, 2));
            
            % Update view window to follow vehicle
            if car_x > view_window/2
                new_xlim = [car_x - view_window/2, car_x + view_window/2];
                xlim(new_xlim);
            end
            
            % Update title with current info
            title(sprintf('TVLQR Lane Keeping - Time: %.1fs, Lat Pos: %.2fm, Heading: %.1f°', ...
                         t(i), car_y, car_heading * 180/pi), 'FontSize', 14);
            
            drawnow limitrate;
            pause(0.05);
        end
    end
    
    if ishandle(fig2)
        fprintf('\nAnimation stopped by user.\n');
    end
    
    % Performance metrics
    fprintf('\nDemo completed successfully!\n');
    fprintf('Final state: Lat pos = %.3f m, Heading = %.2f deg, Velocity = %.2f m/s\n', ...
            X(end, 2), X(end, 3) * 180/pi, X(end, 4));
    
    fprintf('\nPerformance Metrics:\n');
    fprintf('  Maximum lateral deviation: %.3f m\n', max(abs(X(:, 2))));
    fprintf('  Maximum steering angle: %.2f deg\n', max(abs(X(:, 5))) * 180/pi);
    fprintf('  Final velocity: %.2f m/s (target: %.2f m/s)\n', X(end, 4), x_des(4));
    fprintf('  RMS lateral error: %.3f m\n', sqrt(mean(X(:, 2).^2)));
end

function [Q, R] = get_tvlqr_costs()
    % Get TVLQR cost matrices from user
    
    fprintf('\n=== TVLQR Cost Matrix Setup ===\n');
    fprintf('The Q matrix penalizes state deviations from reference trajectory\n');
    fprintf('The R matrix penalizes control effort\n');
    fprintf('State vector: [p_long, p_lat, heading, v_long, steering_angle]\n\n');
    
    % Default values
    default_q_plong = 0.000001;
    default_q_plat = 10;
    default_q_heading = 10;
    default_q_vlong = 10;
    default_q_steering = 10;
    default_r_throttle = 1;
    default_r_steering = 1;
    
    fprintf('Enter Q matrix diagonal elements (press Enter for defaults):\n');
    
    q1 = input(sprintf('Q(1,1) - Longitudinal position cost [%.6f]: ', default_q_plong));
    if isempty(q1), q1 = default_q_plong; end
    
    q2 = input(sprintf('Q(2,2) - Lateral position cost [%.1f]: ', default_q_plat));
    if isempty(q2), q2 = default_q_plat; end
    
    q3 = input(sprintf('Q(3,3) - Heading cost [%.1f]: ', default_q_heading));
    if isempty(q3), q3 = default_q_heading; end
    
    q4 = input(sprintf('Q(4,4) - Longitudinal velocity cost [%.1f]: ', default_q_vlong));
    if isempty(q4), q4 = default_q_vlong; end
    
    q5 = input(sprintf('Q(5,5) - Steering angle cost [%.1f]: ', default_q_steering));
    if isempty(q5), q5 = default_q_steering; end
    
    fprintf('\nEnter R matrix diagonal elements:\n');
    
    r1 = input(sprintf('R(1,1) - Throttle effort cost [%.1f]: ', default_r_throttle));
    if isempty(r1), r1 = default_r_throttle; end
    
    r2 = input(sprintf('R(2,2) - Steering command effort cost [%.1f]: ', default_r_steering));
    if isempty(r2), r2 = default_r_steering; end
    
    % Create matrices
    Q = diag([q1, q2, q3, q4, q5]);
    R = diag([r1, r2]);
    
    fprintf('\nSelected cost matrices:\n');
    fprintf('Q = diag([%.6f, %.1f, %.1f, %.1f, %.1f])\n', diag(Q));
    fprintf('R = diag([%.1f, %.1f])\n', diag(R));
end

function [x_des, T_transition] = get_trajectory_params()
    % Get desired trajectory parameters
    
    fprintf('\n=== Reference Trajectory Setup ===\n');
    
    % Default desired state
    default_plong_des = 0;
    default_plat_des = 0;
    default_heading_des = 0;
    default_vlong_des = 20;
    default_steering_des = 0;
    default_T = 1.75;
    
    fprintf('Enter desired final state (press Enter for defaults):\n');
    
    plong_des = input(sprintf('Desired longitudinal position [%.1f m]: ', default_plong_des));
    if isempty(plong_des), plong_des = default_plong_des; end
    
    plat_des = input(sprintf('Desired lateral position [%.1f m]: ', default_plat_des));
    if isempty(plat_des), plat_des = default_plat_des; end
    
    heading_des = input(sprintf('Desired heading [%.1f deg]: ', default_heading_des));
    if isempty(heading_des), heading_des = default_heading_des; end
    heading_des = heading_des * pi / 180;
    
    vlong_des = input(sprintf('Desired longitudinal velocity [%.1f m/s]: ', default_vlong_des));
    if isempty(vlong_des), vlong_des = default_vlong_des; end
    
    steering_des = input(sprintf('Desired steering angle [%.1f deg]: ', default_steering_des));
    if isempty(steering_des), steering_des = default_steering_des; end
    steering_des = steering_des * pi / 180;
    
    T_transition = input(sprintf('Transition time [%.2f s]: ', default_T));
    if isempty(T_transition), T_transition = default_T; end
    
    x_des = [plong_des; plat_des; heading_des; vlong_des; steering_des];
    
    fprintf('\nReference trajectory:\n');
    fprintf('  Target state: [%.1f, %.1f, %.1f°, %.1f, %.1f°]\n', ...
            x_des(1), x_des(2), x_des(3)*180/pi, x_des(4), x_des(5)*180/pi);
    fprintf('  Transition time: %.2f s\n', T_transition);
end

function [A_sym, B_sym] = create_symbolic_system(params)
    % Create symbolic linearized system matrices
    
    syms p_long p_lat heading v_long throttle delta_cmd steering_angle real
    
    % Extract parameters
    Tc = params.Tc;
    m = params.m;
    lf = params.lf;
    lr = params.lr;
    l = params.l;
    Cf = params.Cf;
    Cr = params.Cr;
    g = params.g;
    kus = params.kus;
    
    % System dynamics (corrected from your original code)
    yr = tan(steering_angle)*v_long/(l + kus*v_long^2/g);
    v_lat = yr*(lr - m*v_long^2*lr/(Cr*l));
    
    f1 = v_long*cos(heading) - v_lat*sin(heading);
    f2 = v_long*sin(heading) + v_lat*cos(heading);
    f3 = yr;
    f4 = -Tc/m*v_long + Tc*throttle;
    f5 = 0.8*(delta_cmd - steering_angle);
    
    % Create Jacobian matrices
    state_vars = [p_long, p_lat, heading, v_long, steering_angle];
    control_vars = [throttle, delta_cmd];
    
    A_sym = jacobian([f1, f2, f3, f4, f5], state_vars);
    B_sym = jacobian([f1, f2, f3, f4, f5], control_vars);
end

function [dxdt, u] = tvlqr_dynamics(t, x, A_sym, B_sym, params, Q, R, x_des, T_transition)
    % TVLQR dynamics function
    
    % Define symbolic variables
    syms p_long p_lat heading v_long throttle delta_cmd steering_angle real
    
    % Calculate reference trajectory (linear interpolation to desired state)
    x0 = [0; 0; -pi/6; 10; 0];  % Initial state from your original code
    
    if t <= T_transition
        x_bar = x0 + t*(x_des - x0)/T_transition;
    else
        x_bar = x_des;
    end
    
    % Nominal control inputs
    u_acc = 0.1 * (x_des(4) - x(4));
    u_steer = 0.1 * (0 - x(3));
    u_bar = [u_acc; u_steer];
    
    % Substitute values into symbolic matrices
    A_tilde = double(subs(A_sym, ...
        [p_long, p_lat, heading, v_long, steering_angle, throttle, delta_cmd], ...
        [x_bar(1), x_bar(2), x_bar(3), x_bar(4), x_bar(5), u_bar(1), u_bar(2)]));
    
    B_tilde = double(subs(B_sym, ...
        [p_long, p_lat, heading, v_long, steering_angle, throttle, delta_cmd], ...
        [x_bar(1), x_bar(2), x_bar(3), x_bar(4), x_bar(5), u_bar(1), u_bar(2)]));
    
    % LQR control
    [K, ~, ~] = lqr(A_tilde, B_tilde, Q, R);
    delta_x = x - x_bar;
    u = u_bar - K*delta_x;
    
    % Apply constraints
    max_throttle = 3;
    max_del_dot = 8;
    
    u(1) = max(-max_throttle, min(max_throttle, u(1)));
    
    % Calculate dynamics
    dxdt = vehicle_dynamics(x, u, params);
    
    % Apply steering rate constraint
    del_dot = dxdt(5);
    del_dot = max(-max_del_dot, min(max_del_dot, del_dot));
    dxdt(5) = del_dot;
end

function dxdt = vehicle_dynamics(x, u, params)
    % Vehicle dynamics function (cleaned up from your original)
    
    % Extract parameters
    Tc = params.Tc;
    m = params.m;
    lf = params.lf;
    lr = params.lr;
    l = params.l;
    Cf = params.Cf;
    Cr = params.Cr;
    g = params.g;
    kus = params.kus;
    
    % Extract states
    heading = x(3);
    v_long = x(4);
    delta = x(5);
    throttle = u(1);
    delta_cmd = u(2);
    
    % Calculate dynamics
    yr = tan(delta)*v_long/(l + kus*v_long^2/g);
    v_lat = yr*(lr - m*v_long^2*lr/(Cr*l));
    
    f1 = v_long*cos(heading) - v_lat*sin(heading);
    f2 = v_long*sin(heading) + v_lat*cos(heading);
    f3 = yr;
    f4 = -Tc/m*v_long + Tc*throttle;
    f5 = 0.8*(delta_cmd - delta);
    
    dxdt = [f1; f2; f3; f4; f5];
end

function corners = create_vehicle_shape(length, width)
    % Create vehicle shape for visualization
    half_length = length/2;
    half_width = width/2;
    
    corners = [-half_length, -half_width;
                half_length, -half_width;
                half_length,  half_width;
               -half_length,  half_width;
               -half_length, -half_width];
end

function rotated_corners = rotate_vehicle(corners, x, y, yaw)
    % Rotate and translate vehicle shape (fixed from your original)
    
    % Rotation matrix
    R = [cos(yaw), -sin(yaw);
         sin(yaw),  cos(yaw)];
    
    % Rotate corners
    rotated = corners * R';
    
    % Translate
    rotated_corners = rotated + [x, y];
end