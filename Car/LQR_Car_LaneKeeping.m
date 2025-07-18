function lane_keeping_lqr_demo()
    % LANE_KEEPING_LQR_DEMO - Enhanced LQR control demonstration for lane keeping
    %
    % This function demonstrates Linear Quadratic Regulator (LQR) control
    % applied to a vehicle lane-keeping system with user-configurable costs 
    % and initial conditions.
    %
    % Usage: lane_keeping_lqr_demo()
    
    clc; close all; clear;
    
    % =====================================================================
    % SYSTEM PARAMETERS - Vehicle and Lane Parameters
    % =====================================================================
    
    % Vehicle parameters
    params = struct();
    params.m = 1500;        % Vehicle mass (kg)
    params.Iz = 2500;       % Yaw moment of inertia (kg*m^2)
    params.lf = 1.2;        % Distance from CG to front axle (m)
    params.lr = 1.5;        % Distance from CG to rear axle (m)
    params.Cf = 65000;      % Front tire cornering stiffness (N/rad)
    params.Cr = 65000;      % Rear tire cornering stiffness (N/rad)
    params.v = 25;          % Forward velocity (m/s) - ~55 mph
    
    % Lane parameters
    params.lane_width = 3.7;    % Standard lane width (m)
    
    % Simulation parameters
    t_final = 20;               % Simulation time (s)
    dt = 0.01;                  % Time step (s)
    
    % Visualization parameters
    car_length = 4.5;           % Vehicle length (m)
    car_width = 2.0;            % Vehicle width (m)
    
    % =====================================================================
    % DESIGN LQR CONTROLLER
    % =====================================================================
    
    fprintf('=== Lane Keeping LQR Control Demo ===\n\n');
    
    % Create linearized system matrices
    [A, B] = create_vehicle_model(params);
    
    fprintf('Vehicle Parameters:\n');
    fprintf('  Mass: %.0f kg, Yaw inertia: %.0f kg*m²\n', params.m, params.Iz);
    fprintf('  Wheelbase: %.1f m, Velocity: %.1f m/s (%.1f mph)\n', ...
            params.lf + params.lr, params.v, params.v * 2.237);
    fprintf('  Lane width: %.1f m\n', params.lane_width);
    
    % Get LQR cost matrices from user
    [Q, R] = get_lqr_costs();
    
    % Design LQR controller
    [K, S, E] = lqr(A, B, Q, R);
    
    fprintf('\nLQR Controller Design:\n');
    fprintf('  Control gains K = [%.4f, %.4f, %.4f, %.4f]\n', K);
    fprintf('  System is stable: %s\n', iif(all(real(E) < 0), 'YES', 'NO'));
    
    % =====================================================================
    % GET USER INPUT FOR INITIAL CONDITIONS
    % =====================================================================
    
    fprintf('\n=== Initial Conditions Setup ===\n');
    fprintf('Enter initial conditions (press Enter for default values):\n');
    
    % Get initial lateral position
    default_y = 1.0;
    y0_input = input(sprintf('Initial lateral position [%.1f m]: ', default_y));
    if isempty(y0_input)
        y0_input = default_y;
    end
    
    % Get initial lateral velocity
    default_vy = 0;
    vy0_input = input(sprintf('Initial lateral velocity [%.1f m/s]: ', default_vy));
    if isempty(vy0_input)
        vy0_input = default_vy;
    end
    
    % Get initial yaw angle (in degrees for user convenience)
    default_psi_deg = 5;
    psi0_input = input(sprintf('Initial yaw angle [%.1f deg]: ', default_psi_deg));
    if isempty(psi0_input)
        psi0_input = default_psi_deg;
    end
    psi0_rad = psi0_input * pi / 180;  % Convert to radians
    
    % Get initial yaw rate
    default_r = 0;
    r0_input = input(sprintf('Initial yaw rate [%.1f deg/s]: ', default_r));
    if isempty(r0_input)
        r0_input = default_r;
    end
    r0_rad = r0_input * pi / 180;  % Convert to radians
    
    % Package initial conditions [y, vy, psi, r]
    x0 = [y0_input; vy0_input; psi0_rad; r0_rad];
    
    fprintf('\nStarting simulation with initial conditions:\n');
    fprintf('  Lateral position: %.2f m\n', x0(1));
    fprintf('  Lateral velocity: %.2f m/s\n', x0(2));
    fprintf('  Yaw angle: %.1f deg\n', x0(3) * 180/pi);
    fprintf('  Yaw rate: %.1f deg/s\n', x0(4) * 180/pi);
    
    % =====================================================================
    % RUN SIMULATION
    % =====================================================================
    
    fprintf('\nRunning simulation...\n');
    
    % Time vector
    tspan = 0:dt:t_final;
    
    % Simulate system
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
    [t, X] = ode45(@(t, x) vehicle_dynamics(t, x, K, params), tspan, x0, options);
    
    % Calculate control input (steering angle)
    U = -K * X';  % Steering angle in radians
    U_deg = U * 180/pi;  % Convert to degrees for display
    
    % Calculate longitudinal distance traveled
    x_distance = params.v * t;
    
    % =====================================================================
    % CREATE PLOTS (after simulation completes)
    % =====================================================================
    
    % =====================================================================
    % CONTINUOUS ANIMATION
    % =====================================================================
    
    fprintf('\nStarting animation...\n');
    fprintf('Press any key to stop the animation.\n');
    
    % Create animation figure
    fig2 = figure('Position', [200, 200, 1200, 800]);
    set(fig2, 'Name', 'Lane Keeping Animation - Press any key to stop', 'NumberTitle', 'off');
    
    % Set up main animation axes (top view)
    ax1 = axes('Position', [0.1, 0.3, 0.8, 0.6]);
    axis equal;
    
    % Calculate view window
    view_window = 80;  % meters
    max_lateral = max(abs(X(:, 1))) + params.lane_width;
    
    xlim([0, view_window]);
    ylim([-max_lateral, max_lateral]);
    xlabel('Longitudinal Distance (m)', 'FontSize', 12);
    ylabel('Lateral Distance (m)', 'FontSize', 12);
    title('Vehicle Lane Keeping - Top View', 'FontSize', 14);
    grid on;
    hold on;
    
    % Draw lane markings
    lane_center = 0;
    lane_left = params.lane_width/2;
    lane_right = -params.lane_width/2;
    
    % Lane boundaries
    plot([0, view_window], [lane_left, lane_left], 'r-', 'LineWidth', 3);
    plot([0, view_window], [lane_right, lane_right], 'r-', 'LineWidth', 3);
    plot([0, view_window], [lane_center, lane_center], 'y--', 'LineWidth', 1);
    
    % Draw dashed center line
    dash_length = 3;
    gap_length = 9;
    for dash_start = 0:dash_length+gap_length:view_window
        dash_end = min(dash_start + dash_length, view_window);
        plot([dash_start, dash_end], [lane_center, lane_center], 'w-', 'LineWidth', 2);
    end
    
    % Initialize vehicle (simple rectangle)
    car_x = 0;
    car_y = X(1, 1);
    car_yaw = X(1, 3);
    
    % Create vehicle shape (rectangle)
    vehicle_corners = create_vehicle_shape(car_length, car_width);
    vehicle_handle = patch('XData', [], 'YData', [], 'FaceColor', 'blue', ...
                          'EdgeColor', 'black', 'LineWidth', 2);
    
    % Information panel
    info_panel = uipanel('Title', 'Real-time Information', 'Position', [0.1, 0.05, 0.8, 0.2]);
    
    % Create text displays in organized layout
    time_text = uicontrol('Parent', info_panel, 'Style', 'text', 'String', 'Time: 0.00 s', ...
                         'Position', [50, 100, 200, 30], 'FontSize', 14, 'FontWeight', 'bold');
    
    lat_pos_text = uicontrol('Parent', info_panel, 'Style', 'text', 'String', 'Lateral Position: 0.00 m', ...
                            'Position', [50, 70, 200, 20], 'FontSize', 11);
    lat_vel_text = uicontrol('Parent', info_panel, 'Style', 'text', 'String', 'Lateral Velocity: 0.00 m/s', ...
                            'Position', [50, 50, 200, 20], 'FontSize', 11);
    
    yaw_angle_text = uicontrol('Parent', info_panel, 'Style', 'text', 'String', 'Yaw Angle: 0.0°', ...
                              'Position', [300, 70, 150, 20], 'FontSize', 11);
    yaw_rate_text = uicontrol('Parent', info_panel, 'Style', 'text', 'String', 'Yaw Rate: 0.0°/s', ...
                             'Position', [300, 50, 150, 20], 'FontSize', 11);
    
    steer_text = uicontrol('Parent', info_panel, 'Style', 'text', 'String', 'Steering: 0.0°', ...
                          'Position', [500, 70, 150, 20], 'FontSize', 11);
    
    speed_text = uicontrol('Parent', info_panel, 'Style', 'text', 'String', ...
                          sprintf('Speed: %.1f m/s (%.1f mph)', params.v, params.v * 2.237), ...
                          'Position', [500, 50, 200, 20], 'FontSize', 11);
    
    % Cost matrix display
    cost_text = uicontrol('Parent', info_panel, 'Style', 'text', ...
                         'String', sprintf('Q = diag([%.1f, %.1f, %.1f, %.1f]), R = %.1f', ...
                                         diag(Q), R), ...
                         'Position', [50, 15, 600, 20], 'FontSize', 10);
    
    % Animation loop
    skip_frames = 3;
    
    % Set up key press detection
    set(fig2, 'KeyPressFcn', @(src, event) set(src, 'UserData', 'stop'));
    set(fig2, 'UserData', 'running');
    
    while strcmp(get(fig2, 'UserData'), 'running') && ishandle(fig2)
        for i = 1:skip_frames:length(t)
            if ~ishandle(fig2) || ~strcmp(get(fig2, 'UserData'), 'running')
                break;
            end
            
            % Get current state
            current_time = t(i);
            lat_pos = X(i, 1);
            lat_vel = X(i, 2);
            yaw_angle = X(i, 3);
            yaw_rate = X(i, 4);
            steering = U_deg(i);
            
            % Vehicle position
            car_x = x_distance(i);
            car_y = lat_pos;
            
            % Update vehicle visualization
            rotated_vehicle = rotate_vehicle(vehicle_corners, car_x, car_y, yaw_angle);
            set(vehicle_handle, 'XData', rotated_vehicle(:, 1), 'YData', rotated_vehicle(:, 2));
            
            % Update view window to follow vehicle
            if car_x > view_window/2
                new_xlim = [car_x - view_window/2, car_x + view_window/2];
                xlim(new_xlim);
                
                % Redraw lane markings for new view
                delete(findobj(ax1, 'Type', 'line'));
                hold on;
                plot(new_xlim, [lane_left, lane_left], 'r-', 'LineWidth', 3);
                plot(new_xlim, [lane_right, lane_right], 'r-', 'LineWidth', 3);
                plot(new_xlim, [lane_center, lane_center], 'y--', 'LineWidth', 1);
                
                % Redraw dashed center line
                for dash_start = new_xlim(1):dash_length+gap_length:new_xlim(2)
                    dash_end = min(dash_start + dash_length, new_xlim(2));
                    plot([dash_start, dash_end], [lane_center, lane_center], 'w-', 'LineWidth', 2);
                end
            end
            
            % Update information displays
            set(time_text, 'String', sprintf('Time: %.2f s', current_time));
            set(lat_pos_text, 'String', sprintf('Lateral Position: %.3f m', lat_pos));
            set(lat_vel_text, 'String', sprintf('Lateral Velocity: %.3f m/s', lat_vel));
            set(yaw_angle_text, 'String', sprintf('Yaw Angle: %.2f°', yaw_angle * 180/pi));
            set(yaw_rate_text, 'String', sprintf('Yaw Rate: %.2f°/s', yaw_rate * 180/pi));
            set(steer_text, 'String', sprintf('Steering: %.2f°', steering));
            
            drawnow limitrate;
            pause(0.025);
        end
    end
    
    if ishandle(fig2)
        fprintf('\nAnimation stopped by user.\n');
    end
    
    % Wait for animation to finish, then show results
    fprintf('Creating results plots...\n');
    
    % Main results figure - simplified
    fig1 = figure('Position', [100, 100, 1000, 600]);
    set(fig1, 'Name', 'Lane Keeping LQR Control Results', 'NumberTitle', 'off');
    
    % Plot 1: Lateral position vs time
    subplot(2, 2, 1);
    plot(t, X(:, 1), 'b-', 'LineWidth', 2);
    hold on;
    plot(t, ones(size(t)) * params.lane_width/2, 'r--', 'LineWidth', 1);
    plot(t, ones(size(t)) * (-params.lane_width/2), 'r--', 'LineWidth', 1);
    plot(t, zeros(size(t)), 'k:', 'LineWidth', 1);
    xlabel('Time (s)');
    ylabel('Lateral Position (m)');
    title('Lateral Position vs Time');
    grid on;
    legend({'Vehicle Position', 'Lane Boundaries', '', 'Lane Center'}, 'Location', 'best');
    
    % Plot 2: Yaw angle vs time
    subplot(2, 2, 2);
    plot(t, X(:, 3) * 180/pi, 'r-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Yaw Angle (deg)');
    title('Yaw Angle vs Time');
    grid on;
    
    % Plot 3: Steering input vs time
    subplot(2, 2, 3);
    plot(t, U_deg, 'g-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Steering Angle (deg)');
    title('Control Input vs Time');
    grid on;
    
    % Plot 4: Vehicle path (bird's eye view)
    subplot(2, 2, 4);
    plot(x_distance, X(:, 1), 'b-', 'LineWidth', 2);
    hold on;
    plot(x_distance, ones(size(x_distance)) * params.lane_width/2, 'r--', 'LineWidth', 2);
    plot(x_distance, ones(size(x_distance)) * (-params.lane_width/2), 'r--', 'LineWidth', 2);
    plot(x_distance, zeros(size(x_distance)), 'k:', 'LineWidth', 1);
    xlabel('Longitudinal Distance (m)');
    ylabel('Lateral Position (m)');
    title('Vehicle Path');
    legend({'Vehicle Path', 'Lane Boundaries', '', 'Lane Center'}, 'Location', 'best');
    grid on;
    
    sgtitle('Lane Keeping LQR Control Analysis', 'FontSize', 14, 'FontWeight', 'bold');
    
    fprintf('\nDemo completed successfully!\n');
    fprintf('Final state: Lateral position = %.3f m, Yaw angle = %.2f deg\n', ...
            X(end, 1), X(end, 3) * 180/pi);
    
    % Performance metrics
    fprintf('\nPerformance Metrics:\n');
    fprintf('  Maximum lateral deviation: %.3f m\n', max(abs(X(:, 1))));
    fprintf('  Maximum steering angle: %.2f deg\n', max(abs(U_deg)));
    fprintf('  Settling time (within 0.1m): %.2f s\n', ...
            find_settling_time(t, X(:, 1), 0.1));
    fprintf('  RMS lateral error: %.3f m\n', sqrt(mean(X(:, 1).^2)));
end

function [Q, R] = get_lqr_costs()
    % Get LQR cost matrices from user with explanations
    
    fprintf('\n=== LQR Cost Matrix Setup ===\n');
    fprintf('The Q matrix penalizes state deviations (higher = more aggressive control)\n');
    fprintf('The R matrix penalizes control effort (higher = less aggressive control)\n');
    fprintf('State vector: [lateral_position, lateral_velocity, yaw_angle, yaw_rate]\n\n');
    
    % Default values tuned for lane keeping
    default_q_lat_pos = 50;      % Penalize lateral position error
    default_q_lat_vel = 1;       % Penalize lateral velocity
    default_q_yaw_angle = 10;    % Penalize yaw angle error
    default_q_yaw_rate = 5;      % Penalize yaw rate
    default_r = 1;               % Control effort penalty
    
    fprintf('Enter Q matrix diagonal elements (press Enter for defaults):\n');
    
    % Get Q matrix elements
    q1 = input(sprintf('Q(1,1) - Lateral position cost [%.1f]: ', default_q_lat_pos));
    if isempty(q1), q1 = default_q_lat_pos; end
    
    q2 = input(sprintf('Q(2,2) - Lateral velocity cost [%.1f]: ', default_q_lat_vel));
    if isempty(q2), q2 = default_q_lat_vel; end
    
    q3 = input(sprintf('Q(3,3) - Yaw angle cost [%.1f]: ', default_q_yaw_angle));
    if isempty(q3), q3 = default_q_yaw_angle; end
    
    q4 = input(sprintf('Q(4,4) - Yaw rate cost [%.1f]: ', default_q_yaw_rate));
    if isempty(q4), q4 = default_q_yaw_rate; end
    
    % Get R matrix
    r = input(sprintf('R - Steering effort cost [%.1f]: ', default_r));
    if isempty(r), r = default_r; end
    
    % Create matrices
    Q = diag([q1, q2, q3, q4]);
    R = r;
    
    fprintf('\nSelected cost matrices:\n');
    fprintf('Q = diag([%.1f, %.1f, %.1f, %.1f])\n', diag(Q));
    fprintf('R = %.1f\n', R);
    
    % Give tuning advice
    fprintf('\nTuning tips:\n');
    fprintf('- Higher Q(1,1) → faster return to lane center\n');
    fprintf('- Higher Q(2,2) → less lateral oscillation\n');
    fprintf('- Higher Q(3,3) → better heading control\n');
    fprintf('- Higher Q(4,4) → smoother yaw motion\n');
    fprintf('- Higher R → gentler steering, slower response\n');
end

function [A, B] = create_vehicle_model(params)
    % Create linearized state-space matrices for vehicle lateral dynamics
    % Based on bicycle model linearized around straight-line driving
    
    % Extract parameters
    m = params.m;
    Iz = params.Iz;
    lf = params.lf;
    lr = params.lr;
    Cf = params.Cf;
    Cr = params.Cr;
    v = params.v;
    
    % State: [y, vy, psi, r] (lateral position, lateral velocity, yaw angle, yaw rate)
    % Input: delta_f (front wheel steering angle)
    
    % A matrix
    A = [0,  1,  v,  0;
         0,  -(Cf + Cr)/(m*v),  0,  -(lf*Cf - lr*Cr)/(m*v) - v;
         0,  0,  0,  1;
         0,  -(lf*Cf - lr*Cr)/(Iz*v),  0,  -(lf^2*Cf + lr^2*Cr)/(Iz*v)];
    
    % B matrix
    B = [0;
         Cf/m;
         0;
         lf*Cf/Iz];
end

function dxdt = vehicle_dynamics(~, x, K, params)
    % Simplified vehicle lateral dynamics with LQR control
    % Uses linearized bicycle model for cleaner simulation
    
    % Extract parameters
    m = params.m;
    Iz = params.Iz;
    lf = params.lf;
    lr = params.lr;
    Cf = params.Cf;
    Cr = params.Cr;
    v = params.v;
    
    % State variables
    y = x(1);        % Lateral position
    vy = x(2);       % Lateral velocity
    psi = x(3);      % Yaw angle
    r = x(4);        % Yaw rate
    
    % Control input (front wheel steering angle)
    delta_f = -K * x;
    
    % Use linearized dynamics for cleaner simulation
    % (This avoids drift issues from nonlinear slip angle calculations)
    A = [0,  1,  v,  0;
         0,  -(Cf + Cr)/(m*v),  0,  -(lf*Cf - lr*Cr)/(m*v) - v;
         0,  0,  0,  1;
         0,  -(lf*Cf - lr*Cr)/(Iz*v),  0,  -(lf^2*Cf + lr^2*Cr)/(Iz*v)];
    
    B = [0;
         Cf/m;
         0;
         lf*Cf/Iz];
    
    % State derivatives using linear model
    dxdt = A * x + B * delta_f;
end

function corners = create_vehicle_shape(length, width)
    % Create vehicle shape for visualization
    half_length = length/2;
    half_width = width/2;
    
    % Vehicle outline (rectangle)
    corners = [-half_length, -half_width;
               half_length,  -half_width;
               half_length,   half_width;
               -half_length,  half_width;
               -half_length, -half_width];
end

function rotated_corners = rotate_vehicle(corners, x, y, yaw)
    % Rotate and translate vehicle shape
    
    % Rotation matrix
    R = [cos(yaw), -sin(yaw);
         sin(yaw),  cos(yaw)];
    
    % Rotate corners
    rotated = corners * R';
    
    % Translate
    rotated_corners = rotated + [x, y];
end

function settling_time = find_settling_time(t, signal, tolerance)
    % Find settling time within tolerance
    
    % Find last time signal exceeds tolerance
    exceed_idx = find(abs(signal) > tolerance, 1, 'last');
    
    if isempty(exceed_idx)
        settling_time = 0;
    else
        settling_time = t(exceed_idx);
    end
end

function result = iif(condition, true_val, false_val)
    % Inline if function for cleaner code
    if condition
        result = true_val;
    else
        result = false_val;
    end
end