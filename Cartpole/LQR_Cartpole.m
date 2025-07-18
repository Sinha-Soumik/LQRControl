function cartpole_lqr_demo()
    % CARTPOLE_LQR_DEMO - Enhanced LQR control demonstration for cart-pole system
    %
    % This function demonstrates Linear Quadratic Regulator (LQR) control
    % applied to a cart-pole system with user-configurable costs and initial conditions.
    %
    % Usage: cartpole_lqr_demo()
    
    clc; close all; clear;
    
    % =====================================================================
    % SYSTEM PARAMETERS - Modify these as needed
    % =====================================================================
    
    % Physical parameters
    params = struct();
    params.m = 0.5;      % Mass of the pole (kg)
    params.M = 2.0;      % Mass of the cart (kg)
    params.l = 0.5;      % Length of the pole (m)
    params.g = 9.8;      % Gravitational acceleration (m/s^2)
    
    % Simulation parameters
    t_final = 15;              % Simulation time (s)
    dt = 0.01;                 % Time step (s)
    
    % Animation parameters
    cart_width = 0.4;          % Made narrower
    cart_height = 0.2;         % Made shorter (less "fat")
    pole_length = 0.75;
    pole_thickness = 6;
    
    % =====================================================================
    % DESIGN LQR CONTROLLER
    % =====================================================================
    
    fprintf('=== Cart-Pole LQR Control Demo ===\n\n');
    
    % Create linearized system matrices
    [A, B] = create_linearized_system(params);
    
    fprintf('System Parameters:\n');
    fprintf('  Cart mass: %.1f kg, Pole mass: %.1f kg\n', params.M, params.m);
    fprintf('  Pole length: %.1f m, Gravity: %.1f m/s²\n', params.l, params.g);
    
    % Get LQR cost matrices from user
    [Q, R] = get_lqr_costs();
    
    % Design LQR controller
    [K, S, E] = lqr(A, B, Q, R);
    
    fprintf('\nLQR Controller Design:\n');
    fprintf('  Control gains K = [%.3f, %.3f, %.3f, %.3f]\n', K);
    fprintf('  System is stable: %s\n', iif(all(real(E) < 0), 'YES', 'NO'));
    
    % =====================================================================
    % GET USER INPUT FOR INITIAL CONDITIONS
    % =====================================================================
    
    fprintf('\n=== Initial Conditions Setup ===\n');
    fprintf('Enter initial conditions (press Enter for default values):\n');
    
    % Get initial cart position
    default_x = 0;
    x0_input = input(sprintf('Initial cart position [%.1f m]: ', default_x));
    if isempty(x0_input)
        x0_input = default_x;
    end
    
    % Get initial cart velocity
    default_xdot = 0;
    xdot0_input = input(sprintf('Initial cart velocity [%.1f m/s]: ', default_xdot));
    if isempty(xdot0_input)
        xdot0_input = default_xdot;
    end
    
    % Get initial pole angle (in degrees for user convenience)
    default_theta_deg = -30;
    theta0_input = input(sprintf('Initial pole angle [%.1f deg]: ', default_theta_deg));
    if isempty(theta0_input)
        theta0_input = default_theta_deg;
    end
    theta0_rad = theta0_input * pi / 180;  % Convert to radians
    
    % Get initial pole angular velocity
    default_thetadot = 0;
    thetadot0_input = input(sprintf('Initial pole angular velocity [%.1f deg/s]: ', default_thetadot));
    if isempty(thetadot0_input)
        thetadot0_input = default_thetadot;
    end
    thetadot0_rad = thetadot0_input * pi / 180;  % Convert to radians
    
    % Package initial conditions
    x0 = [x0_input; xdot0_input; theta0_rad; thetadot0_rad];
    
    fprintf('\nStarting simulation with initial conditions:\n');
    fprintf('  Cart position: %.2f m\n', x0(1));
    fprintf('  Cart velocity: %.2f m/s\n', x0(2));
    fprintf('  Pole angle: %.1f deg\n', x0(3) * 180/pi);
    fprintf('  Pole angular velocity: %.1f deg/s\n', x0(4) * 180/pi);
    
    % =====================================================================
    % RUN SIMULATION
    % =====================================================================
    
    fprintf('\nRunning simulation...\n');
    
    % Time vector
    tspan = 0:dt:t_final;
    
    % Simulate system
    options = odeset('RelTol', 1e-6, 'AbsTol', 1e-8);
    [t, X] = ode45(@(t, x) cartpole_dynamics(t, x, K, params), tspan, x0, options);
    
    % Calculate control input
    U = -K * X';
    
    % =====================================================================
    % CREATE PLOTS
    % =====================================================================
    
    fprintf('Creating plots...\n');
    
    % Main results figure
    fig1 = figure('Position', [100, 100, 1000, 600]);
    set(fig1, 'Name', 'Cart-Pole LQR Control Results', 'NumberTitle', 'off');
    
    % Plot 1: Cart position and velocity
    subplot(2, 2, 1);
    yyaxis left;
    plot(t, X(:, 1), 'b-', 'LineWidth', 2);
    ylabel('Cart Position (m)');
    ylim_left = ylim;
    
    yyaxis right;
    plot(t, X(:, 2), 'r--', 'LineWidth', 2);
    ylabel('Cart Velocity (m/s)');
    
    xlabel('Time (s)');
    title('Cart Motion');
    grid on;
    legend({'Cart Position', 'Cart Velocity'}, 'Location', 'best');
    
    % Plot 2: Pole angle and angular velocity
    subplot(2, 2, 2);
    yyaxis left;
    plot(t, X(:, 3) * 180/pi, 'b-', 'LineWidth', 2);
    ylabel('Pole Angle (deg)');
    
    yyaxis right;
    plot(t, X(:, 4) * 180/pi, 'r--', 'LineWidth', 2);
    ylabel('Pole Angular Velocity (deg/s)');
    
    xlabel('Time (s)');
    title('Pole Motion');
    grid on;
    legend({'Pole Angle', 'Pole Angular Velocity'}, 'Location', 'best');
    
    % Plot 3: Control input
    subplot(2, 2, 3);
    plot(t, U, 'g-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Control Force (N)');
    title('Control Input');
    grid on;
    
    % Plot 4: System overview
    subplot(2, 2, 4);
    plot(t, X(:, 1), 'b-', 'LineWidth', 2);
    hold on;
    plot(t, X(:, 3) * 180/pi / 180 * max(abs(X(:, 1))), 'r-', 'LineWidth', 2);
    plot(t, U / max(abs(U)) * max(abs(X(:, 1))), 'g-', 'LineWidth', 2);
    xlabel('Time (s)');
    ylabel('Normalized Values');
    title('System Overview (Normalized)');
    legend({'Cart Position', 'Pole Angle', 'Control Force'}, 'Location', 'best');
    grid on;
    
    sgtitle('Cart-Pole LQR Control Analysis', 'FontSize', 14, 'FontWeight', 'bold');
    
    % =====================================================================
    % CONTINUOUS ANIMATION
    % =====================================================================
    
    fprintf('\nStarting animation...\n');
    fprintf('Press any key to stop the animation.\n');
    
    % Create animation figure
    fig2 = figure('Position', [200, 200, 1000, 700]);
    set(fig2, 'Name', 'Cart-Pole Animation - Press any key to stop', 'NumberTitle', 'off');
    
    % Set up animation axes
    ax = axes('Position', [0.1, 0.3, 0.8, 0.6]);
    axis equal;
    
    % Calculate appropriate axis limits based on system parameters and simulation
    max_cart_pos = max(abs(X(:, 1)));
    x_margin = max(2, max_cart_pos + 1);  % At least 2m margin, or 1m beyond max position
    
    xlim([-x_margin, x_margin]);
    ylim([-0.5, pole_length + 0.5]);
    xlabel('Position (m)');
    ylabel('Height (m)');
    title('Cart-Pole LQR Control Animation');
    grid on;
    hold on;
    
    % Draw ground
    ground_y = -0.1;
    fill([-x_margin, x_margin, x_margin, -x_margin], [ground_y, ground_y, ground_y-0.05, ground_y-0.05], ...
         [0.5, 0.5, 0.5], 'EdgeColor', 'none');
    
    % Draw track
    track_y = ground_y + 0.02;
    plot([-x_margin, x_margin], [track_y, track_y], 'k-', 'LineWidth', 3);
    
    % Initialize graphics objects
    cart_handle = rectangle('Position', [0, 0, cart_width, cart_height], ...
                           'FaceColor', 'blue', 'EdgeColor', 'black', 'LineWidth', 2);
    pole_handle = plot([0, 0], [0, 0], 'r-', 'LineWidth', pole_thickness);
    
    % Add pole tip marker
    pole_tip_handle = plot(0, 0, 'ro', 'MarkerSize', 8, 'MarkerFaceColor', 'red');
    
    % Add information panel
    info_panel = uipanel('Title', 'Real-time Information', 'Position', [0.1, 0.05, 0.8, 0.2]);
    
    % Create text displays in a grid
    time_text = uicontrol('Parent', info_panel, 'Style', 'text', 'String', 'Time: 0.00 s', ...
                         'Position', [20, 80, 150, 25], 'FontSize', 11, 'FontWeight', 'bold');
    cart_pos_text = uicontrol('Parent', info_panel, 'Style', 'text', 'String', 'Cart Position: 0.00 m', ...
                             'Position', [20, 55, 150, 20], 'FontSize', 10);
    cart_vel_text = uicontrol('Parent', info_panel, 'Style', 'text', 'String', 'Cart Velocity: 0.00 m/s', ...
                             'Position', [20, 35, 150, 20], 'FontSize', 10);
    
    pole_angle_text = uicontrol('Parent', info_panel, 'Style', 'text', 'String', 'Pole Angle: 0.0°', ...
                               'Position', [200, 55, 150, 20], 'FontSize', 10);
    pole_vel_text = uicontrol('Parent', info_panel, 'Style', 'text', 'String', 'Pole Velocity: 0.0°/s', ...
                             'Position', [200, 35, 150, 20], 'FontSize', 10);
    
    control_text = uicontrol('Parent', info_panel, 'Style', 'text', 'String', 'Control Force: 0.0 N', ...
                            'Position', [380, 55, 150, 20], 'FontSize', 10);
    
    % Cost matrix display
    cost_text = uicontrol('Parent', info_panel, 'Style', 'text', ...
                         'String', sprintf('Q = diag([%.1f, %.1f, %.1f, %.1f]), R = %.1f', ...
                                         diag(Q), R), ...
                         'Position', [20, 10, 400, 20], 'FontSize', 9);
    
    % Animation loop - continuous until key press
    loop_count = 0;
    skip_frames = 3;  % Skip frames for smoother animation
    
    % Set up key press detection
    set(fig2, 'KeyPressFcn', @(src, event) set(src, 'UserData', 'stop'));
    set(fig2, 'UserData', 'running');
    
    while strcmp(get(fig2, 'UserData'), 'running') && ishandle(fig2)
        for i = 1:skip_frames:length(t)
            if ~ishandle(fig2) || ~strcmp(get(fig2, 'UserData'), 'running')
                break;
            end
            
            % Get current state
            cart_pos = X(i, 1);
            cart_vel = X(i, 2);
            pole_angle = X(i, 3);
            pole_vel = X(i, 4);
            control_force = U(i);
            
            % Calculate positions
            cart_x = cart_pos - cart_width/2;
            cart_y = ground_y + 0.02;
            
            pole_base_x = cart_pos;
            pole_base_y = cart_y + cart_height;
            pole_tip_x = cart_pos + pole_length * sin(pole_angle);
            pole_tip_y = pole_base_y + pole_length * cos(pole_angle);
            
            % Update graphics
            set(cart_handle, 'Position', [cart_x, cart_y, cart_width, cart_height]);
            set(pole_handle, 'XData', [pole_base_x, pole_tip_x], 'YData', [pole_base_y, pole_tip_y]);
            set(pole_tip_handle, 'XData', pole_tip_x, 'YData', pole_tip_y);
            
            % Update information displays
            set(time_text, 'String', sprintf('Time: %.2f s', t(i)));
            set(cart_pos_text, 'String', sprintf('Cart Position: %.3f m', cart_pos));
            set(cart_vel_text, 'String', sprintf('Cart Velocity: %.3f m/s', cart_vel));
            set(pole_angle_text, 'String', sprintf('Pole Angle: %.1f°', pole_angle * 180/pi));
            set(pole_vel_text, 'String', sprintf('Pole Velocity: %.1f°/s', pole_vel * 180/pi));
            set(control_text, 'String', sprintf('Control Force: %.2f N', control_force));
            
            % Dynamic axis adjustment - smooth following
            if abs(cart_pos) > x_margin * 0.7  % Start adjusting when cart reaches 70% of current limit
                new_center = cart_pos;
                new_xlim = [new_center - x_margin, new_center + x_margin];
                xlim(new_xlim);
                
                % Update ground for new limits
                delete(findobj(ax, 'Type', 'patch'));  % Remove old ground
                fill(new_xlim([1 2 2 1]), [ground_y, ground_y, ground_y-0.05, ground_y-0.05], ...
                     [0.5, 0.5, 0.5], 'EdgeColor', 'none');
                plot(new_xlim, [track_y, track_y], 'k-', 'LineWidth', 3);
            end
            
            drawnow limitrate;
            pause(0.015);  % Slightly faster animation
        end
        
        % Increment loop counter and continue
        loop_count = loop_count + 1;
        if loop_count > 1
            set(time_text, 'String', sprintf('Time: %.2f s (Loop %d)', t(end), loop_count));
        end
    end
    
    if ishandle(fig2)
        fprintf('\nAnimation stopped by user.\n');
    end
    
    fprintf('\nDemo completed successfully!\n');
    fprintf('Final state: Cart = %.3f m, Pole = %.1f deg\n', X(end, 1), X(end, 3) * 180/pi);
end

function [Q, R] = get_lqr_costs()
    % Get LQR cost matrices from user with explanations
    
    fprintf('\n=== LQR Cost Matrix Setup ===\n');
    fprintf('The Q matrix penalizes state deviations (higher = more aggressive control)\n');
    fprintf('The R matrix penalizes control effort (higher = less aggressive control)\n');
    fprintf('State vector: [cart_position, cart_velocity, pole_angle, pole_angular_velocity]\n\n');
    
    % Default values
    default_q_cart_pos = 10;
    default_q_cart_vel = 1;
    default_q_pole_angle = 10;
    default_q_pole_vel = 1;
    default_r = 40;
    
    fprintf('Enter Q matrix diagonal elements (press Enter for defaults):\n');
    
    % Get Q matrix elements
    q1 = input(sprintf('Q(1,1) - Cart position cost [%.1f]: ', default_q_cart_pos));
    if isempty(q1), q1 = default_q_cart_pos; end
    
    q2 = input(sprintf('Q(2,2) - Cart velocity cost [%.1f]: ', default_q_cart_vel));
    if isempty(q2), q2 = default_q_cart_vel; end
    
    q3 = input(sprintf('Q(3,3) - Pole angle cost [%.1f]: ', default_q_pole_angle));
    if isempty(q3), q3 = default_q_pole_angle; end
    
    q4 = input(sprintf('Q(4,4) - Pole angular velocity cost [%.1f]: ', default_q_pole_vel));
    if isempty(q4), q4 = default_q_pole_vel; end
    
    % Get R matrix
    r = input(sprintf('R - Control effort cost [%.1f]: ', default_r));
    if isempty(r), r = default_r; end
    
    % Create matrices
    Q = diag([q1, q2, q3, q4]);
    R = r;
    
    fprintf('\nSelected cost matrices:\n');
    fprintf('Q = diag([%.1f, %.1f, %.1f, %.1f])\n', diag(Q));
    fprintf('R = %.1f\n', R);
    
    % Give tuning advice
    fprintf('\nTuning tips:\n');
    fprintf('- Higher Q values → faster response, more aggressive control\n');
    fprintf('- Higher R values → smoother control, less aggressive response\n');
    fprintf('- Pole angle (Q3) is usually most important for stability\n');
    fprintf('- Pole velocity (Q4) helps with damping and smoother motion\n');
end

function [A, B] = create_linearized_system(params)
    % Create linearized state-space matrices for cart-pole system
    
    m = params.m;
    M = params.M;
    l = params.l;
    g = params.g;
    
    % Linearized around upright equilibrium (theta = 0)
    % State: [x, x_dot, theta, theta_dot]
    % Input: u (force on cart)
    
    A = [0,  1,           0,        0;
         0,  0,   -m*g/(M+m),       0;
         0,  0,           0,        1;
         0,  0,   g*(M+m)/(l*(M+m)), 0];
    
    B = [0;
         1/(M+m);
         0;
         -1/(l*(M+m))];
end

function dxdt = cartpole_dynamics(~, x, K, params)
    % Nonlinear cart-pole dynamics with LQR control
    
    % Extract parameters
    m = params.m;
    M = params.M;
    l = params.l;
    g = params.g;
    
    % State variables
    x_pos = x(1);
    x_vel = x(2);
    theta = x(3);
    theta_vel = x(4);
    
    % Normalize angle to [-pi, pi]
    theta = atan2(sin(theta), cos(theta));
    
    % Control input
    u = -K * [x_pos; x_vel; theta; theta_vel];
    
    % Nonlinear dynamics
    sin_theta = sin(theta);
    cos_theta = cos(theta);
    
    % Denominator term
    denom = M + m - m * cos_theta^2;
    
    % Accelerations
    x_accel = (u + m * l * theta_vel^2 * sin_theta - m * g * cos_theta * sin_theta) / denom;
    theta_accel = (-u * cos_theta - m * l * theta_vel^2 * cos_theta * sin_theta + ...
                   (M + m) * g * sin_theta) / (l * denom);
    
    % State derivatives
    dxdt = [x_vel; x_accel; theta_vel; theta_accel];
end

function result = iif(condition, true_val, false_val)
    % Inline if function for cleaner code
    if condition
        result = true_val;
    else
        result = false_val;
    end
end