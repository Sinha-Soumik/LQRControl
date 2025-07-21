
syms p_long p_lat heading v_long throttle delta_cmd steering_angle; 

Tc = 2;         % Time Constant
m = 1558;       % Vehicle Mass
lf = 1.462884;  % Distance from cg to front axle
lr = 1.405516;  % Distance from cg to rear axle
l = lf + lr;



Cf = 1.432394487827058e+05; %
Cr = 2.214094963126969e+05;
g = 9.80655;

kus = (m*g*lr/(l*Cf)-m*g*lf/(l*Cr));    
yr = tan(steering_angle)*v_long/(l+kus*v_long^2/g);
v_lat = yr*(lr-m*v_long^2*lr/(Cr*l));

f1 = v_long*cos(heading)-((tan(steering_angle)*v_long/(l+((m*g*lr/(l*Cf)-m*g*lf/(l*Cr)))*v_long^2/g))*(lr-m*v_long^2*lr/(Cr*l)))*sin(heading);
f2 = v_long*sin(heading)+((tan(steering_angle)*v_long/(l+((m*g*lr/(l*Cf)-m*g*lf/(l*Cr)))*v_long^2/g))*(lr-m*v_long^2*lr/(Cr*l)))*cos(heading);
f3 = tan(steering_angle)*v_long/(l+((m*g*lr/(l*Cf)-m*g*lf/(l*Cr)))*v_long^2/g);
f4 = -Tc/m*v_long+Tc*throttle;
f5 = 0.8*(delta_cmd-steering_angle);


A = jacobian([f1, f2, f3, f4, f5], [p_long,p_lat, heading, v_long, steering_angle]);
B = jacobian([f1, f2, f3, f4, f5], [throttle, delta_cmd]);


x0 = [0; 0; -pi/6; 10; 0];
tspan = 0:0.01:10;
[t, X] = ode45(@(t, x) closedloopdyn(t, x, A, B), tspan, x0);



close all;
figure('Position',[300 500 1000 650]);
clf;

yyaxis left; % Left side of the y-axis
plot(t, X(:, 3), 'r-', 'LineWidth', 2); 
hold on;
plot(t, X(:, 5), 'b-', 'LineWidth', 5); 

xlabel('Time (s)');
ylabel('Lateral Position');

yyaxis right; % Right side of the y-axis
plot(t, X(:, 4), 'g-', 'LineWidth', 2);
plot(t, X(:, 2), 'k-', 'LineWidth', 2); 
xlabel('Time (s)');
ylabel('Speed / Lateral Position'); % Updated ylabel

legend({'Heading' 'Steering Angle' 'Velocity' 'Lateral Position'}); % Updated legend labels
title('TVLQR Car Control');
grid on;
set(gca, "FontName", "Arial", "FontSize", 24)

xlim([0 5.5])
hold off;

print("TVLQR_heading.eps", "-depsc")




% Visualization parameters
car_length = 1;
car_width = 2;

% Number of cars to visualize
num_cars = 1;

% Time step interval for updating visualization
update_interval = 10; % Adjust as needed

% Initialize figure
figure('Position',[0 500 1500 300]);

% Simulate and visualize the car dynamics
for car_idx = 1:num_cars
    for i = 1:update_interval:length(t)
        % Extract states from X
        x_pos = X(i, 1);
        y_pos = X(i, 2);
        heading = X(i, 3) + (car_idx - 1) * 0.1; % Offset headings for multiple cars
        delta = X(i, 5);
        alpha = atan((car_width/2)/car_length/2);

        % Calculate the car's corner points based on the steering angle
        front_left = [x_pos + cos(heading+alpha)*sqrt(car_width^2+car_length^2), y_pos + sin(heading+alpha)*sqrt(car_width^2+car_length^2)];
        front_right = [x_pos + cos(heading-alpha)*sqrt(car_width^2+car_length^2), y_pos + sin(heading-alpha)*sqrt(car_width^2+car_length^2)];
        rear_left = [x_pos - cos(heading-alpha)*sqrt(car_width^2+car_length^2), y_pos - sin(heading-alpha)*sqrt(car_width^2+car_length^2)];
        rear_right = [x_pos - sin((pi/2-heading)-alpha)*sqrt(car_width^2+car_length^2), y_pos - cos((pi/2-heading)-alpha)*sqrt(car_width^2+car_length^2)];

        % Plot car body with increasing opacity
        opacity = min(0.2 + 0.8 * (i / length(t)), 1);



        % Plot car body with increasing opacity
        car_body = fill([front_left(1), front_right(1), rear_right(1), rear_left(1)], ...
                        [front_left(2), front_right(2), rear_right(2), rear_left(2)], 'b', 'FaceAlpha', opacity);

        hold on;

        plot([x_pos - 20, x_pos + 20], [2.5, 2.5], 'k--', 'LineWidth', 2); % Upper lane line
        plot([x_pos - 20, x_pos + 20], [-2.5, -2.5], 'k--', 'LineWidth', 2); % Lower lane line

        % Update plot properties
        xlim([x_pos - 20, x_pos + 20]);
        ylim([y_pos - 20, y_pos + 20]);
        xlabel('X Position');
        ylabel('Y Position');
        title('Car Simulation');
        grid on;
        drawnow; % Update the plot

        pause(0.1);
    end
end
xlim([0 100]);
ylim([-20 20]);

set(gca, "FontName", "Arial", "FontSize", 24)
print("TVLQR_good_simulation.eps", "-depsc")

hold off;



function dxdt = closedloopdyn(t, x, A, B)   
    syms p_long p_lat heading v_long throttle delta_cmd steering_angle; 


    
   

    u_acc = 0.1* (20 - x(4));
    u_steer = 0.1 *(0 - x(3));
    
    u_bar = [u_acc; u_steer];
    x_des = [0; 0; 0; 20; 0];
    initial_state = [0; 0; pi/4; 50; 0];


    T=1.75;

    x_bar = initial_state + t*(x_des-initial_state)/T;

    if t>T
        x_bar = x_des;
    end
 

    % T = 5;
    % x_bar(1) = x0(1)+((0-x0(1))/T)*t;
    % x_bar(2) = x0(2)+((0-x0(2))/T)*t;
    % x_bar(3) = x0(3)+((0-x0(3))/T)*t;
    % x_bar(4) = x0(4)+((20-x0(4))/T)*t;
    % x_bar(5) = x0(5)+((0-x0(5))/T)*t;


    A_tilde = subs(A, [p_long, p_lat, heading, v_long, steering_angle, throttle, delta_cmd], ...
            [x_bar(1), x_bar(2), x_bar(3), x_bar(4), x_bar(5), u_bar(1), u_bar(2)]);
    B_tilde = subs(B, [p_long, p_lat, heading, v_long, steering_angle, throttle, delta_cmd], ...
            [x_bar(1), x_bar(2), x_bar(3), x_bar(4), x_bar(5), u_bar(1), u_bar(2)]);

    A_tilde = double(A_tilde);
    B_tilde = double(B_tilde);

    Q = diag([0.000001, 10, 10, 10, 10]); %Cost on States
    R = diag([1, 1]);            % Cost on Actions
    
    %LQR
    [K, ~, ~] = lqr(A_tilde, B_tilde, Q, R);
    delta_x = x-x_bar;
   
    u = u_bar -K*delta_x;


    max_del_dot = 8;
    max_throttle = 3;

    throttle = u(1);
    delta_cmd= u(2);


    if throttle > max_throttle
        throttle  = max_throttle;      
    end
    if throttle < -max_throttle
        throttle = -max_throttle;      
    end

    u(1) = throttle;

    dxdt = dynamics(x, u);

    del_dot = dxdt(5);

    if del_dot > max_del_dot
        del_dot = max_del_dot;      
    end
    if del_dot < -max_del_dot
        del_dot = -max_del_dot;      
    end

    dxdt(5) = del_dot;

end






function dxdt = dynamics(x, u)
    Tc = 2;         % Time Constant
    m = 1558;       % Vehicle Mass
    lf = 1.462884;  % Distance from cg to front axle
    lr = 1.405516;  % Distance from cg to rear axle
    l = lf + lr;    
    
    Cf = 1.432394487827058e+05; %
    Cr = 2.214094963126969e+05;
    g = 9.80655;
        
    heading = x(3);
    v_long = x(4);
    delta = x(5);

    throttle = u(1);
    delta_cmd = u(2);
  
    f1 = v_long*cos(heading)-((tan(delta)*v_long/(l+((m*g*lr/(l*Cf)-m*g*lf/(l*Cr)))*v_long^2/g))*(lr-m*v_long^2*lr/(Cr*l)))*sin(heading);
    f2 = v_long*sin(heading)+((tan(delta)*v_long/(l+((m*g*lr/(l*Cf)-m*g*lf/(l*Cr)))*v_long^2/g))*(lr-m*v_long^2*lr/(Cr*l)))*cos(heading);
    f3 = tan(delta)*v_long/(l+((m*g*lr/(l*Cf)-m*g*lf/(l*Cr)))*v_long^2/g);
    f4 = -Tc/m*v_long+Tc*throttle;
    f5 = 0.8*(delta_cmd-delta);

    dxdt = [f1; f2; f3; f4; f5];

end