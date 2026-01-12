%% ========================================================================
%% ESP32 GIMBAL HARDWARE PID TUNING
%% BLDC Motors with SimpleFOC - OpenLoop Angle Control
%% ========================================================================
% This script simulates the physical gimbal to find optimal PID gains
% for smooth motor movement without oscillation
%% ========================================================================

clear all; close all; clc;

%% PHYSICAL SYSTEM PARAMETERS
% ========================================================================
% Motor & Mechanical Properties
motor_inertia = 0.001;      % kg·m² (moment of inertia of gimbal arm)
motor_friction = 0.05;      % Damping coefficient
motor_time_constant = 0.08; % Response time of BLDC motor (seconds)

% Gimbal Mechanical Limits
tilt_min = 45;              % degrees (mechanical limit)
tilt_max = 135;             % degrees (mechanical limit)
tilt_home = 90;             % degrees (balanced position)

pan_limit = 180;            % degrees (±180° rotation)

% Control System
update_rate = 100;          % Hz (ESP32 loop frequency)
Ts = 1/update_rate;         % Sampling time (0.01s = 10ms)

%% SIMULATION PARAMETERS
% ========================================================================
simulation_time = 5;        % seconds
time = 0:Ts:simulation_time;
N = length(time);

%% INITIAL PID GUESS (From your code)
% ========================================================================
% Tilt Motor (Motor 2)
Kp_tilt_init = 5.0;
Ki_tilt_init = 0.0;
Kd_tilt_init = 0.15;

% Pan Motor (Motor 1)
Kp_pan_init = 3.5;
Ki_pan_init = 0.0;
Kd_pan_init = 0.15;

%% TEST SCENARIO: STEP RESPONSE
% ========================================================================
% Simulate receiving tracking data from Android app

% Tilt: Step from 90° to 110° (target moves up)
target_tilt = zeros(1, N);
target_tilt(1:floor(N/4)) = 90;                    % Start at home
target_tilt(floor(N/4):floor(N/2)) = 110;          % Step to 110°
target_tilt(floor(N/2):floor(3*N/4)) = 80;         % Step to 80°
target_tilt(floor(3*N/4):end) = 90;                % Return home

% Pan: Sinusoidal movement (tracking moving person)
target_pan = 30 * sin(2*pi*0.3*time);  % ±30° oscillation at 0.3 Hz

%% MOTOR DYNAMICS MODEL
% ========================================================================
% Second-order system: Position -> Velocity -> Acceleration
% Simulates physical inertia and friction

function [pos_next, vel_next] = motor_dynamics(pos_current, vel_current, control_input, dt, inertia, friction, tau)
    % control_input: desired position from PID
    % tau: motor time constant
    
    % Position error drives acceleration
    pos_error = control_input - pos_current;
    
    % Acceleration (simplified model)
    acceleration = (pos_error / tau) - (friction * vel_current / inertia);
    
    % Update velocity
    vel_next = vel_current + acceleration * dt;
    
    % Limit velocity (realistic motor speed limit)
    max_velocity = 180;  % deg/s
    vel_next = max(-max_velocity, min(max_velocity, vel_next));
    
    % Update position
    pos_next = pos_current + vel_next * dt;
end

%% SIMULATION FUNCTION
% ========================================================================
function [pos_history, error_history, control_history] = simulate_gimbal(Kp, Ki, Kd, target, start_pos, Ts, motor_tau, inertia, friction)
    N = length(target);
    
    % State variables
    pos_history = zeros(1, N);
    vel_history = zeros(1, N);
    error_history = zeros(1, N);
    control_history = zeros(1, N);
    
    % PID state
    integral = 0;
    prev_error = 0;
    
    % Initial conditions
    pos_history(1) = start_pos;
    vel_history(1) = 0;
    
    for i = 1:N-1
        % Calculate error
        error = target(i) - pos_history(i);
        error_history(i) = error;
        
        % PID calculation
        P = Kp * error;
        
        integral = integral + error * Ts;
        integral = max(-5, min(5, integral));  % Anti-windup
        I = Ki * integral;
        
        if i > 1
            derivative = (error - prev_error) / Ts;
        else
            derivative = 0;
        end
        D = Kd * derivative;
        
        % PID output (position adjustment)
        pid_output = P + I + D;
        control_history(i) = pid_output;
        
        % Desired position = current + PID adjustment
        desired_pos = pos_history(i) + pid_output * Ts;
        
        % Motor dynamics
        [pos_history(i+1), vel_history(i+1)] = motor_dynamics(...
            pos_history(i), vel_history(i), desired_pos, Ts, inertia, friction, motor_tau);
        
        prev_error = error;
    end
    
    error_history(N) = target(N) - pos_history(N);
end

%% RUN TILT SIMULATION
% ========================================================================
fprintf('========================================\n');
fprintf('SIMULATING TILT MOTOR (Motor 2)\n');
fprintf('========================================\n');
fprintf('Initial PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n\n', Kp_tilt_init, Ki_tilt_init, Kd_tilt_init);

[tilt_pos, tilt_error, tilt_control] = simulate_gimbal(...
    Kp_tilt_init, Ki_tilt_init, Kd_tilt_init, ...
    target_tilt, tilt_home, Ts, motor_time_constant, motor_inertia, motor_friction);

% Performance metrics
tilt_mae = mean(abs(tilt_error));
tilt_rmse = sqrt(mean(tilt_error.^2));
tilt_max_error = max(abs(tilt_error));
tilt_overshoot = max(tilt_pos) - max(target_tilt);

fprintf('=== TILT PERFORMANCE ===\n');
fprintf('MAE: %.2f degrees\n', tilt_mae);
fprintf('RMSE: %.2f degrees\n', tilt_rmse);
fprintf('Max Error: %.2f degrees\n', tilt_max_error);
fprintf('Overshoot: %.2f degrees\n\n', tilt_overshoot);

%% RUN PAN SIMULATION
% ========================================================================
fprintf('========================================\n');
fprintf('SIMULATING PAN MOTOR (Motor 1)\n');
fprintf('========================================\n');
fprintf('Initial PID: Kp=%.2f, Ki=%.2f, Kd=%.2f\n\n', Kp_pan_init, Ki_pan_init, Kd_pan_init);

[pan_pos, pan_error, pan_control] = simulate_gimbal(...
    Kp_pan_init, Ki_pan_init, Kd_pan_init, ...
    target_pan, 0, Ts, motor_time_constant, motor_inertia, motor_friction);

% Performance metrics
pan_mae = mean(abs(pan_error));
pan_rmse = sqrt(mean(pan_error.^2));
pan_max_error = max(abs(pan_error));

fprintf('=== PAN PERFORMANCE ===\n');
fprintf('MAE: %.2f degrees\n', pan_mae);
fprintf('RMSE: %.2f degrees\n', pan_rmse);
fprintf('Max Error: %.2f degrees\n\n', pan_max_error);

%% PLOTTING
% ========================================================================
figure('Position', [100, 100, 1400, 900]);

% TILT PLOTS
subplot(3,3,1);
plot(time, target_tilt, 'b-', 'LineWidth', 2); hold on;
plot(time, tilt_pos, 'r--', 'LineWidth', 1.5);
yline(tilt_min, 'k--', 'Min Limit');
yline(tilt_max, 'k--', 'Max Limit');
grid on;
xlabel('Time (s)');
ylabel('Angle (degrees)');
title('TILT: Target vs Actual Position');
legend('Target', 'Actual', 'Limits', 'Location', 'best');

subplot(3,3,2);
plot(time, tilt_error, 'k-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Error (degrees)');
title('TILT: Tracking Error');

subplot(3,3,3);
plot(time, tilt_control, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Control Signal');
title('TILT: PID Output');

% PAN PLOTS
subplot(3,3,4);
plot(time, target_pan, 'b-', 'LineWidth', 2); hold on;
plot(time, pan_pos, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Angle (degrees)');
title('PAN: Target vs Actual Position');
legend('Target', 'Actual', 'Location', 'best');

subplot(3,3,5);
plot(time, pan_error, 'k-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Error (degrees)');
title('PAN: Tracking Error');

subplot(3,3,6);
plot(time, pan_control, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Control Signal');
title('PAN: PID Output');

% ERROR DISTRIBUTION
subplot(3,3,7);
histogram(tilt_error, 30);
grid on;
xlabel('Error (degrees)');
ylabel('Frequency');
title('TILT: Error Distribution');

subplot(3,3,8);
histogram(pan_error, 30);
grid on;
xlabel('Error (degrees)');
ylabel('Frequency');
title('PAN: Error Distribution');

% SUMMARY
subplot(3,3,9);
axis off;
text(0.1, 0.9, 'PERFORMANCE SUMMARY', 'FontSize', 14, 'FontWeight', 'bold');
text(0.1, 0.75, 'TILT MOTOR:', 'FontSize', 12, 'FontWeight', 'bold');
text(0.1, 0.65, sprintf('  Kp = %.2f', Kp_tilt_init), 'FontSize', 10);
text(0.1, 0.55, sprintf('  MAE = %.2f°', tilt_mae), 'FontSize', 10);
text(0.1, 0.45, sprintf('  Overshoot = %.2f°', tilt_overshoot), 'FontSize', 10);
text(0.1, 0.30, 'PAN MOTOR:', 'FontSize', 12, 'FontWeight', 'bold');
text(0.1, 0.20, sprintf('  Kp = %.2f', Kp_pan_init), 'FontSize', 10);
text(0.1, 0.10, sprintf('  MAE = %.2f°', pan_mae), 'FontSize', 10);

sgtitle('ESP32 Gimbal PID Simulation', 'FontSize', 16, 'FontWeight', 'bold');

%% OPTIMIZATION (OPTIONAL)
% ========================================================================
fprintf('\n========================================\n');
fprintf('OPTIMIZATION OPTIONS:\n');
fprintf('========================================\n');
fprintf('1. Manual tuning: Adjust Kp, Ki, Kd values above and re-run\n');
fprintf('2. Auto-tune: Uncomment optimization code below\n\n');

% Uncomment to enable auto-optimization for TILT
% fprintf('Starting TILT optimization...\n');
% cost_tilt = @(params) optimize_cost(params, target_tilt, tilt_home, Ts, motor_time_constant, inertia, friction);
% x0_tilt = [Kp_tilt_init, Ki_tilt_init, Kd_tilt_init];
% lb_tilt = [1.0, 0.0, 0.05];
% ub_tilt = [15.0, 0.5, 1.0];
% options = optimoptions('fmincon', 'Display', 'iter', 'MaxIterations', 50);
% [optimal_tilt, cost_tilt_val] = fmincon(cost_tilt, x0_tilt, [], [], [], [], lb_tilt, ub_tilt, [], options);
% fprintf('\nOptimal TILT PID: Kp=%.4f, Ki=%.4f, Kd=%.4f\n', optimal_tilt(1), optimal_tilt(2), optimal_tilt(3));

%% RECOMMENDATIONS
% ========================================================================
fprintf('========================================\n');
fprintf('TUNING RECOMMENDATIONS:\n');
fprintf('========================================\n');

if tilt_overshoot > 5
    fprintf('⚠ TILT: High overshoot (%.1f°)\n', tilt_overshoot);
    fprintf('   → Increase Kd (try %.2f)\n', Kd_tilt_init * 1.5);
    fprintf('   → Or decrease Kp (try %.2f)\n\n', Kp_tilt_init * 0.8);
end

if tilt_mae > 2
    fprintf('⚠ TILT: High tracking error (%.1f°)\n', tilt_mae);
    fprintf('   → Increase Kp (try %.2f)\n\n', Kp_tilt_init * 1.2);
end

if pan_mae > 3
    fprintf('⚠ PAN: High tracking error (%.1f°)\n', pan_mae);
    fprintf('   → Increase Kp (try %.2f)\n\n', Kp_pan_init * 1.2);
end

fprintf('✓ Update ESP32 code with optimized values\n');
fprintf('✓ Test on real hardware\n');
fprintf('✓ Fine-tune based on physical behavior\n');

%% HELPER FUNCTION FOR OPTIMIZATION
% ========================================================================
function cost = optimize_cost(params, target, start_pos, Ts, tau, inertia, friction)
    Kp = params(1);
    Ki = params(2);
    Kd = params(3);
    
    [~, error_history, ~] = simulate_gimbal(Kp, Ki, Kd, target, start_pos, Ts, tau, inertia, friction);
    
    rmse = sqrt(mean(error_history.^2));
    max_error = max(abs(error_history));
    
    % Cost: prioritize low RMSE and max error
    cost = rmse + 0.5 * max_error;
end
