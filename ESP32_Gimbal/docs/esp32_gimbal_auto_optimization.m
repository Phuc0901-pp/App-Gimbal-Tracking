%% ========================================================================
%% ESP32 GIMBAL AUTO-OPTIMIZATION
%% BLDC Motors - Find Optimal PID for Tilt & Pan
%% ========================================================================

clear all; close all; clc;

%% SYSTEM PARAMETERS
% ========================================================================
motor_inertia = 0.001;
motor_friction = 0.05;
motor_time_constant = 0.08;

tilt_min = 45;
tilt_max = 135;
tilt_home = 90;
pan_limit = 180;

update_rate = 100;
Ts = 1/update_rate;

simulation_time = 5;
time = 0:Ts:simulation_time;
N = length(time);

%% INITIAL GUESS
% ========================================================================
Kp_tilt_init = 5.0;
Ki_tilt_init = 0.0;
Kd_tilt_init = 0.15;

Kp_pan_init = 3.5;
Ki_pan_init = 0.0;
Kd_pan_init = 0.15;

%% TARGET TRAJECTORIES
% ========================================================================
target_tilt = zeros(1, N);
target_tilt(1:floor(N/4)) = 90;
target_tilt(floor(N/4):floor(N/2)) = 110;
target_tilt(floor(N/2):floor(3*N/4)) = 80;
target_tilt(floor(3*N/4):end) = 90;

target_pan = 30 * sin(2*pi*0.3*time);

%% MOTOR DYNAMICS
% ========================================================================
function [pos_next, vel_next] = motor_dynamics(pos_current, vel_current, control_input, dt, inertia, friction, tau)
    pos_error = control_input - pos_current;
    acceleration = (pos_error / tau) - (friction * vel_current / inertia);
    vel_next = vel_current + acceleration * dt;
    max_velocity = 180;
    vel_next = max(-max_velocity, min(max_velocity, vel_next));
    pos_next = pos_current + vel_next * dt;
end

%% SIMULATION FUNCTION
% ========================================================================
function [pos_history, error_history, control_history] = simulate_gimbal(Kp, Ki, Kd, target, start_pos, Ts, motor_tau, inertia, friction)
    N = length(target);
    pos_history = zeros(1, N);
    vel_history = zeros(1, N);
    error_history = zeros(1, N);
    control_history = zeros(1, N);
    
    integral = 0;
    prev_error = 0;
    pos_history(1) = start_pos;
    vel_history(1) = 0;
    
    for i = 1:N-1
        error = target(i) - pos_history(i);
        error_history(i) = error;
        
        P = Kp * error;
        integral = integral + error * Ts;
        integral = max(-5, min(5, integral));
        I = Ki * integral;
        
        if i > 1
            derivative = (error - prev_error) / Ts;
        else
            derivative = 0;
        end
        D = Kd * derivative;
        
        pid_output = P + I + D;
        control_history(i) = pid_output;
        desired_pos = pos_history(i) + pid_output * Ts;
        
        [pos_history(i+1), vel_history(i+1)] = motor_dynamics(...
            pos_history(i), vel_history(i), desired_pos, Ts, inertia, friction, motor_tau);
        
        prev_error = error;
    end
    
    error_history(N) = target(N) - pos_history(N);
end

%% COST FUNCTION
% ========================================================================
function cost = optimize_cost(params, target, start_pos, Ts, tau, inertia, friction)
    Kp = params(1);
    Ki = params(2);
    Kd = params(3);
    
    [pos_history, error_history, ~] = simulate_gimbal(Kp, Ki, Kd, target, start_pos, Ts, tau, inertia, friction);
    
    rmse = sqrt(mean(error_history.^2));
    max_error = max(abs(error_history));
    
    % Penalize overshoot
    overshoot = max(pos_history) - max(target);
    overshoot_penalty = max(0, overshoot - 5) * 2;
    
    cost = rmse + 0.5 * max_error + overshoot_penalty;
end

%% OPTIMIZE TILT MOTOR
% ========================================================================
fprintf('========================================\n');
fprintf('OPTIMIZING TILT MOTOR (Motor 2)\n');
fprintf('========================================\n');
fprintf('Initial: Kp=%.2f, Ki=%.2f, Kd=%.2f\n\n', Kp_tilt_init, Ki_tilt_init, Kd_tilt_init);

cost_tilt = @(params) optimize_cost(params, target_tilt, tilt_home, Ts, motor_time_constant, motor_inertia, motor_friction);

x0_tilt = [Kp_tilt_init, Ki_tilt_init, Kd_tilt_init];
lb_tilt = [3.0, 0.0, 0.05];
ub_tilt = [20.0, 0.5, 1.0];

options = optimoptions('fmincon', ...
    'Display', 'iter', ...
    'MaxIterations', 100, ...
    'MaxFunctionEvaluations', 300);

fprintf('Running optimization (may take 5-10 minutes)...\n\n');
tic;
[optimal_tilt, cost_tilt_val] = fmincon(cost_tilt, x0_tilt, [], [], [], [], lb_tilt, ub_tilt, [], options);
tilt_time = toc;

fprintf('\n=== TILT OPTIMIZATION COMPLETE ===\n');
fprintf('Time: %.1f seconds\n', tilt_time);
fprintf('Optimal Kp = %.4f\n', optimal_tilt(1));
fprintf('Optimal Ki = %.4f\n', optimal_tilt(2));
fprintf('Optimal Kd = %.4f\n', optimal_tilt(3));
fprintf('Cost = %.4f\n\n', cost_tilt_val);

%% OPTIMIZE PAN MOTOR
% ========================================================================
fprintf('========================================\n');
fprintf('OPTIMIZING PAN MOTOR (Motor 1)\n');
fprintf('========================================\n');
fprintf('Initial: Kp=%.2f, Ki=%.2f, Kd=%.2f\n\n', Kp_pan_init, Ki_pan_init, Kd_pan_init);

cost_pan = @(params) optimize_cost(params, target_pan, 0, Ts, motor_time_constant, motor_inertia, motor_friction);

x0_pan = [Kp_pan_init, Ki_pan_init, Kd_pan_init];
lb_pan = [3.0, 0.0, 0.05];
ub_pan = [20.0, 0.5, 1.0];

tic;
[optimal_pan, cost_pan_val] = fmincon(cost_pan, x0_pan, [], [], [], [], lb_pan, ub_pan, [], options);
pan_time = toc;

fprintf('\n=== PAN OPTIMIZATION COMPLETE ===\n');
fprintf('Time: %.1f seconds\n', pan_time);
fprintf('Optimal Kp = %.4f\n', optimal_pan(1));
fprintf('Optimal Ki = %.4f\n', optimal_pan(2));
fprintf('Optimal Kd = %.4f\n', optimal_pan(3));
fprintf('Cost = %.4f\n\n', cost_pan_val);

%% RUN FINAL SIMULATION WITH OPTIMAL VALUES
% ========================================================================
fprintf('Running final simulation with optimal PID...\n\n');

[tilt_pos, tilt_error, tilt_control] = simulate_gimbal(...
    optimal_tilt(1), optimal_tilt(2), optimal_tilt(3), ...
    target_tilt, tilt_home, Ts, motor_time_constant, motor_inertia, motor_friction);

[pan_pos, pan_error, pan_control] = simulate_gimbal(...
    optimal_pan(1), optimal_pan(2), optimal_pan(3), ...
    target_pan, 0, Ts, motor_time_constant, motor_inertia, motor_friction);

%% PERFORMANCE METRICS
% ========================================================================
tilt_mae = mean(abs(tilt_error));
tilt_rmse = sqrt(mean(tilt_error.^2));
tilt_max_error = max(abs(tilt_error));
tilt_overshoot = max(tilt_pos) - max(target_tilt);

pan_mae = mean(abs(pan_error));
pan_rmse = sqrt(mean(pan_error.^2));
pan_max_error = max(abs(pan_error));

fprintf('========================================\n');
fprintf('FINAL PERFORMANCE\n');
fprintf('========================================\n\n');

fprintf('TILT MOTOR:\n');
fprintf('  MAE: %.2f° (Target: < 2°) %s\n', tilt_mae, ternary(tilt_mae < 2, '✓', '⚠'));
fprintf('  RMSE: %.2f°\n', tilt_rmse);
fprintf('  Max Error: %.2f°\n', tilt_max_error);
fprintf('  Overshoot: %.2f° (Target: < 5°) %s\n\n', tilt_overshoot, ternary(abs(tilt_overshoot) < 5, '✓', '⚠'));

fprintf('PAN MOTOR:\n');
fprintf('  MAE: %.2f° (Target: < 3°) %s\n', pan_mae, ternary(pan_mae < 3, '✓', '⚠'));
fprintf('  RMSE: %.2f°\n', pan_rmse);
fprintf('  Max Error: %.2f°\n\n', pan_max_error);

%% PLOTTING
% ========================================================================
figure('Position', [100, 100, 1400, 900]);

subplot(3,3,1);
plot(time, target_tilt, 'b-', 'LineWidth', 2); hold on;
plot(time, tilt_pos, 'r--', 'LineWidth', 1.5);
yline(tilt_min, 'k--', 'Min');
yline(tilt_max, 'k--', 'Max');
grid on;
xlabel('Time (s)');
ylabel('Angle (°)');
title('TILT: Optimized Response');
legend('Target', 'Actual', 'Location', 'best');

subplot(3,3,2);
plot(time, tilt_error, 'k-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Error (°)');
title('TILT: Error');

subplot(3,3,3);
plot(time, tilt_control, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Control');
title('TILT: PID Output');

subplot(3,3,4);
plot(time, target_pan, 'b-', 'LineWidth', 2); hold on;
plot(time, pan_pos, 'r--', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Angle (°)');
title('PAN: Optimized Response');
legend('Target', 'Actual', 'Location', 'best');

subplot(3,3,5);
plot(time, pan_error, 'k-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Error (°)');
title('PAN: Error');

subplot(3,3,6);
plot(time, pan_control, 'g-', 'LineWidth', 1.5);
grid on;
xlabel('Time (s)');
ylabel('Control');
title('PAN: PID Output');

subplot(3,3,7);
histogram(tilt_error, 30);
grid on;
xlabel('Error (°)');
title('TILT: Error Distribution');

subplot(3,3,8);
histogram(pan_error, 30);
grid on;
xlabel('Error (°)');
title('PAN: Error Distribution');

subplot(3,3,9);
axis off;
text(0.1, 0.95, 'OPTIMIZED PID GAINS', 'FontSize', 14, 'FontWeight', 'bold');
text(0.1, 0.80, 'TILT MOTOR:', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'blue');
text(0.1, 0.70, sprintf('Kp = %.4f', optimal_tilt(1)), 'FontSize', 10);
text(0.1, 0.60, sprintf('Ki = %.4f', optimal_tilt(2)), 'FontSize', 10);
text(0.1, 0.50, sprintf('Kd = %.4f', optimal_tilt(3)), 'FontSize', 10);
text(0.1, 0.35, 'PAN MOTOR:', 'FontSize', 12, 'FontWeight', 'bold', 'Color', 'blue');
text(0.1, 0.25, sprintf('Kp = %.4f', optimal_pan(1)), 'FontSize', 10);
text(0.1, 0.15, sprintf('Ki = %.4f', optimal_pan(2)), 'FontSize', 10);
text(0.1, 0.05, sprintf('Kd = %.4f', optimal_pan(3)), 'FontSize', 10);

sgtitle('ESP32 Gimbal - Auto-Optimized PID', 'FontSize', 16, 'FontWeight', 'bold');

%% SAVE RESULTS
% ========================================================================
fprintf('========================================\n');
fprintf('UPDATE ESP32 CODE\n');
fprintf('========================================\n\n');

fprintf('// Tilt Motor (Motor 2)\n');
fprintf('PID_Controller pidTilt(\n');
fprintf('    %.4ff,  // Kp\n', optimal_tilt(1));
fprintf('    %.4ff,  // Ki\n', optimal_tilt(2));
fprintf('    %.4ff,  // Kd\n', optimal_tilt(3));
fprintf('    90.0f * DEG_TO_RAD\n');
fprintf(');\n\n');

fprintf('// Pan Motor (Motor 1)\n');
fprintf('PID_Controller pidPan(\n');
fprintf('    %.4ff,  // Kp\n', optimal_pan(1));
fprintf('    %.4ff,  // Ki\n', optimal_pan(2));
fprintf('    %.4ff,  // Kd\n', optimal_pan(3));
fprintf('    0.0f\n');
fprintf(');\n\n');

% Save to file
fid = fopen('esp32_optimal_pid.txt', 'w');
fprintf(fid, 'ESP32 GIMBAL OPTIMAL PID GAINS\n');
fprintf(fid, '==============================\n\n');
fprintf(fid, 'TILT MOTOR (Motor 2):\n');
fprintf(fid, 'Kp = %.6f\n', optimal_tilt(1));
fprintf(fid, 'Ki = %.6f\n', optimal_tilt(2));
fprintf(fid, 'Kd = %.6f\n\n', optimal_tilt(3));
fprintf(fid, 'PAN MOTOR (Motor 1):\n');
fprintf(fid, 'Kp = %.6f\n', optimal_pan(1));
fprintf(fid, 'Ki = %.6f\n', optimal_pan(2));
fprintf(fid, 'Kd = %.6f\n', optimal_pan(3));
fclose(fid);

fprintf('Results saved to: esp32_optimal_pid.txt\n');

%% HELPER FUNCTION
% ========================================================================
function result = ternary(condition, true_val, false_val)
    if condition
        result = true_val;
    else
        result = false_val;
    end
end
