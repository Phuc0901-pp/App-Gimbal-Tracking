%% VISUAL SERVOING SIMULATION - PD vs PD+Deadband
% So sánh: PD only vs PD+Deadband
% Author: Gemini Assistant
% Date: 2024

clear; close all; clc;

%% THÔNG SỐ HỆ THỐNG
Ts = 0.02;          % Sample time (s) - 50Hz
T_sim = 10;         % Thời gian mô phỏng (s)
t = 0:Ts:T_sim;     % Vector thời gian
N = length(t);

% Motor parameters
tau_motor = 0.1;    % Time constant của motor (s)

%% THÔNG SỐ PD - TỪ CODE ESP32
Kp = 1.2;           % Proportional gain
Kd = 0.08;          % Derivative gain
MAX_VEL = 30.0;     % deg/s max velocity

% Deadband options
DEADBAND_VALUES = [0, 1.0, 2.0];  % So sánh: không deadband, 1°, 2°

%% TARGET - Step + Noise (mô phỏng nhiễu cảm biến)
target = zeros(1, N);
target(t > 1) = 20;    % Step to 20° at t=1s

% Thêm nhiễu nhỏ để mô phỏng nhiễu cảm biến
noise = 0.5 * randn(1, N);  % ±0.5° noise
target_noisy = target + noise;

%% MÔ PHỎNG
colors = {'b-', 'r-', 'g-'};
labels = {'PD (no deadband)', 'PD + Deadband 1°', 'PD + Deadband 2°'};

figure('Position', [100 100 1400 900], 'Name', 'PD vs PD+Deadband');

% Lưu kết quả
results = struct();

for d_idx = 1:length(DEADBAND_VALUES)
    DEADBAND = DEADBAND_VALUES(d_idx);
    
    fprintf('Simulating PD with Deadband = %.1f°...\n', DEADBAND);
    
    gimbal_pos = zeros(1, N);
    gimbal_vel = zeros(1, N);
    error_raw = zeros(1, N);
    error_used = zeros(1, N);
    prev_error = 0;
    
    for k = 2:N
        % Camera error (với nhiễu)
        error_raw(k) = target_noisy(k) - gimbal_pos(k-1);
        
        % Apply deadband
        if DEADBAND > 0 && abs(error_raw(k)) < DEADBAND
            error_used(k) = 0;
        else
            error_used(k) = error_raw(k);
        end
        
        % PD Control
        d_error = (error_used(k) - prev_error) / Ts;
        prev_error = error_used(k);
        
        velocity_cmd = Kp * error_used(k) + Kd * d_error;
        velocity_cmd = max(-MAX_VEL, min(MAX_VEL, velocity_cmd));
        
        % Motor dynamics
        gimbal_vel(k) = gimbal_vel(k-1) + (velocity_cmd - gimbal_vel(k-1)) * Ts / tau_motor;
        gimbal_pos(k) = gimbal_pos(k-1) + gimbal_vel(k) * Ts;
    end
    
    % Lưu kết quả
    results(d_idx).deadband = DEADBAND;
    results(d_idx).pos = gimbal_pos;
    results(d_idx).vel = gimbal_vel;
    results(d_idx).error_raw = error_raw;
    results(d_idx).error_used = error_used;
end

%% VẼ ĐỒ THỊ

% Subplot 1: Step Response
subplot(2,2,1);
plot(t, target, 'k--', 'LineWidth', 2, 'DisplayName', 'Target (20°)');
hold on;
for d_idx = 1:length(DEADBAND_VALUES)
    plot(t, results(d_idx).pos, colors{d_idx}, 'LineWidth', 1.5, 'DisplayName', labels{d_idx});
end
xlabel('Time (s)');
ylabel('Angle (deg)');
title('Step Response - Deadband Comparison');
legend('Location', 'southeast');
grid on;
ylim([0 25]);

% Subplot 2: Error (Zoomed in steady-state)
subplot(2,2,2);
for d_idx = 1:length(DEADBAND_VALUES)
    plot(t, results(d_idx).error_raw, colors{d_idx}, 'LineWidth', 1, 'DisplayName', labels{d_idx});
    hold on;
end
plot(t, zeros(size(t)), 'k--', 'LineWidth', 1);
xlabel('Time (s)');
ylabel('Error (deg)');
title('Tracking Error (with noise)');
legend('Location', 'northeast');
grid on;
xlim([5 10]);  % Zoom vào steady-state
ylim([-5 5]);

% Subplot 3: Velocity Command
subplot(2,2,3);
for d_idx = 1:length(DEADBAND_VALUES)
    plot(t, results(d_idx).vel, colors{d_idx}, 'LineWidth', 1.5, 'DisplayName', labels{d_idx});
    hold on;
end
xlabel('Time (s)');
ylabel('Velocity (deg/s)');
title('Velocity Command - Deadband giảm jitter');
legend('Location', 'northeast');
grid on;
xlim([5 10]);  % Zoom vào steady-state

% Subplot 4: Velocity RMS (đo lường jitter)
subplot(2,2,4);
rms_values = zeros(1, length(DEADBAND_VALUES));
for d_idx = 1:length(DEADBAND_VALUES)
    % Tính RMS velocity trong steady-state (t > 5s)
    idx_ss = t > 5;
    rms_values(d_idx) = rms(results(d_idx).vel(idx_ss));
end
bar(DEADBAND_VALUES, rms_values);
xlabel('Deadband (deg)');
ylabel('RMS Velocity (deg/s)');
title('Jitter Analysis - Lower is Better');
grid on;

%% PERFORMANCE METRICS
fprintf('\n========== PERFORMANCE METRICS ==========\n');

for d_idx = 1:length(DEADBAND_VALUES)
    DEADBAND = DEADBAND_VALUES(d_idx);
    gimbal_pos = results(d_idx).pos;
    gimbal_vel = results(d_idx).vel;
    
    % Steady-state error (average in last 2 seconds)
    idx_ss = t > 8;
    ss_error = mean(abs(target(idx_ss) - gimbal_pos(idx_ss)));
    
    % Rise time
    idx_90 = find(gimbal_pos >= 0.9 * 20, 1);
    if ~isempty(idx_90)
        rise_time = t(idx_90) - 1;
    else
        rise_time = NaN;
    end
    
    % Overshoot
    overshoot = (max(gimbal_pos) - 20) / 20 * 100;
    
    % Jitter (RMS velocity in steady-state)
    jitter_rms = rms(gimbal_vel(idx_ss));
    
    fprintf('\n%s:\n', labels{d_idx});
    fprintf('  Steady-state error: %.4f deg\n', ss_error);
    fprintf('  Rise time (10-90%%): %.3f s\n', rise_time);
    fprintf('  Overshoot: %.1f %%\n', max(0, overshoot));
    fprintf('  Jitter (RMS vel): %.4f deg/s\n', jitter_rms);
end

%% KẾT LUẬN
fprintf('\n========== KẾT LUẬN ==========\n');
fprintf('Deadband giúp:\n');
fprintf('  ✓ Giảm jitter khi có nhiễu cảm biến\n');
fprintf('  ✓ Tiết kiệm năng lượng motor (ít command nhỏ)\n');
fprintf('Deadband không giúp:\n');
fprintf('  ✗ Triệt tiêu sai số xác lập (ngược lại, có thể tạo thêm!)\n');
fprintf('\nĐề xuất cho ESP32:\n');
fprintf('  Kp=1.2, Kd=0.08, Ki=0, DEADBAND=1° (optional)\n');
