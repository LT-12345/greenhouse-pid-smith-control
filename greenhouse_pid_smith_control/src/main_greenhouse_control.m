%% main_greenhouse_control.m
% 基于串级 PID 与 Smith 预估补偿的温室温度控制系统仿真
% 说明：
% 1. 不依赖 Simulink，采用离散差分模型模拟一阶惯性加纯滞后对象；
% 2. 自动对比单回路 PID、串级 PID、串级 PID + Smith 预估补偿；
% 3. 自动保存温度响应曲线、控制量曲线和性能指标表。

clear; clc; close all;

% 路径设置
this_dir = fileparts(mfilename('fullpath'));
project_root = fileparts(this_dir);
fig_dir = fullfile(project_root, 'figures');
res_dir = fullfile(project_root, 'results');

if ~exist(fig_dir, 'dir'); mkdir(fig_dir); end
if ~exist(res_dir, 'dir'); mkdir(res_dir); end

% 加载参数
p = greenhouse_params();

% 执行三种控制方案
single_pid = simulate_single_pid(p);
cascade_pid = simulate_cascade_pid(p);
cascade_smith = simulate_cascade_smith(p);

% 计算性能指标
metrics_single = calc_metrics(single_pid.t, single_pid.y, p.T_set);
metrics_cascade = calc_metrics(cascade_pid.t, cascade_pid.y, p.T_set);
metrics_smith = calc_metrics(cascade_smith.t, cascade_smith.y, p.T_set);

metrics = [
    metrics_single;
    metrics_cascade;
    metrics_smith
];

scheme_names = {
    'Single PID';
    'Cascade PID';
    'Cascade PID + Smith'
};

save_results_table(metrics, scheme_names, fullfile(res_dir, 'performance_metrics.csv'));

% 绘制温度响应对比图
figure('Color', 'w', 'Position', [100, 100, 920, 520]);
plot(single_pid.t, single_pid.y, 'LineWidth', 1.8); hold on;
plot(cascade_pid.t, cascade_pid.y, 'LineWidth', 1.8);
plot(cascade_smith.t, cascade_smith.y, 'LineWidth', 1.8);
yline(p.T_set, '--', 'Setpoint', 'LineWidth', 1.2);
xline(p.disturbance_time, ':', 'Disturbance', 'LineWidth', 1.2);
grid on;
xlabel('Time / s');
ylabel('Greenhouse Temperature / ℃');
title('Temperature Response Comparison');
legend('Single PID', 'Cascade PID', 'Cascade PID + Smith', 'Setpoint', 'Disturbance', ...
       'Location', 'southeast');
saveas(gcf, fullfile(fig_dir, 'temperature_response_comparison.png'));

% 绘制控制量输出对比图
figure('Color', 'w', 'Position', [130, 130, 920, 520]);
plot(single_pid.t, single_pid.u, 'LineWidth', 1.8); hold on;
plot(cascade_pid.t, cascade_pid.u, 'LineWidth', 1.8);
plot(cascade_smith.t, cascade_smith.u, 'LineWidth', 1.8);
xline(p.disturbance_time, ':', 'Disturbance', 'LineWidth', 1.2);
grid on;
xlabel('Time / s');
ylabel('Control Output / %');
title('Control Output Comparison');
legend('Single PID', 'Cascade PID', 'Cascade PID + Smith', 'Disturbance', ...
       'Location', 'northeast');
ylim([0, 100]);
saveas(gcf, fullfile(fig_dir, 'control_output_comparison.png'));

% 打印结果
disp(' ');
disp('=== Greenhouse Temperature Control Simulation Finished ===');
disp('Figures saved to:');
disp(fig_dir);
disp('Metrics saved to:');
disp(fullfile(res_dir, 'performance_metrics.csv'));
disp(' ');
disp('Performance metrics:');
disp(array2table(metrics, ...
    'VariableNames', {'Overshoot_percent','RiseTime_s','SettlingTime_s','IAE'}, ...
    'RowNames', scheme_names));
