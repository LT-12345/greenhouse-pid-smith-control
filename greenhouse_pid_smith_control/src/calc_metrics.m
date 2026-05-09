function metrics = calc_metrics(t, y, setpoint)
% calc_metrics
% 计算温度响应性能指标：
% 1. 超调量 Overshoot / %
% 2. 上升时间 Rise Time / s
% 3. 调节时间 Settling Time / s，按 ±2% 误差带计算
% 4. 积分绝对误差 IAE

Ts = t(2) - t(1);
y0 = y(1);
delta = setpoint - y0;

if abs(delta) < 1e-9
    metrics = [0, 0, 0, 0];
    return;
end

% 超调量
if delta > 0
    overshoot = max(0, (max(y) - setpoint) / abs(delta) * 100);
else
    overshoot = max(0, (setpoint - min(y)) / abs(delta) * 100);
end

% 上升时间：10% 到 90%
y10 = y0 + 0.1 * delta;
y90 = y0 + 0.9 * delta;

if delta > 0
    idx10 = find(y >= y10, 1, 'first');
    idx90 = find(y >= y90, 1, 'first');
else
    idx10 = find(y <= y10, 1, 'first');
    idx90 = find(y <= y90, 1, 'first');
end

if isempty(idx10) || isempty(idx90)
    rise_time = NaN;
else
    rise_time = t(idx90) - t(idx10);
end

% 调节时间：±2% 误差带
band = 0.02 * abs(delta);
err = abs(y - setpoint);
settling_time = NaN;

for k = 1:length(t)
    if all(err(k:end) <= band)
        settling_time = t(k);
        break;
    end
end

% IAE
iae = sum(abs(setpoint - y)) * Ts;

metrics = [overshoot, rise_time, settling_time, iae];
end
