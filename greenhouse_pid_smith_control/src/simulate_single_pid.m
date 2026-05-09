function out = simulate_single_pid(p)
% simulate_single_pid
% 单回路 PID 控制温室温度对象

Ts = p.Ts;
t = (0:p.n-1)' * Ts;

y = zeros(p.n, 1);
u = zeros(p.n, 1);
d = zeros(p.n, 1);

y(1) = p.T_initial;

% 延迟步数
delay_steps = max(1, round(p.Lp_plant / Ts));
u_buffer = zeros(delay_steps + 1, 1);

% PID 状态
e_int = 0;
e_prev = p.T_set - y(1);

% 扰动状态
d_state = 0;

for k = 2:p.n
    % 外界扰动，一阶动态进入温室
    d_in = 0;
    if t(k) >= p.disturbance_time
        d_in = p.disturbance_amp;
    end
    d_state = d_state + Ts / p.Td * (d_in - d_state);
    d(k) = d_state;

    % PID 控制
    e = p.T_set - y(k-1);
    e_int = e_int + e * Ts;
    e_der = (e - e_prev) / Ts;

    u_raw = p.single.Kp * e + p.single.Ki * e_int + p.single.Kd * e_der;
    u(k) = min(max(u_raw, p.u_min), p.u_max);

    % 简单抗积分饱和
    if u(k) ~= u_raw
        e_int = e_int - e * Ts;
    end
    e_prev = e;

    % 纯滞后
    u_buffer = [u(k); u_buffer(1:end-1)];
    u_delay = u_buffer(end);

    % 一阶惯性对象
    y_ss = p.T_initial + p.Kp_plant * (u_delay / 100) * 8 + d(k);
    y(k) = y(k-1) + Ts / p.Tp_plant * (y_ss - y(k-1));
end

out.t = t;
out.y = y;
out.u = u;
out.d = d;
end
