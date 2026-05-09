function out = simulate_cascade_pid(p)
% simulate_cascade_pid
% 串级 PID 控制：外环控制温室温度，内环控制执行机构/局部热交换环节

Ts = p.Ts;
t = (0:p.n-1)' * Ts;

y = zeros(p.n, 1);
u = zeros(p.n, 1);
rin = zeros(p.n, 1);
act_y = zeros(p.n, 1);
d = zeros(p.n, 1);

y(1) = p.T_initial;

% 延迟
plant_delay_steps = max(1, round(p.Lp_plant / Ts));
act_delay_steps = max(1, round(p.L_act / Ts));
plant_buffer = zeros(plant_delay_steps + 1, 1);
act_buffer = zeros(act_delay_steps + 1, 1);

% PID 状态
e_outer_int = 0;
e_outer_prev = p.T_set - y(1);
e_inner_int = 0;
e_inner_prev = 0;

d_state = 0;

for k = 2:p.n
    % 扰动
    d_in = 0;
    if t(k) >= p.disturbance_time
        d_in = p.disturbance_amp;
    end
    d_state = d_state + Ts / p.Td * (d_in - d_state);
    d(k) = d_state;

    % 外环 PID
    e_outer = p.T_set - y(k-1);
    e_outer_int = e_outer_int + e_outer * Ts;
    e_outer_der = (e_outer - e_outer_prev) / Ts;

    rin_raw = p.outer.Kp * e_outer + p.outer.Ki * e_outer_int + p.outer.Kd * e_outer_der;
    rin(k) = min(max(rin_raw, p.rin_min), p.rin_max);

    if rin(k) ~= rin_raw
        e_outer_int = e_outer_int - e_outer * Ts;
    end
    e_outer_prev = e_outer;

    % 内环 PID
    e_inner = rin(k) - act_y(k-1);
    e_inner_int = e_inner_int + e_inner * Ts;
    e_inner_der = (e_inner - e_inner_prev) / Ts;

    u_raw = p.inner.Kp * e_inner + p.inner.Ki * e_inner_int + p.inner.Kd * e_inner_der;
    u(k) = min(max(u_raw, p.u_min), p.u_max);

    if u(k) ~= u_raw
        e_inner_int = e_inner_int - e_inner * Ts;
    end
    e_inner_prev = e_inner;

    % 执行机构/局部热交换一阶环节
    act_buffer = [u(k); act_buffer(1:end-1)];
    u_delay = act_buffer(end);
    act_y(k) = act_y(k-1) + Ts / p.T_act * (p.K_act * u_delay - act_y(k-1));

    % 温室主对象
    plant_buffer = [act_y(k); plant_buffer(1:end-1)];
    act_delay = plant_buffer(end);

    y_ss = p.T_initial + p.Kp_plant * (act_delay / 100) * 8 + d(k);
    y(k) = y(k-1) + Ts / p.Tp_plant * (y_ss - y(k-1));
end

out.t = t;
out.y = y;
out.u = u;
out.rin = rin;
out.act_y = act_y;
out.d = d;
end
