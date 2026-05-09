function out = simulate_cascade_smith(p)
% simulate_cascade_smith
% 串级 PID + Smith 预估补偿控制
%
% 说明：
% Smith 分支使用无滞后模型构造预测反馈：
% y_feedback = y0 + (y_actual - ym_delay)
% 其中 y0 为无滞后模型输出，ym_delay 为含滞后模型输出。

Ts = p.Ts;
t = (0:p.n-1)' * Ts;

y = zeros(p.n, 1);
u = zeros(p.n, 1);
rin = zeros(p.n, 1);
act_y = zeros(p.n, 1);
d = zeros(p.n, 1);

y0 = zeros(p.n, 1);          % 无滞后模型输出
ym = zeros(p.n, 1);          % 含滞后模型输出
y_feedback = zeros(p.n, 1);  % Smith 预测反馈

y(1) = p.T_initial;
y0(1) = p.T_initial;
ym(1) = p.T_initial;
y_feedback(1) = p.T_initial;

% 延迟
plant_delay_steps = max(1, round(p.Lp_plant / Ts));
act_delay_steps = max(1, round(p.L_act / Ts));
plant_buffer = zeros(plant_delay_steps + 1, 1);
model_buffer = zeros(plant_delay_steps + 1, 1);
act_buffer = zeros(act_delay_steps + 1, 1);

% PID 状态
e_outer_int = 0;
e_outer_prev = p.T_set - y_feedback(1);
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

    % Smith 预测反馈参与外环
    e_outer = p.T_set - y_feedback(k-1);
    e_outer_int = e_outer_int + e_outer * Ts;
    e_outer_der = (e_outer - e_outer_prev) / Ts;

    rin_raw = p.smith.Kp * e_outer + p.smith.Ki * e_outer_int + p.smith.Kd * e_outer_der;
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

    % 执行机构动态
    act_buffer = [u(k); act_buffer(1:end-1)];
    u_delay = act_buffer(end);
    act_y(k) = act_y(k-1) + Ts / p.T_act * (p.K_act * u_delay - act_y(k-1));

    % 真实温室对象，含纯滞后
    plant_buffer = [act_y(k); plant_buffer(1:end-1)];
    act_delay = plant_buffer(end);

    y_ss = p.T_initial + p.Kp_plant * (act_delay / 100) * 8 + d(k);
    y(k) = y(k-1) + Ts / p.Tp_plant * (y_ss - y(k-1));

    % Smith 无滞后模型
    y0_ss = p.T_initial + p.Kp_plant * (act_y(k) / 100) * 8;
    y0(k) = y0(k-1) + Ts / p.Tp_plant * (y0_ss - y0(k-1));

    % Smith 含滞后模型
    model_buffer = [act_y(k); model_buffer(1:end-1)];
    model_act_delay = model_buffer(end);
    ym_ss = p.T_initial + p.Kp_plant * (model_act_delay / 100) * 8;
    ym(k) = ym(k-1) + Ts / p.Tp_plant * (ym_ss - ym(k-1));

    % 预测反馈修正
    y_feedback(k) = y0(k) + (y(k) - ym(k));
end

out.t = t;
out.y = y;
out.u = u;
out.rin = rin;
out.act_y = act_y;
out.y0 = y0;
out.ym = ym;
out.y_feedback = y_feedback;
out.d = d;
end
