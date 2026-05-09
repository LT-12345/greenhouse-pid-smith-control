function p = greenhouse_params()
% greenhouse_params
% 温室温度控制仿真参数设置

% 基本仿真参数
p.Ts = 1;                     % 采样周期 / s
p.t_end = 3600;               % 仿真时长 / s
p.n = p.t_end / p.Ts + 1;     % 仿真步数

% 温度条件
p.T_initial = 20;             % 初始温度 / ℃
p.T_set = 25;                 % 设定温度 / ℃

% 温室主对象：G_p(s)=K*exp(-Ls)/(Ts+1)
p.Kp_plant = 1.0;
p.Tp_plant = 520;             % 主对象时间常数 / s
p.Lp_plant = 70;              % 主对象纯滞后 / s

% 执行机构/局部热交换模型：G_v(s)=K_v*exp(-L_v s)/(T_v s+1)
p.K_act = 1.0;
p.T_act = 80;                 % 执行机构时间常数 / s
p.L_act = 10;                 % 执行机构纯滞后 / s

% 扰动通道
p.disturbance_time = 2400;    % 扰动加入时刻 / s
p.disturbance_amp = -1.2;     % 等效降温扰动幅值 / ℃
p.Td = 300;                   % 扰动通道时间常数 / s

% 控制器参数
% 单回路 PID
p.single.Kp = 2.4;
p.single.Ki = 0.004;
p.single.Kd = 80;

% 串级 PID 外环
p.outer.Kp = 2.6;
p.outer.Ki = 0.005;
p.outer.Kd = 70;

% 串级 PID 内环
p.inner.Kp = 1.8;
p.inner.Ki = 0.020;
p.inner.Kd = 5;

% Smith 方案主控制器
p.smith.Kp = 5.0;
p.smith.Ki = 0.010;
p.smith.Kd = 30;

% 控制量限制
p.u_min = 0;
p.u_max = 100;

% 副回路设定值限制
p.rin_min = 0;
p.rin_max = 100;

end
