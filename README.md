# Greenhouse Temperature Control: Cascade PID with Smith Predictor

本仓库用于《过程控制工程》课程论文：

**基于串级 PID 与 Smith 预估补偿的温室温度控制系统设计与仿真**

## 1. 项目简介

本项目围绕温室温度过程控制问题，建立一阶惯性加纯滞后对象模型，并对比三种控制方案：

1. 单回路 PID 控制；
2. 串级 PID 控制；
3. 串级 PID + Smith 预估补偿控制。

仿真对象具有大惯性、纯滞后和外界扰动等过程控制特征。代码会自动生成温度响应曲线、控制量输出曲线，并计算超调量、上升时间、调节时间和 IAE 等性能指标。

## 2. 仓库结构

```text
greenhouse_pid_smith_control/
├── README.md
├── LICENSE
├── .gitignore
├── src/
│   ├── main_greenhouse_control.m
│   ├── greenhouse_params.m
│   ├── simulate_single_pid.m
│   ├── simulate_cascade_pid.m
│   ├── simulate_cascade_smith.m
│   ├── calc_metrics.m
│   └── save_results_table.m
├── figures/
│   └── 运行后自动生成图片
├── results/
│   └── 运行后自动生成 CSV 指标表
└── docs/
    └── paper_note.md
```

## 3. 运行环境

推荐环境：

- MATLAB R2020a 及以上版本；
- 不强制依赖 Simulink；
- 不强制依赖 Control System Toolbox。

代码采用离散差分方式模拟一阶惯性加纯滞后对象，因此普通 MATLAB 即可运行。

## 4. 快速运行

进入项目根目录后，在 MATLAB 命令行运行：

```matlab
cd src
main_greenhouse_control
```

运行后会自动生成：

- `figures/temperature_response_comparison.png`
- `figures/control_output_comparison.png`
- `results/performance_metrics.csv`

## 5. 仿真设置

默认参数如下：

| 参数 | 数值 |
|---|---|
| 采样周期 | 1 s |
| 仿真时长 | 3600 s |
| 初始温度 | 20 ℃ |
| 设定温度 | 25 ℃ |
| 扰动加入时刻 | 2400 s |
| 温室主对象 | 1/(520s+1)，纯滞后 70 s |
| 执行机构/局部热交换模型 | 1/(80s+1)，纯滞后 10 s |

## 6. 说明

本代码主要用于课程论文仿真验证。若用于真实温室控制系统，需要根据现场阶跃响应数据重新辨识对象参数，并重新整定 PID 参数。
