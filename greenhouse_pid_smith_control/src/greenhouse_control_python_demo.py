"""
greenhouse_control_python_demo.py

可选 Python 版本仿真脚本。
如果没有 MATLAB，可以运行本脚本快速生成三种控制方案的温度响应曲线。

运行：
    python greenhouse_control_python_demo.py
"""

import os
import numpy as np
import matplotlib.pyplot as plt
import csv

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIG_DIR = os.path.join(PROJECT_ROOT, "figures")
RES_DIR = os.path.join(PROJECT_ROOT, "results")
os.makedirs(FIG_DIR, exist_ok=True)
os.makedirs(RES_DIR, exist_ok=True)


def clip(x, lo, hi):
    return max(lo, min(hi, x))


def metrics(t, y, sp):
    y0 = y[0]
    delta = sp - y0
    overshoot = max(0.0, (np.max(y) - sp) / abs(delta) * 100.0)
    y10 = y0 + 0.1 * delta
    y90 = y0 + 0.9 * delta
    idx10 = np.argmax(y >= y10)
    idx90 = np.argmax(y >= y90)
    rise = t[idx90] - t[idx10] if idx90 > idx10 else np.nan
    band = 0.02 * abs(delta)
    settling = np.nan
    for i in range(len(t)):
        if np.all(np.abs(y[i:] - sp) <= band):
            settling = t[i]
            break
    iae = np.sum(np.abs(sp - y))
    return overshoot, rise, settling, iae


def simulate(mode="single"):
    Ts = 1.0
    t_end = 3600
    n = int(t_end / Ts) + 1
    t = np.arange(n) * Ts

    T_initial = 20.0
    T_set = 25.0
    plant_T = 520.0
    plant_L = 70.0
    act_T = 80.0
    act_L = 10.0
    disturbance_time = 2400
    disturbance_amp = -1.2
    Td = 300.0

    y = np.zeros(n)
    u = np.zeros(n)
    act_y = np.zeros(n)
    y0 = np.zeros(n)
    ym = np.zeros(n)
    yfb = np.zeros(n)

    y[0] = T_initial
    y0[0] = T_initial
    ym[0] = T_initial
    yfb[0] = T_initial

    plant_delay = int(round(plant_L / Ts))
    act_delay = int(round(act_L / Ts))
    plant_buf = [0.0] * (plant_delay + 1)
    act_buf = [0.0] * (act_delay + 1)
    model_buf = [0.0] * (plant_delay + 1)

    d_state = 0.0

    if mode == "single":
        Kp, Ki, Kd = 2.4, 0.004, 80.0
    elif mode == "cascade":
        Kpo, Kio, Kdo = 2.6, 0.005, 70.0
        Kpi, Kii, Kdi = 1.8, 0.020, 5.0
    else:
        Kpo, Kio, Kdo = 5.0, 0.010, 30.0
        Kpi, Kii, Kdi = 1.8, 0.020, 5.0

    eint = 0.0
    eprev = T_set - y[0]
    eoi = 0.0
    eop = T_set - y[0]
    eii = 0.0
    eip = 0.0

    rin = np.zeros(n)

    for k in range(1, n):
        d_in = disturbance_amp if t[k] >= disturbance_time else 0.0
        d_state += Ts / Td * (d_in - d_state)

        if mode == "single":
            e = T_set - y[k - 1]
            eint += e * Ts
            eder = (e - eprev) / Ts
            raw = Kp * e + Ki * eint + Kd * eder
            u[k] = clip(raw, 0.0, 100.0)
            if u[k] != raw:
                eint -= e * Ts
            eprev = e

            plant_buf = [u[k]] + plant_buf[:-1]
            u_delay = plant_buf[-1]
            yss = T_initial + (u_delay / 100.0) * 8.0 + d_state
            y[k] = y[k - 1] + Ts / plant_T * (yss - y[k - 1])
        else:
            feedback = yfb[k - 1] if mode == "smith" else y[k - 1]
            eo = T_set - feedback
            eoi += eo * Ts
            eod = (eo - eop) / Ts
            raw_r = Kpo * eo + Kio * eoi + Kdo * eod
            rin[k] = clip(raw_r, 0.0, 100.0)
            if rin[k] != raw_r:
                eoi -= eo * Ts
            eop = eo

            ei = rin[k] - act_y[k - 1]
            eii += ei * Ts
            eid = (ei - eip) / Ts
            raw_u = Kpi * ei + Kii * eii + Kdi * eid
            u[k] = clip(raw_u, 0.0, 100.0)
            if u[k] != raw_u:
                eii -= ei * Ts
            eip = ei

            act_buf = [u[k]] + act_buf[:-1]
            u_delay = act_buf[-1]
            act_y[k] = act_y[k - 1] + Ts / act_T * (u_delay - act_y[k - 1])

            plant_buf = [act_y[k]] + plant_buf[:-1]
            act_delay_value = plant_buf[-1]
            yss = T_initial + (act_delay_value / 100.0) * 8.0 + d_state
            y[k] = y[k - 1] + Ts / plant_T * (yss - y[k - 1])

            if mode == "smith":
                y0ss = T_initial + (act_y[k] / 100.0) * 8.0
                y0[k] = y0[k - 1] + Ts / plant_T * (y0ss - y0[k - 1])
                model_buf = [act_y[k]] + model_buf[:-1]
                ymss = T_initial + (model_buf[-1] / 100.0) * 8.0
                ym[k] = ym[k - 1] + Ts / plant_T * (ymss - ym[k - 1])
                yfb[k] = y0[k] + (y[k] - ym[k])

    return t, y, u


def main():
    schemes = [
        ("Single PID", "single"),
        ("Cascade PID", "cascade"),
        ("Cascade PID + Smith", "smith"),
    ]

    rows = []
    plt.figure(figsize=(9, 5))
    for name, mode in schemes:
        t, y, u = simulate(mode)
        plt.plot(t, y, linewidth=1.8, label=name)
        rows.append([name, *metrics(t, y, 25.0)])
    plt.axhline(25.0, linestyle="--", linewidth=1.2, label="Setpoint")
    plt.axvline(2400, linestyle=":", linewidth=1.2, label="Disturbance")
    plt.grid(True)
    plt.xlabel("Time / s")
    plt.ylabel("Greenhouse Temperature / ℃")
    plt.title("Temperature Response Comparison")
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(FIG_DIR, "temperature_response_comparison_python.png"), dpi=300)

    plt.figure(figsize=(9, 5))
    for name, mode in schemes:
        t, y, u = simulate(mode)
        plt.plot(t, u, linewidth=1.8, label=name)
    plt.axvline(2400, linestyle=":", linewidth=1.2, label="Disturbance")
    plt.grid(True)
    plt.xlabel("Time / s")
    plt.ylabel("Control Output / %")
    plt.title("Control Output Comparison")
    plt.ylim(0, 100)
    plt.legend()
    plt.tight_layout()
    plt.savefig(os.path.join(FIG_DIR, "control_output_comparison_python.png"), dpi=300)

    with open(os.path.join(RES_DIR, "performance_metrics_python.csv"), "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["Scheme", "Overshoot_percent", "RiseTime_s", "SettlingTime_s", "IAE"])
        writer.writerows(rows)

    print("Simulation finished.")
    print("Figures saved to:", FIG_DIR)
    print("Metrics saved to:", RES_DIR)


if __name__ == "__main__":
    main()
