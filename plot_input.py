import numpy as np
import matplotlib.pyplot as plt

# 產生三段式 Ramp 訊號
def three_stage_ramp(time, t_peak1, t_peak2, slope_up1=1, slope_down=-1, slope_up2=0.5):
    y = np.piecewise(
        time,
        [time < t_peak1, 
         (time >= t_peak1) & (time < t_peak2), 
         time >= t_peak2],
        [lambda t: slope_up1 * t,
         lambda t: slope_down * (t - t_peak1) + slope_up1 * t_peak1,
         lambda t: slope_up2 * (t - t_peak2) + (slope_down * (t_peak2 - t_peak1) + slope_up1 * t_peak1)]
    )
    return y

# 產生 sin + cos 合成訊號
def sin_cos_input(time, A_sin=1.0, A_cos=0.5, f_sin=1.0, f_cos=1.0, phase_sin=0.0, phase_cos=0.0):
    y = A_sin * np.sin(2 * np.pi * f_sin * time + phase_sin) + A_cos * np.cos(2 * np.pi * f_cos * time + phase_cos)
    return y

def main():
    # 建立時間軸
    duration = 10  # 秒
    dt = 0.01  # 時間間隔
    t = np.arange(0, duration, dt)

    t_peak1 = np.random.uniform(2, 4)  # Random peak time
    t_peak2 = np.random.uniform(6, 8)
    slope_up1 = np.random.uniform(200000, 320000)
    slope_down = np.random.uniform(-360000, -200000)
    slope_up2 = np.random.uniform(200000, 320000)

    ramp_signal = three_stage_ramp(t, t_peak1=t_peak1, t_peak2=t_peak2, slope_up1=slope_up1, slope_down=slope_down, slope_up2=slope_up2)

    A_sin = np.random.uniform(150000, 250000)
    A_cos = np.random.uniform(150000, 250000)
    f_sin = np.random.uniform(0.1, 1.0)  # 頻率範圍
    f_cos = np.random.uniform(0.1, 1.0)
    phase_sin = np.random.uniform(0, 2 * np.pi)  # 隨機相位
    phase_cos = np.random.uniform(0, 2 * np.pi)
    sin_cos_signal = sin_cos_input(t, A_sin=A_sin, A_cos=A_cos, f_sin=f_sin, f_cos=f_cos, phase_sin=phase_sin, phase_cos=phase_cos)

    # 繪圖
    plt.figure(figsize=(12, 6))

    plt.subplot(2, 1, 1)
    plt.plot(t, ramp_signal, label="Three-stage Ramp", color='tab:blue')
    plt.title("Three-stage Ramp Input")
    plt.xlabel("Time (s)")
    plt.ylabel("Amplitude")
    plt.grid(True)
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(t, sin_cos_signal, label="Sin + Cos Composite", color='tab:orange')
    plt.title("Sin + Cos Input")
    plt.xlabel("Time (s)")
    plt.ylabel("Amplitude")
    plt.grid(True)
    plt.legend()

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    main()
