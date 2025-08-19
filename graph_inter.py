import matplotlib.pyplot as plt
import pandas as pd


# df = pd.read_csv('./train_data/merged_20250615_033157.csv')
df_speed = pd.read_csv('./data_pos_speed_sin/data_20250616_140132.csv')
df_interpolated = pd.read_csv('interpolated_output.csv')

fig, ax1 = plt.subplots(figsize=(12, 6))

# Position
ax1.set_xlabel('Timestamp (秒)')
ax1.set_ylabel('Position', color='black')

# ax1.scatter(df_speed['Timestamp'], df_speed['Encoder Position'], color='blue', label='原始 Encoder Position', s=10)
ax1.plot(df_interpolated['Timestamp'], df_interpolated['Encoder Position'], color='cyan', label='插值後 Encoder Position')


# ax1.scatter(df_speed['Timestamp'], df_speed['Target Position'], color='red', label='原始 Target Position', s=10)
# ax1.plot(df_interpolated['Timestamp'], df_interpolated['Target Position'], color='orange', label='插值後 Target Position')

ax1.tick_params(axis='y', labelcolor='black')

# Speed
ax2 = ax1.twinx()
ax2.set_ylabel('Speed', color='green')

# 原始 Speed
ax2.scatter(df_speed['Timestamp'], df_speed['Encoder Position'], color='green', label='原始 Speed', s=10)

# 插值後 Speed
ax2.plot(df_speed['Timestamp'], df_speed['Encoder Position'], color='lime', label='插值後 Speed')

ax2.tick_params(axis='y', labelcolor='green')

lines_1, labels_1 = ax1.get_legend_handles_labels()
lines_2, labels_2 = ax2.get_legend_handles_labels()
ax1.legend(lines_1 + lines_2, labels_1 + labels_2, loc='upper left')

plt.title('Encoder Position & Target Position 與 Speed 插值前後比較')
plt.grid(True)
plt.tight_layout()
plt.show()
