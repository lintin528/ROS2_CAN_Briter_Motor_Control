import pandas as pd
import matplotlib.pyplot as plt


df1 = pd.read_csv('./data_pos_pos/data_pos_20250603_110328.csv')
df2 = pd.read_csv('./data_pos_speed/data_20250603_110328.csv')

# plt.figure(figsize=(12, 6))
# plt.plot(df['Timestamp'], df['Encoder Position'], label='Encoder Position', color='blue')
# plt.plot(df['Timestamp'], df['Target Position'], label='Target Position', linestyle='--', color='red')
# plt.plot(df2['Timestamp'], df2['Encoder Position'], label='apeed', color='green')

fig, ax1 = plt.subplots(figsize=(12, 6))
color1 = 'tab:blue'
ax1.set_xlabel('Time (s)')
ax1.set_ylabel('Encoder 1 Position', color=color1)
ax1.plot(df1['Timestamp'], df1['Encoder Position'], color=color1, label='Encoder Position')
ax1.tick_params(axis='y', labelcolor=color1)

# 建立第二個 y 軸（右邊的 y 軸）
ax2 = ax1.twinx()  # 共享 x 軸

color2 = 'tab:green'
ax2.set_ylabel('Encoder 2 Position', color=color2)
ax2.plot(df2['Timestamp'], df2['Encoder Position'], color=color2, label='speed')
ax2.tick_params(axis='y', labelcolor=color2)

plt.title('Encoder Position over Time')
plt.xlabel('Time (s)')
plt.ylabel('Encoder Position')
plt.grid(True)
plt.legend()

plt.show()
