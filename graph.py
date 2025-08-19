import pandas as pd
import matplotlib.pyplot as plt

# 讀取 CSV
df = pd.read_csv('./train_data/merged_20250619_032924.csv')

# 建立主圖表和副 Y 軸
fig, ax1 = plt.subplots()

# 主 Y 軸：Position
color = 'tab:blue'
ax1.set_xlabel('Time Step')
ax1.set_ylabel('Position', color=color)
ax1.plot(df.index, df['Position'], color=color, label='Position')
ax1.tick_params(axis='y', labelcolor=color)

# 副 Y 軸：Speed
ax2 = ax1.twinx()
color = 'tab:red'
ax2.set_ylabel('Speed', color=color)
ax2.plot(df.index, df['Speed'], color=color, label='Speed')
ax2.tick_params(axis='y', labelcolor=color)

# 顯示圖表
plt.title('Position and Speed Over Time')
plt.grid(True)
plt.tight_layout()
plt.show()
