import os
import pandas as pd


data_folder = './inter_data'
output_folder = './train_data'
os.makedirs(output_folder, exist_ok=True)

all_files = [f for f in os.listdir(data_folder) if f.endswith('.csv')]
pos_files = [f for f in all_files if f.startswith('interpolated_data_pos_')]

for pos_file in pos_files:
    timestamp = pos_file.replace('interpolated_data_pos_', '').replace('.csv', '')
    speed_file = f'interpolated_data_{timestamp}.csv'
    
    pos_path = os.path.join(data_folder, pos_file)
    speed_path = os.path.join(data_folder, speed_file)
    
    if not os.path.exists(speed_path):
        print(f"⚠️ 找不到 speed 檔案: {speed_file}")
        continue

    df_pos = pd.read_csv(pos_path)
    df_speed = pd.read_csv(speed_path)
    
    pos_col = df_pos.iloc[:, 1].copy()
    speed_col = df_speed.iloc[:, 1].copy()

    for i in range(1, len(speed_col) - 1):
        if abs(speed_col[i]) > 5000:
            speed_col[i] = speed_col[i-1]
    

    for i in range(3, len(pos_col) - 3):
        surrounding_values = list(pos_col[i-3:i]) + list(pos_col[i+1:i+4])
        reference_value = sum(surrounding_values) / len(surrounding_values)
    
        if abs(pos_col[i] - reference_value) > 20000:
            pos_col[i] = reference_value
    
    # 合併成新的 DataFrame
    merged_df = pd.DataFrame({
        'Position': pos_col,
        'Speed': speed_col
    })
    
    output_file = f'merged_{timestamp}.csv'
    output_path = os.path.join(output_folder, output_file)
    with open(output_path, 'w', newline='', encoding='utf-8') as f:
        merged_df.to_csv(f, index=False)
    print(f"✅ 合併完成：{output_file}")
