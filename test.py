import pandas as pd
import numpy as np
from pathlib import Path


input_folder = Path('./data_pos_pos_sin')
output_folder = Path('./inter_data')
output_folder.mkdir(exist_ok=True)

csv_files = list(input_folder.glob('*.csv'))

for csv_file in csv_files:
    df = pd.read_csv(csv_file)

    if not {'Timestamp', 'Encoder Position', 'Target Position'}.issubset(df.columns):
        print(f"⚠️ {csv_file.name} 缺少必要欄位，跳過處理。")
        continue

    new_timestamp = np.arange(df['Timestamp'].min(), df['Timestamp'].max(), 0.01)

    encoder_interp = np.interp(new_timestamp, df['Timestamp'], df['Encoder Position'])
    target_interp = np.interp(new_timestamp, df['Timestamp'], df['Target Position'])

    df_interpolated = pd.DataFrame({
        'Timestamp': new_timestamp,
        'Encoder Position': encoder_interp,
        'Target Position': target_interp
    })
    df_interpolated_1000 = df_interpolated.head(1000)
    output_path = output_folder / f'interpolated_{csv_file.name}'
    df_interpolated_1000.to_csv(output_path, index=False)

    print(f"✅ 已儲存到: {output_path}")
