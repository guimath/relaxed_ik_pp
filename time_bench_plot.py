import matplotlib.pyplot as plt
import pandas as pd
import numpy as np

from pathlib import Path

ABS_PATH= Path(__file__).parent
def stat_all(df) :
    out = ""
    for name in ['single_ik', 'double_ik', 'motion_alone', 'total']:
        val = df[name].values/1000
        mean = np.mean(val)
        std = np.std(val)
        print(f'{name:12s} : {mean:.2f}+-{std:.2f}ms')
        out += f'${mean:.2f}\pm{std:.2f}$ms & '
    out = out[:-2] + '\\\\'
    print(out)

if __name__ == "__main__":
    file = ABS_PATH / 'ex_out' / 'xarm6' / 'data' / 'time_bench_data.csv'
    df = pd.read_csv(file, sep=',')
    df.dropna(inplace = True)
    df['rad'] = np.sqrt(df['x']**2+df['y']**2)
    df = df[df['rad']>0.2]
    df = df[df['rad']<1.0]

    df['total'] = df['double_ik'] + df['motion_alone']
    df_within_10ms= df[df['total']<10_000]
    print(f'{df_within_10ms.shape[0]/df.shape[0]*100}% solved under 10 millisecond')
    stat_all(df)

    range= (0,10)
    _ = plt.hist(df["single_ik"].values/1000, 40, label='single_ik', range=range, alpha=0.7)
    _ = plt.hist(df["double_ik"].values/1000, 40, label='double_ik', range=range, alpha=0.7)
    _ = plt.hist(df["total"].values/1000, 40, label='total', range=range, alpha=0.7)
    plt.xlabel('Run time (ms)')
    plt.ylabel('Number of samples')
    plt.legend()
    plt.show()