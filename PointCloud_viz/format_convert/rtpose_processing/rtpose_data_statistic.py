import json
import os
import os.path as osp
import argparse
import numpy as np

data_path_tail = "radar/npy_DZYX_real"

def main():
    parser = argparse.ArgumentParser(
        prog="rtpose_data_statistic.py",
        description=""
    )

    parser.add_argument(
        "-d", "--dir",
        type=str,
        help="Path to folder that keep all data"
    )

    args = parser.parse_args()

    seq_list = sorted(os.listdir(args.dir))
    min_val, max_val = np.inf, -np.inf
    for seq in seq_list:
        # Broken data
        if (seq == "civit_easy_march_02"):
            continue
        print(seq)
        frame_list = sorted(os.listdir(osp.join(args.dir, seq, data_path_tail)))
        for frame in frame_list:
            data = None
            with open(osp.join(args.dir, seq, data_path_tail, frame), "rb") as f:
                data = np.load(f)
            min_data, max_data = np.min(data), np.max(data)
            if (min_val > min_data):
                min_val = min_data
            if (max_val < max_data):
                max_val = max_data
            print(min_val, max_val)

    print(min_val, max_val)
if __name__ == "__main__":
    main()