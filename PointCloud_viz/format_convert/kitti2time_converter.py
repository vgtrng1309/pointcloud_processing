import numpy as np
import os
import os.path as osp
import argparse
import shutil

npy_DZYX_path_tail = "radar/npy_DZYX_real"
mapping_file_tail = "calib/radarpcl2gt_mapping.txt"

def main():
    parser = argparse.ArgumentParser(
        prog="kitti2time.py",
        description="For converting radar frame between kitti format (index) and timestamp"
    )

    parser.add_argument(
        "-d", "--dir",
        type=str,
        help="Path to data dir"
    )

    parser.add_argument(
        "-m", "--mode",
        type=str,
        default="k2t",
        help="Choose mode to convert kitti to timestamp (k2t), vice versa"
    )

    args = parser.parse_args()

    # File list
    rad_files = sorted(os.listdir(osp.join(args.dir, npy_DZYX_path_tail)))
    
    if (args.mode == "k2t"):
        # Read  timestamps
        ts_map = np.loadtxt(osp.join(args.dir, mapping_file_tail), dtype=str)[:,0]
        for rad_file in rad_files:
            if (osp.exists(osp.join(args.dir, npy_DZYX_path_tail, rad_file))):
                i = int(rad_file.replace(".npy",""))
                os.rename(osp.join(args.dir, npy_DZYX_path_tail, rad_file),
                          osp.join(args.dir, npy_DZYX_path_tail, ts_map[i]+".npy"))
    else:
        for i, rad_file in enumerate(rad_files):
            if (osp.exists(osp.join(args.dir, npy_DZYX_path_tail, rad_file))):
                os.rename(osp.join(args.dir, npy_DZYX_path_tail, rad_file),
                          osp.join(args.dir, npy_DZYX_path_tail, str(i).zfill(6)+".npy"))


if __name__ == "__main__":
    main()
