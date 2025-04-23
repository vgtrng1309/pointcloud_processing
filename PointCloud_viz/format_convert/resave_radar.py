import numpy as np
import os
import os.path as osp
import argparse
import time
import json
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
import shutil

DRAE_SHAPE = (16, 256, 128, 32)
process_4d = False
process_pcl = True

radar_path_prefix = "radar"

def main():
    parser = argparse.ArgumentParser(
        prog="resave_radar_mat.py",
        description="resave radar mat to float16 and change radar data frame name"
    )

    parser.add_argument(
        "-d", "--dir",
        type=str,
        help="Path to radar folder",
    )

    parser.add_argument(
        "-b", "--batch",
        action="store_true"
    )

    # parser.add_argument(
    #     "-m", "--mode",
    #     type=str,
    #     default="all",
    #     help="Select to save 4d/pcl/all"
    # )

    args = parser.parse_args()
    if (args.batch):
        seqs = sorted(os.listdir(args.dir))
    else:
        seqs = ["None"]

    for seq in seqs:
        print(seq)
        if (seq != "None"):
            dir = osp.join(args.dir, seq, radar_path_prefix)
        else:
            dir = osp.join(args.dir, radar_path_prefix)
        # if (not osp.exists(osp.join(dir, "remat"))):
        #     os.makedirs(osp.join(dir, "remat"))
        
        if (not osp.exists(osp.join(dir, "radarpcl"))):
            os.makedirs(osp.join(dir, "radarpcl"))
        # elif (len(os.listdir(osp.join(dir, "radarpcl"))) != 0):
        #     continue

        # mat_files = sorted(os.listdir(osp.join(dir, "mat")))
        if (not osp.exists(osp.join(dir, "pcl"))):
            continue
        pcl_files = sorted(os.listdir(osp.join(dir, "pcl")))

        # Read  timestamps
        ts = None
        with open(osp.join(dir,"timestamps.txt"), "r") as f:
            data = f.read().split("\n")
            # first_ts = data[0].split('.')[0] + ".000000000"
            # data = [first_ts] + data
            ts = np.array([line.split('.')[0] + line.split('.')[1].ljust(9, "0") for line in data])

        # Resave mat file
        # if (mode == "all" or mode == "4d"):
        #     for mat_file in mat_files:
        #         data = None
        #         idx = int(mat_file[mat_file.rfind("_")+1:mat_file.rfind(".")]) - 1
        #         print(mat_file, idx, ts[idx])

        #         if (not osp.exists(osp.join(dir, "remat", str(ts[idx])+".npy"))):
        #             data = np.fromfile(osp.join(dir, "mat", mat_file), np.float32)
        #             data = data.reshape(DRAE_SHAPE, order="F")
        #             # print(data[0][0])
        #             # print(data.astype(np.float16)[0][0])
        #             # break
        #             # print(idx)
        #             np.save(osp.join(dir, "remat", str(ts[idx])+".npy"), 
        #                 data.astype(np.float16))
        #     for t in ts:
        #         if (not osp.exists(osp.join(dir, cd"remat", str(t)+".npy"))):
        #             print(t)

        # Resave pcl file
        # if (args.mode == "all" or args.mode == "pcl"):
        format = ".csv" if pcl_files[0][-3:] == "csv" else ".xyzdi"
        for pcl in pcl_files:
            idx = int(pcl.replace("radarpcl_","").replace(format, "")) - 1
            # print(idx)
            if (not osp.exists(osp.join(dir, "radarpcl", str(ts[idx])+format))):
                shutil.copyfile(osp.join(dir, "pcl", pcl), 
                                osp.join(dir, "radarpcl", str(ts[idx])+format))


if __name__ == "__main__":
    main()


