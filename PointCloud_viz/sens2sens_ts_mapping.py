import numpy as np
import os
import argparse

def get_timestamp(path):
    # In case of using ground truth
    if (".csv" in path):
        data = None
        with open(path, "r") as f:
            data = f.read().split("\n")
            data = data[1:-1] # Remove header and last row
        ts = np.array([float(line.split(",")[1]) for line in data])
    else:
        frame_list = sorted(os.listdir(path))
        f_idx = frame_list[0].find(".") # Find where the format start
        ts = np.array([float(ts[:f_idx]) for ts in frame_list])
    
    return ts

def main():
    parser = argparse.ArgumentParser(
        prog="sens2sens_ts_mapping.py",
        description="For synchronizing sensors data frame based on timestamp"
    )

    parser.add_argument(
        "-r", "--ref_datadir",
        type=str,
        help="Path to reference data dir. Should have lower FPS."
    )

    parser.add_argument(
        "-s", "--synced_datadir",
        type=str,
        help="Path to synced data dir. Should have higher FPS."
    )

    parser.add_argument(
        "-o", "--output_dir",
        type=str,
        help="Path to output file"
    )

    parser.add_argument(
        "--max_diff",
        type=float,
        help="Max difference in seconds"
    )

    args = parser.parse_args()

    ref_ts = get_timestamp(args.ref_datadir)
    synced_ts = get_timestamp(args.synced_datadir)

    # print(ref_ts[:10])
    # print(synced_ts[:10])
    # print(ref_ts[:10] - synced_ts[:10])

    ref_idx, synced_idx = 0, 0
    max_diff = args.max_diff * 1e9 # max difference 50ms

    # Find starting point
    # check_start = np.argmin(np.abs(np.subtract(ref_ts[0], synced_ts)))
    # synced_ts = synced_ts[check_start:]

    # # print(ref_ts.shape)
    # ref_ts_tile = np.tile(ref_ts, (synced_ts.shape[0],1)).transpose()
    # # print(ref_ts_tile.shape)
    # # print(synced_ts.shape)
    # time_diff = np.abs(np.subtract(synced_ts, ref_ts_tile))
    # print(time_diff)
    # # print(time_diff.shape)
    # min_diff_idx_ver = np.argmin(time_diff, axis=0)
    # print(min_diff_idx_ver)
    # min_diff_idx_hor = np.argmin(time_diff, axis=1)

    filename = "_mapping.txt"
    ref_sensor = args.ref_datadir[args.ref_datadir.rfind("/")+1:]
    if (".csv" in args.synced_datadir):
        synced_sensor = "gt"
    else:
        synced_sensor = args.synced_datadir[args.ref_datadir.rfind("/")+1:]
    filename = ref_sensor + "2" + synced_sensor + filename
    with open(os.path.join(args.output_dir, filename), "w") as f:
        i, j = 0, 0
        while (i < ref_ts.shape[0] and j < synced_ts.shape[0]):
            curr_min = np.abs(ref_ts[i] - synced_ts[j])
            offset = 1
            while (j + offset < synced_ts.shape[0]):
                curr_diff = np.abs(ref_ts[i] - synced_ts[j+offset])
                print("{:10.0f}".format(ref_ts[i]), " ", 
                      "{:10.0f}".format(synced_ts[j+offset]), " ",
                      curr_diff)
                if (curr_diff < curr_min):
                    offset += 1
                    curr_min = curr_diff
                else:
                    j += offset - 1
                    break
            if (curr_min < max_diff):
                f.write("{:10.0f}".format(ref_ts[i]) + "," + \
                        "{:10.0f}".format(synced_ts[j]) + "\n")
            # print(curr_min)
            i += 1
            j += 1

if __name__ == "__main__":
    main()
