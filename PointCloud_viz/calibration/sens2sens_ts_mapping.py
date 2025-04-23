import numpy as np
import os
import argparse

def get_timestamp(path):
    # In case of using ground truth
    if (".csv" == path[-4:]):
        data = None
        with open(path, "r") as f:
            data = f.read().split("\n")
            data = data[1:-1] # Remove header and last row
        ts = np.array([(line.split(",")[1]) for line in data])
    elif (".txt" == path[-4:]):
        data = None
        with open(path, "r") as f:
            data = f.read().split("\n")[:-1]
        ts = np.array([line.split('.')[0] + line.split('.')[1].ljust(9, "0") for line in data])
    else:
        frame_list = sorted(os.listdir(path))
        f_idx = frame_list[0].find(".") # Find where the format start
        ts = np.array([(ts[:f_idx]) for ts in frame_list])
    
    return ts

def main():
    parser = argparse.ArgumentParser(
        prog="sens2sens_ts_mapping.py",
        description="For synchronizing sensors data frame based on timestamp"
    )

    parser.add_argument(
        "-d", "--datadir",
        type=str,
        help="Path to data directory."
    )


    parser.add_argument(
        "-r", "--ref",
        type=str,
        help="reference folder name."
    )

    parser.add_argument(
        "-s", "--sync",
        type=str,
        help="sync folder name."
    )

    parser.add_argument(
        "-o", "--output",
        type=str,
        help="output folder name"
    )

    parser.add_argument(
        "--max_diff",
        type=float,
        help="Max difference in seconds"
    )

    parser.add_argument(
        "--offset",
        type=float,
        default=0.0,
        help="Offset timestamp from gt to sensor"
    )

    args = parser.parse_args()

    ref_ts = get_timestamp(os.path.join(args.datadir, args.ref))
    ref_fl_ts = ref_ts.astype(np.float64)
    sync_ts = get_timestamp(os.path.join(args.datadir, args.sync))
    sync_fl_ts = sync_ts.astype(np.float64)
    sync_fl_ts += args.offset * 1e9
    

    max_diff = args.max_diff * 1e9 # max difference 50ms

    # # Find starting point
    # check_start = np.argmin(np.abs(ref_fl_ts[0] - sync_fl_ts))
    # sync_fl_ts = sync_fl_ts[check_start:]
    # sync_ts = sync_ts[check_start:]
    # print(check_start)

    # check_start = np.argmin(np.abs(sync_fl_ts[0] - ref_fl_ts))
    # ref_fl_ts = ref_fl_ts[check_start:]
    # ref_ts = ref_ts[check_start:]
    # print(check_start)

    """
    Note: the np.tile will repeat the ref_fl_ts into 2D array
          with shape (ref_fl_ts.shape[0], sync_fl_ts.shape[0])
          i.e. [[ref1, ref2, ref3],
                [ref1, ref2, ref3],
                [ref1, ref2, ref3]]
        
          The time_diff then hold the difference between sync and ref pair by pair
    """
    ref_ts_tile = np.tile(ref_fl_ts, (sync_fl_ts.shape[0],1)).transpose()
    time_diff = np.abs(np.subtract(sync_fl_ts, ref_ts_tile))
    # np.savetxt("time_diff.txt", sync_fl_ts - ref_fl_ts[0])
    
    # Get ref frame that has smallest difference to each sync frame
    min_diff_ref2sync = np.argmin(time_diff, axis=0)
    # print(min_diff_ref2sync)

    # Get sync frame that has smallest difference to each ref frame
    min_diff_sync2ref = np.argmin(time_diff, axis=1)

    filename = "_mapping.txt"
    if (".txt" == args.ref[-4:]):
        ref_sensor = "tiradar"
    else:    
        ref_sensor = args.ref[args.ref.rfind("/")+1:]
    
    if (".csv" == args.sync[-4:]):
        sync_sensor = "gt"
    else:
        sync_sensor = args.sync[args.sync.rfind("/")+1:]
    filename = ref_sensor + "2" + sync_sensor + filename

    drop_frame_count = 0
    with open(os.path.join(args.datadir, args.output, filename), "w") as f:
        for i, ref in enumerate(ref_ts):
            # Check the cross match between ref and sync frame
            nearest_sync_ts = min_diff_sync2ref[i]
            # print(i, min_diff_ref2sync[nearest_sync_ts])
            if (i == min_diff_ref2sync[nearest_sync_ts] and 
                ref_fl_ts[i] - sync_fl_ts[nearest_sync_ts] <= max_diff):
                f.write(ref_ts[i] + " " + sync_ts[nearest_sync_ts] + "\n")
            else:
                f.write(ref_ts[i] + " " + "nan\n")
                drop_frame_count += 1

    print(drop_frame_count)

if __name__ == "__main__":
    main()
