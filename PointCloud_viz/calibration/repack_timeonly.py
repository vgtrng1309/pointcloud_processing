"""Format MMWCAS-RF-EVM & MMWCAS-DSP-EVM kit Recordings.

    @author: AMOUSSOU Z. Kenneth
    @date: 13-08-2022
    Modified by Prashant Kumar Rai(@pkrason, prashant.rai@tuni.fi) to log correct timestamps for the recorded frames.
"""


from typing import Optional
from datetime import timedelta, datetime
import os
import glob
import argparse
import sys
import numpy as np
import struct

__VERSION__: str = "0.1"
__COPYRIGHT__: str = "Copyright (C) 2022, RWU-RADAR Project"


def getInfo(idx_file: str, start_time):
    """Get information about the recordings.

    The "*_idx.bin" files along the sample files gather usefule
    information aout the dataset.

    The structure of the "*_idx.bin" file is as follow:

    ---------------------------------------------------------------------------
         File header in *_idx.bin:
             struct Info
             {
                 uint32_t tag;
                 uint32_t version;
                 uint32_t flags;
                 uint32_t numIdx;       // number of frames
                 uint64_t dataFileSize; // total data size written into file
             };

         Index for every frame from each radar:
             struct BuffIdx
             {
                 uint16_t tag;
                 uint16_t version; /*same as Info.version*/
                 uint32_t flags;
                 uint16_t width;
                 uint16_t height;

                 /*
                  * For image data, this is pitch. For raw data, this is
                  * size in bytes per metadata plane
                  */
                 uint32_t pitchOrMetaSize[4];

                /*
                 * Total size in bytes of the data in the buffer
                 * (sum of all planes)
                */
                 uint32_t size;
                 uint64_t timestamp; // timestamp in ns
                 uint64_t offset;
             };

        Source: Example matlab script provided by Texas Instrument
    ---------------------------------------------------------------------------

    Arguemnt:
        idx_file: Path to an index file from any of the cascaded chip

    Return:
        Tuple containing respectively the number of valid frames recorded
        and the size of the data file
    """
    # Data type based on the structure of the file header
    dt = np.dtype([
        ("tag", np.uint32),
        ("version", np.uint32),
        ("flags", np.uint32),
        ("numIdx", np.uint32),
        ("size", np.uint64),
    ])
    header = np.fromfile(idx_file, dtype=dt, count=1)[0]

    dt = np.dtype([
        ("tag", np.uint16),
        ("version", np.uint16),
        ("flags", np.uint32),
        ("width", np.uint16),
        ("height", np.uint16),

        ("_meta0", np.uint32),
        ("_meta1", np.uint32),
        ("_meta2", np.uint32),
        ("_meta3", np.uint32),

        ("size", np.uint32),
        ("timestamp", np.uint64),
        ("offset", np.uint64),
    ])

    data = np.fromfile(idx_file, dtype=dt, count=-1, offset=24)
    timestamp_values = [log[-2] for log in data]
    diff_time = np.diff(np.array(timestamp_values))
    cumulative_times_microseconds = np.cumsum(diff_time)

    calculated_timestamps = [datetime.strptime(start_time, "%Y-%m-%d %H:%M:%S") + timedelta(microseconds=int(time)) for time in cumulative_times_microseconds]

    # Now calculated_timestamps contains datetime objects for each frame
    unix_timestamps = [datetime.strptime(start_time, "%Y-%m-%d %H:%M:%S").timestamp()]
    unix_timestamps += [timestamp.timestamp() for timestamp in calculated_timestamps]
    return header[3], header[4], unix_timestamps

def load(inputdir: str, device: str):
    """Load the recordings of the radar chip provided in argument.

    Arguments:
        inputdir: Input directory to read the recordings from
        device: Name of the device

    Return:
        Dictionary containing the data and index files
    """
    # Collection of the recordings data file
    # They all have the "*.bin" ending
    recordings: dict[str, list[str]] = {
        "data": glob.glob(f"{inputdir}{os.sep}{device}*data.bin"),
        "idx": glob.glob(f"{inputdir}{os.sep}{device}*idx.bin")
    }
    recordings["data"].sort()
    recordings["idx"].sort()

    if (len(recordings["data"]) == 0) or (recordings["idx"] == 0):
        print(f"[ERROR]: No file found for device '{device}'")
        return None
    elif len(recordings["data"]) != len(recordings["idx"]):
        print(
            f"[ERROR]: Missing {device} data or index file!\n"
            "Please check your recordings!"
            "\nYou must have the same number of "
            "'*data.bin' and '*.idx.bin' files."
        )
        return None
    return recordings


if __name__ == "__main__":
    # Output directory that would hold the formatted data per frame
    OUTPUT_DIR: str = "output"

    # Number of samples
    NS: int = 256

    # Number of chirps
    NC: int = 16

    parser = argparse.ArgumentParser(
        prog="repack.py",
        description="MMWAVECAS-RF-EVM board recordings post-processing routine. "
                    "Repack the recordings into MIMO frames"
    )
    parser.add_argument(
        "-v", "--version",
        help="Print software version and information about the dataset.",
        action="store_true"
    )
    parser.add_argument(
        "-ns", "--number-samples",
        help="Number of samples per chirp ",
        type=int,
        default=NS,
    )
    parser.add_argument(
        "-nc", "--number-chirps",
        help="Number of chirp loops per frame ",
        type=int,
        default=NC,
    )
    parser.add_argument(
        "-o", "--output-dir",
        help="Output directory for storing the mimo frames",
        type=str,
        default=OUTPUT_DIR,
    )
    parser.add_argument(
        "-i", "--input-dir",
        help="Input directory containing the recordings",
        type=str,
        default=None,
    )
    parser.add_argument(
        "-st", "--start-time",
        help="start time of the recording",
        type=str,
        default=None,
    )
    parser.add_argument(
    	"-m", "--mode",
    	help="Mode to use. Save all or only timestamps",
    	type=str,
    	default="all", # mode: all/ts
    )

    args = parser.parse_args()

    if args.version:
        print(f"mmwave-repack version {__VERSION__}, {__COPYRIGHT__}")
        sys.exit(0)

    if args.input_dir is None:
        print("[ERROR]: Missing input directory to read recordings")
        sys.exit(1)

    # The output directory will be created inside the data directory by default
    if (args.input_dir is not None) and (args.output_dir == OUTPUT_DIR):
        args.output_dir = os.path.join(args.input_dir, OUTPUT_DIR)

    if not os.path.isdir(args.output_dir):
        os.makedirs(args.output_dir, exist_ok=True)

    # Load devices recording file paths
    master = load(args.input_dir, "master")
    slave1 = load(args.input_dir, "slave1")
    slave2 = load(args.input_dir, "slave2")
    slave3 = load(args.input_dir, "slave3")

    assert master != None, "Error with master data files"
    assert slave1 != None, "Error with slave1 data files"
    assert slave2 != None, "Error with slave2 data files"
    assert slave3 != None, "Error with slave3 data files"

    # Integrity status of the recordings
    # Check if the number of files generated for each device is
    # identical
    status: bool = True

    status = status and (len(master["data"]) == len(slave1["data"]))
    status = status and (len(master["data"]) == len(slave2["data"]))
    status = status and (len(master["data"]) == len(slave3["data"]))

    if not status:
        print("[ERROR]: Missing recording for cascade MIMO configuration")
        sys.exit(1)

    size: int = len(master["data"])

    # Number of frames recorded
    nf: int = 0

    # Number of frames generated from the last recording batch
    previous_nf: int = 0

    timestamps = np.array([])

    for idx in range(size):
        # Master file
        mf: str = master["data"][idx]
        mf_idx: str = master["idx"][idx]

        # Slave data files
        sf1: str = slave1["data"][idx]
        sf2: str = slave2["data"][idx]
        sf3: str = slave3["data"][idx]

        nf, _, timelogs = getInfo(mf_idx, args.start_time)
        #print(getInfo(mf_idx))

        # Skip if the number of valid frame is 0
        if not nf:
            continue

        timestamps = np.append(timestamps, timelogs)

    print(f"[SUCCESS]: {previous_nf:04} frames written!")
    # Save all the timestamps in a single file
    timestamps.tofile(os.path.join(args.output_dir, "timestamps.txt"), "\n")

    print(f"[SUCCESS]: {previous_nf:04d} MIMO frames successfully generated!")
