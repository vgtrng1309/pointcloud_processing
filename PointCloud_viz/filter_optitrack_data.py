import numpy as np
import os
import argparse
import datetime

class CSV_Reader(object):
    def __init__(self, file_path):
        self.csv_lines = None
        self.file_path = file_path
        with open(self.file_path, "r") as f:
            self.csv_lines = f.read().split("\n")[:-1]
        
        self.start_ts = self.get_timestamp() # in second
        print(self.start_ts)

        self.type_list = ["Bone Marker", "Rigid Body"]
        
        self.

        self.get_data()

    def get_timestamp(self):
        line = self.csv_lines[0].split(",") # First line has timestamp
        ts_idx = line.index("Capture Start Time") + 1
        ts_line = line[ts_idx]

        ts_date = ts_line[:19] # Get date with format %Y-%m-%d %H.%M.%S
        element = datetime.datetime.strptime(ts_date,"%Y-%m-%d %H.%M.%S")
        timestamp = np.float(datetime.datetime.timestamp(element))
        ts_ms   = np.float(ts_line[20:-4])

        timestamp += ts_ms * 1e-3

        return timestamp

    def get_data(self):
        type_line = self.csv_lines[2].split(",") # Bone - Bone Marker - Rigid Body - ...
        name_line = self.csv_lines[3].split(",") # Skeleton001: --....
        id_line   = self.csv_lines[4].split(",") # 1 - 2 - 3 - ....
        header    = self.csv_lines[6].split(",") # Frame - Time (Seconds) - X - Y - Z - ...
        data      = self.csv_lines[7:]

        # Get selected type index
        type_idx  = [0,1] + [i for i, x in enumerate(type_line) if x in self.type_list]
        # print(type_idx)
        
        # 
        header_out = ",".join([n+'+'+h for n, h in zip(np.asarray(name_line)[type_idx], 
                                                       np.asarray(header)[type_idx])])
        with open(self.file_path[:-4] + "_stage_01.csv", "w") as f:
            f.write(header_out+"\n")
            for data_line in data:
                data_split = data_line.split(",")
                
                # Adjust timestamp
                time = (self.start_ts + np.float(data_split[1]))*1e9
                time = "{:.0f}".format(time)
                data_split[1] = time

                data_out = ",".join(np.asarray(data_split)[type_idx])
                f.write(data_out+"\n")
        
        with open(self.file_path[:-4] + "_stage_02.csv", "w") as f:

        

def main():
    parser = argparse.ArgumentParser(
        prog="filter_optitrack_data.py",
        description="For calculating centroid of reflector on lidar"
    )

    parser.add_argument(
        "-f", "--file",
        type=str,
        help="Path to csv file"
    )

    args = parser.parse_args()

    csv_reader = CSV_Reader(args.file)

if __name__ == "__main__":
    main()