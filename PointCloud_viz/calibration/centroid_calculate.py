import numpy as np
import os
import argparse

def main():
    parser = argparse.ArgumentParser(
        prog="centroid_calculate.py",
        description="For calculating centroid of reflector on lidar"
    )

    parser.add_argument(
        "-f", "--file",
        type=str,
        help="Path to picked list"
    )

    parser.add_argument(
        "-n", "--num",
        type=int,
        help="Number of subjects"
    )

    args = parser.parse_args()

    with open(args.file, "r") as f:
        data = f.read().split("\n")[:-1]
        n_points = len(data)
        points = np.zeros((n_points, 3))
        print(points.shape)
        for i, point in enumerate(data):
            ps = point.split(",")
            for j in range(3):
                points[i][j] = np.float64(ps[j])
        # points = np.array(np.array(point.split(",")) for point in data)
        for i in range(args.num):
            print(np.mean(points[i*args.num:(i+1)*args.num], axis=0))


if __name__ == "__main__":
    main()
