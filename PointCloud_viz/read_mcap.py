import sys
from mcap.reader import make_reader
from edgefirst.schemas.std_msgs import Header
from edgefirst.schemas.sensor_msgs import Image, PointCloud2
import argparse

# topics_list = {'/off_highway_premium_radar_sample_driver/locations': [0,0], 
#                '/ouster/points': [0,0],
#                '/ouster/imu': [0,0],
#                '/zed2i/zed_node/left/image_rect_gray': [0,0],
#                '/zed2i/zed_node/right/image_rect_gray': [0,0],
#                '/zed2i/zed_node/odom': [0,0],
#                '/zed2i/zed_node/imu/data': [0,0],
#                '/imu/data': [0,0],
#                '/gnss': [0,0]}

# bags_list = ['rosbag_2024_07_18-15-35-26_lidar_imu',
#              'rosbag_2024_07_18-15-48-58_lidar_imu',
#              'rosbag_2024_07_18-15-58-21_lidar_imu',
#              'rosbag_2024_07_18-16-04-59_lidar_imu',
#              'rosbag_2024_07_23-15-21-57_lidar_imu_zed_bosch',
#              'rosbag_2024_07_23-15-25-58_lidar_imu_zed_bosch',
#              'rosbag_2024_07_23-15-33-37_lidar_imu_zed_bosch',
#              'rosbag_2024_07_23-15-42-41_lidar_imu_zed_bosch']

parser = argparse.ArgumentParser(
    prog="read_mcap.py",
    description="For synchronizing sensors data frame based on timestamp"
)

parser.add_argument(
    "-i", "--input",
    type=str,
    help="Path to mcap file"
)

args = parser.parse_args()

with open(args.input, "rb") as f:
    reader = make_reader(f)
    statistic = reader.get_summary().statistics
    start = statistic.message_start_time * 1e-9 # convert nano second to second
    end = statistic.message_end_time * 1e-9 # convert nano second to second
    for schema, channel, message in reader.iter_messages():
        if ("zed2i/zed_node/left/image_rect" in channel.topic):
            print("ZED: ", Image.deserialize(message.data).header)
        elif ("off" in  channel.topic):
            print("BOSCH: ", PointCloud2.deserialize(message.data).header)

        # topics_list[channel.topic][1] += len(message.data) * 1e-3 # convert bytes to kilobytes
        # if (channel.topic == "/zed2i/zed_node/left/image_rect_gray"):
        #     print(len(message.data))
        # topics_list[channel.topic][0] += 1

# for key, value in topics_list.items():
#     if (value[0] == 0):
#         continue
#     print("Topic: ", key)
#     print("\tRate (Hz): {:.3f}".format(value[0] / (end-start)))
#     print("\tRate (KB/s): {:.3f}".format(value[1] / (end-start)))
#     print("\tSize (KB/msg): {:3f}".format(value[1] / value[0]))
#     print()
