# pointcloud_processing

## Refining OptiTrack MoCap csv file
python filter_optitrack_data.py -f <path/to/optitrack_file.csv>

## Visualizing refined csv point cloud file
python visual_pointcloud_csv.py -f <path/to/refined_file.csv> -s <start_time> -m <frame_by_frame_mode>

## Visualizing pointcloud sequence in pcd/xyzdi format
## NOTE: xyzdi is a self-made text based format to store 5D radar data
##       including 3D position (x, y, z), Doppler velocity (d), Intensity (i)
python visual_pointcloud.py -d <path/to/pointcloud/directory> -s <start_time> -m <frame_by_frame_mode> -t <point/type/pcd_or_xyzdi>