# Enhancing LiDAR Odometry and Mapping with Pointcloud-based Loop Closures (16833 Project)
## Usage
### Set up
```bash
$ git clone https://github.com/bobbyshashin/16833-project.git
$ cd 16833-project/aloam_ws
$ catkin_make
$ source devel/setup.bash
```
### Generate KITTI ros bags
Skip this if you already have kitti rosbags.
1. Download [KITTI Odometry dataset](http://www.cvlibs.net/datasets/kitti/eval_odometry.php), we do not need grayscale and color, and orgnize it as following:
```bash
├── poses
│   ├── 00.txt
│   ├── ......
│   └── 10.txt
└── sequences
│   ├── 00
│   │   ├── velodyne
│   │   │   ├── 000000.bin
│   │   │   ├── ......
│   │   │   └── xxxxxx.bin
│   │   ├── calib.txt
│   │   └── times.txt
│   ├── ...
│   └── 21
```
2. Modify line 5,6,7 in [kitti_help.launch](aloam_ws/src/A-LOAM/launch/kitti_helper.launch).
3. Run kitti_helper to generate rosbags (after catkin_make, source)
```bash
roslaunch aloam_velodyne kitti_helper.launch
```
This will save kitti rosbags to your local.
### Run the algorithm
1. Launch LOAM and pose graph optimization node
```bash
$ roslaunch aloam_velodyne aloam_velodyne_HDL_64.launch
```
2. Launch loop closure detection node
```bash
$ cd src/A-LOAM/scripts/
$ python LCD.py
```
3. Play KITTI rosbag.
```bash
$ rosbag play PATH/TO/KITTI_ROSBAG/SEQUENCE_NUMBER.bag
```

## Acknowledgements
Thanks for [Qin's LOAM implementation](https://github.com/HKUST-Aerial-Robotics/A-LOAM).

