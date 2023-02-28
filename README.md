# kaist2bag

**This sourced code is modified from [kaist2bag](https://github.com/tsyxyz/kaist2bag) provided form [tsyxyz](https://github.com/tsyxyz), while we combine the left and right 3D LiDAR data under one topic and share the same timestamp.**

## Guide

1. Download desired database files from the [website](https://sites.google.com/view/complex-urban-dataset)

2. Extract `.gz.tar` files into their folders
```
find . -name 'urban28*.tar.gz' -execdir tar -xzvf '{}' \;
```

3. Clone and build this repository
```
cd src
git clone https://github.com/irapkaist/irp_sen_msgs.git
git clone https://github.com/ZikangYuan/kaist2bag.git
cd ..
catkin build
```

4. Edit the [config file](config/config.yaml) with path and desired topics


5. Create a rosbag file for each sensor type
```
source devel/setup.bash
roslaunch kaist2bag kaist2bag.launch
```

6. Merge all bags into a single one (if desired)
```
rosrun kaist2bag mergebag.py merged.bag <bag_file_1> ... <bag_file_8>
```

## Acknowledgments

Thanks for [kaist2bag](https://github.com/tsyxyz/kaist2bag).

