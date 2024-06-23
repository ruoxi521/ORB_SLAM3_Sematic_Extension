# ORB_SLAM3_Sematic_Extension
A ORB_SLAM3-based semantic SLAM extensions repository

# Prerequisites

- PCL-1.12

# Usage

1. remove the `ROS` folder that raw

```
cd yourWorkspace_ws/src/ORB_SLAM3/Examples/ && rm -r ROS
```

2. git clone this repository in special path  

```
git clone https://github.com/ruoxi521/ORB_SLAM3_Sematic_Extension.git ./ROS
```

3. rebuild the `ROS` Workspace

```
cd ~/yourWorkspace_ws && catkin_make
```

4. `sourc`e your `ros` workspace and run the command in terminal

```
rosbag play --pause rgbd_dataset_freiburg3_walking_xyz.bag
```

```
roslaunch orb_slam3 tum1_rgbd_pointcloudmap.launch
```

```
roslaunch orbslam3_pointcloud_mapping tum1.launch
```

# Video

<iframe src="//player.bilibili.com/player.html?isOutside=true&aid=490233086&bvid=BV14N411i7ZN&cid=1246572349&p=1"&high_quality=1&danmaku=0&autoplay=0" scrolling="no" border="0" frameborder="no" framespacing="0" allowfullscreen="true" sandbox="allow-top-navigation allow-same-origin allow-forms allow-scripts" width="100%" height="600"></iframe>
