
# 1. Installation
## 1.1 Required Dependences: 
```
sudo apt install ros-humble-joint-state-publisher ros-humble-robot-state-publisher ros-humble-cartographer ros-humble-cartographer-ros ros-humble-gazebo-plugins ros-humble-teleop-twist-keyboard  ros-humble-teleop-twist-joy ros-humble-xacro ros-humble-nav2* ros-humble-urdf 

```
```
cd ~/your workscpace
colcon build
```

## 1.2 Clone this repo 
Make sure you clone the repo in your robot and your remote PC 
```
git clone -b ros2-humble https://github.com/avirupghosh1/frost_hack.git
```
```
cd ~/your workscpace
colcon build
```
# 2. Setup

- Run bringup.launch.py to only spawn the robot
- Run autobringup.launch.py to spawn the robot with navigation and slam/localization
- Launch the files with use_sim_time:=False when working on real robot

### 2.1 Launching the robot

```
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=True exploration:=True
```
- exploration:=False for passed a saved map to navigation
### 2.2 Launching the initial_pose estimator node(Aligner)

```
ros2 run icp_localization icp_localization
```

### 2.3 Launching the Vins-fusion(MONO) feature extractor and pose estimator
before jumping directly make sure you have-
- OpenCV 4.2
- Ceres Solver 1.14.0
- Eigen 3.37
```
ros2 launch feature_tracker vins_feature_tracker.launch.py
```
```
ros2 launch vins_estimator euroc.launch.py
```


## TortosieBot is sourced, assembled, made & maintained by RigBetellLabs team 🧑🏻‍🤝‍🧑🏻 at<br>

RigBetel Labs LLP®, Charholi Bk., via. Loheagaon, Pune - 412105, MH, India 🇮🇳<br>
🌐 [RigBetelLabs.com](https://rigbetellabs.com) 📞 [+91-8432152998](https://wa.me/918432152998) 📨 getintouch.rbl@gmail.com , info@rigbetellabs.com <br>
[LinkedIn](http://linkedin.com/company/rigbetellabs/) | [Instagram](http://instagram.com/rigbetellabs/) | [Facebook](http://facebook.com/rigbetellabs) | [Twitter](http://twitter.com/rigbetellabs) | [YouTube](https://www.youtube.com/channel/UCfIX89y8OvDIbEFZAAciHEA) | [Discord Community](https://discord.gg/qDuCSMTjvN)

