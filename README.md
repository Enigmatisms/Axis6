# Axis6

---

6-axis robot simulation in rviz and Gazebo, control implemented with ROS.

- Hand-deduced D-H forward-inverse kinematics implemented in Eigen
- Simulated and visualized in RViz and Gazebo
- ROS implementation (melodic), with key control.

---

### Usage

```shell
user@user:~/Axis6$ sudo chmod +x ./make.sh	# 编译脚本权限 （Executable authorization for compilation script）
user@user:~/Axis6$ ./make.sh 8				# 编译 (compile with 8 threads)
user@user:~/Axis6$ source devel/setup.bash	# source
user@user:~/Axis6$ roslaunch axis6 axis6_gazebo.launch &		# 后台运行gazebo以及模型生成 (run gazebo and robot generation in the background)
user@user:~/Axis6$ roslauch axis6 gazebo_sim.launch			# 控制六轴机器人 (control node)
```



![](asset/gazebo.png)

![](asset/rviz.png)
