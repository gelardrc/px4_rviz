# px4_rviz

A package to add iris uav model to rviz and also plot uav's path.

# How to install

> git clone https://github.com/gelardrc/px4_rviz.git

Dependencies:

> git clone  https://github.com/PX4/PX4-Autopilot.git

# How to use

> rosrun px4_rviz px4_rviz.py <robot_id>

🚀️ If your UAV has no namespace associated, just run px4_rviz.py without an argument.

# Example

> roslaunch px4_rviz example.launch
*You should see rviz like this :*

# To do list

* [ ] Improve path representation ( working ).
* [ ] Change sys.argv ( robot_id ) to a rosparam.

