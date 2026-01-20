# whi_swerve_steering_controller
Swerve steer controller under ROS 2

4 steers with a central pivot
![swerve](https://github.com/user-attachments/assets/a93bec30-9be7-4a34-87e6-8ed58a059426)

6 steers with a central pivot
![6](https://github.com/user-attachments/assets/87d0a8f2-9b70-4c0a-8972-d6f201f00ba2)

## Dependency

```
sudo apt install ros-$ROS_DISTRO-ros2-control
sudo apt install ros-$ROS_DISTRO-ros2-controlls
sudo apt install ros-$ROS_DISTRO-controller-interface
```

## Verify

Check the hardware interface:
```
ros2 control list_hardware_interfaces 
```

<img width="1461" height="1149" alt="屏幕截图 2026-01-20 171227" src="https://github.com/user-attachments/assets/b6e256b1-6e59-473a-a19c-0a387d8d3866" />

Check the status:
```
ros2 control list_controllers
```

<img width="545" height="103" alt="image" src="https://github.com/user-attachments/assets/a4ea891c-c244-4938-86da-b7c4bf2a7127" />

