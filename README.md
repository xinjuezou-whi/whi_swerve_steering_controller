# whi_swerve_steering_controller
Swerve steer controller under ROS 2

## Dependency

```
sudo apt install ros-foxy-ros2-control
sudo apt install ros-foxy-ros2-controlls
sudo apt install ros-foxy-controller-interface
```

## Verify

Check the hardware interface:
```
ros2 control list_hardware_interfaces 
```

<img width="1170" height="700" alt="image" src="https://github.com/user-attachments/assets/831363a3-31c4-4bb8-b3d3-2384c457e9e3" />


Check the status:
```
ros2 control list_controllers
```

<img width="1170" height="204" alt="image" src="https://github.com/user-attachments/assets/c7e902b0-bc7d-4ae4-8e30-68c422781809" />
