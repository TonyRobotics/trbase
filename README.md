## Moving robots of Tonyrobotics company

### trd_driver

trd_driver package provides a usb-serial(TTL) motor driver for Abel05/Abel05x, Abel10, Abel30 and Xiaobai moving robots of [Tonyrobotics](http://www.tonyrobotics.com/).

Put the package in your ROS workspace and build:

```
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/src
$ git clone https://github.com/tonyrobotics/trbase.git
```

```
$ cd ~/catkin_ws
$ catkin_make
```

Then create udev rules of TRD motor driver.

```
$ cd ~/catkin_ws/src/trbase/trd_driver/udev
$ sudo sh create_udev_rules.sh
```

Connect TRD board to USB port and test the code:

```
$ roscore
```

```
$ rosrun trd_driver trd_driver.py
```

### Abel05/Abel05x 

* For keyboard control:

```
$ roslaunch abel05_navigation controller.launch
```

Open a new ternimal,

```
$ rosrun trd_driver keyboard_teleop.py
```

* For mapping:

```
$ roslaunch abel05_navigation mapping.launch
```

* For navigation:

```
$ roslaunch abel05_navigation navigation.launch
```

### xiaobai

* For keyboard control:

```
$ roslaunch xiaobai_navigation controller.launch
```

Open a new ternimal,

```
$ rosrun trd_driver keyboard_teleop.py
```

* For mapping:

```
$ roslaunch xiaobai_navigation mapping.launch
```

* For navigation:

```
$ roslaunch xiaobai_navigation navigation.launch
```

### usb camera

To open the usb camera on the robot,:

```
$ roscore
```

Then open a new terminal:

```
$ rosrun usb_cam usb_cam_node _video_device:='/dev/video0'
```

On your PC, start image view: 

```
$ rqt_image_view
```

