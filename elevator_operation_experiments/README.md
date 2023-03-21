# elevator experments

## Setup

### Fetch

First, create workspace with []().

```bash
roslaunch jsk_fetch_startup fetch.launch launch_teleop:=false
```

```bash
roslaunch jsk_fetch_startup fetch_bringup.launch hostname:=$(hostname) launch_google_chat:=false launch_gdrive_server:=false fetch_lifelog:=false fetch_tweet:=false sound_device:=plughw:2,0 use_speech_recognition:=false launch_move_base:=false launch_app_scheduler:=false
```

```bash
roslaunch jsk_fetch_startup fetch_navigation.launch
```

And connect ENV III module to Fetch via USB and

```bash
roslaunch elevator_operation elevator_state_publisher.launch device_type:=enviii device_name:=/dev/ttyUSB0 device_baud:=57600 robot_type:=fetch
```

```bash
roslaunch elevator_operation elevator_operation.launch input_topic_points:=/l515_head/depth_registered/quater/points switchbot_token_yaml:=<yaml>
```

### Turtlebot

First, create workspace with [this forked robot-programming](https://github.com/sktometometo/robot-programming)

And add this package and `elevator_operation` package to it.

And launch basic launches.

```bash
roslaunch dxl_armed_turtlebot dxl_armed_turtlebot_bringup.launch
```

And connect ENV III module to Turtlebot via USB and

```bash
roslaunch elevator_operation elevator_state_publisher.launch device_type:=enviii device_name:=/dev/ttyUSB0 device_baud:=57600 robot_type:=turtlebot
```

```bash
roslaunch elevator_operation elevator_operation.launch input_topic_points:=/camera/depth_registered/points launch_switchbot_client:=true switchbot_token_yaml:=/home/sktometometo/switchbot_token.yaml robot_type:=turtlebot use_elevator_movement_status:=false
```
