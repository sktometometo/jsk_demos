# Person Follow Demo

<TODO image>

This demo enables Spot to follow person.

## Prerequities

This demo requires packages below.

- [spot_ros]() with [this branch](https://github.com/sktometometo/spot_ros)
- [common_msgs]() with [this branch](https://github.com/sktometometo/common_msgs/tree/PR/add-panorama-info)
- [jsk_recognition]() with [this branch](https://github.com/tongtybj/jsk_recognition/tree/develop/spot_trtr)
- [jsk_spot_startup]() with [this branch](https://github.com/sktometometo/jsk_robot/tree/feature/spot/dualshock4), to support dualshock4
- [coral_usb_ros](https://github.com/knorth55/coral_usb_ros) based on an independent catkin workspace (python3)

## How to run

Before running this demo, please launch and prepair a controller.

- `roslaunch jsk_spot_startup jsk_spot_bringup.launch`
- `roslaunch jsk_spot_startup object_detection_and_tracking.launch`

And then, please run

```bash
roslaunch spot_person_follower demo.launch JOY_TOPIC:=/joy_dualshock4
```

After this, you can start following behavior by pressing L2 button of the controller.
If you want to stop the behavior, please press the L2 button again.
