#!/usr/bin/env roseus

(load "package://spotkinovaeus/spotkinova-interface.l")
(load "package://image_view2/euslisp/image-capture-utils.l")
(load "package://gdrive_ros/euslisp/gdrive-ros-utils.l")
(load "package://jsk_robot_startup/euslisp/email-topic-client.l")

(ros::roseus-add-msgs "sensor_msgs")
(ros::roseus-add-msgs "jsk_recognition_msgs")

(defun main ()
  (let (target-coords)
    (while (ros::ok)
      ;; TODO update target-coords

      ;; look at target-coords
      (send *spotkinova* :look-at-target target-coords)
      (send *ri* :angle-vector (send *spotkinova* :angle-vector) 1000)
      (send *ri* :wait-interpolation)
      )
    )
  )
