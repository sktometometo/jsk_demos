#!/usr/bin/env roseus

(ros::roseus-add-msgs "elevator_operation")
(ros::roseus-add-msgs "std_msgs")

(load "package://elevator_operation/euslisp/utils.l")


(defun main ()
  (ros::roseus "demo")

  (setq *floor-initial* (ros::get-param "~floor_initial" "eng2/7f"))
  (setq *floor-target* (ros::get-param "~floor_pickup" "eng2/1f"))

  (ros::ros-info "moving to floor-pickup ~A" *floor-target*)
  (move-elevator *floor-target*)

  (exit)
  )


(main)
