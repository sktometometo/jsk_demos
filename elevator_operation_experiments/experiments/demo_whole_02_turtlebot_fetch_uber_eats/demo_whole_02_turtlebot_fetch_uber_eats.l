#!/usr/bin/env roseus

(ros::roseus-add-msgs "elevator_operation")
(ros::roseus-add-msgs "std_msgs")

(load "package://dxl_armed_turtlebot/euslisp/dxl-armed-turtlebot-interface.l")
(load "package://elevator_operation/euslisp/utils.l")


(setq *button-pressed* nil)
(defun callback-button-pressed (msg)
  (setq *button-pressed* (send msg :data))
  )

(defun wait-until-button-pressed ()
  (ros::rate 2)
  (while (ros::ok)
    (ros::sleep)
    (ros::spin-once)
    (ros::ros-info "waiting for button pressed: ~A" *button-pressed*)
    (if *button-pressed*
      (return))
    )
  t)


(defun main ()
  (ros::roseus "demo_whole_02_turtlebot_fetch_uber_eats")
  (dxl-armed-turtlebot-init)
  (ros::subscribe "/pressed" std_msgs::Bool #'callback-button-pressed)

  (setq *floor-initial* (ros::get-param "~floor_initial" "eng2/7f"))
  (setq *pose-initial* (ros::get-param "~pose_initial"))
  (setq *floor-pickup* (ros::get-param "~floor_pickup" "eng2/1f"))
  (setq *pose-pickup* (ros::get-param "~pose_pickup"))

  (ros::spin-once)

  (ros::ros-info "moving to initial pose")
  (send-move-base-goal 0 0 0 0 0 0 1 "base_link" :wait nil)
  (unix::sleep 1)
  (send-move-base-goal
    (elt *pose-initial* 0)
    (elt *pose-initial* 1)
    (elt *pose-initial* 2)
    (elt *pose-initial* 3)
    (elt *pose-initial* 4)
    (elt *pose-initial* 5)
    (elt *pose-initial* 6)
    (elt *pose-initial* 7)
    :wait t
    :timeout 0
    )

  (send *ri* :speak-jp "ボタンを押して取りに行きます")

  (wait-until-button-pressed)
  (send *ri* :speak-jp "食べ物を取りに行ってきます")

  (ros::ros-info "moving to floor-pickup ~A" *floor-pickup*)
  (move-elevator *floor-pickup*)

  (ros::ros-info "moving to pickup pose" *pose-pickup*)
  (send-move-base-goal 0 0 0 0 0 0 1 "base_link" :wait nil)
  (unix::sleep 1)
  (send-move-base-goal
    (elt *pose-pickup* 0)
    (elt *pose-pickup* 1)
    (elt *pose-pickup* 2)
    (elt *pose-pickup* 3)
    (elt *pose-pickup* 4)
    (elt *pose-pickup* 5)
    (elt *pose-pickup* 6)
    (elt *pose-pickup* 7)
    :wait t
    :timeout 0
    )

  (send *ri* :speak-jp "食べ物をかごにいれてボタンを押して下さい")
  (wait-until-button-pressed)

  (ros::ros-info "moving to floor-initial ~A" *floor-initial*)
  (move-elevator *floor-initial*)

  (send-move-base-goal 0 0 0 0 0 0 1 "base_link" :wait nil)
  (unix::sleep 1)
  (send-move-base-goal
    (elt *pose-initial* 0)
    (elt *pose-initial* 1)
    (elt *pose-initial* 2)
    (elt *pose-initial* 3)
    (elt *pose-initial* 4)
    (elt *pose-initial* 5)
    (elt *pose-initial* 6)
    (elt *pose-initial* 7)
    :wait t
    :timeout 0
    )

  (send *ri* :speak-jp "食べ物を取ってきました")

  (exit)
  )


(main)
