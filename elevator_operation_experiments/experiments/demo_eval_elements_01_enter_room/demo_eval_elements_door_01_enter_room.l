#!/usr/bin/env roseus

(ros::roseus-add-srvs "elevator_operation")
(ros::roseus-add-msgs "std_msgs")

(load "package://fetcheus/fetch-interface.l")
(load "package://elevator_operation/euslisp/utils.l")

(setq *door-is-open* nil)
(defun callback-door-is-open (msg)
  (setq *door-is-open* (send msg :data))
  )

(defun main()
  (fetch-init)
  (ros::subscribe "~door_is_open" std_msgs::Bool #'callback-door-is-open)

  (setq *door-frame-id* (ros::get-param "~door_frame_id" ))
  (setq *pose-outside-room* (ros::get-param "~pose_outside_room"))
  (setq *pose-inside-room* (ros::get-param "~pose_inside_room"))

  (send-move-base-goal
    (elt *pose-outside-room* 0)
    (elt *pose-outside-room* 1)
    (elt *pose-outside-room* 2)
    (elt *pose-outside-room* 3)
    (elt *pose-outside-room* 4)
    (elt *pose-outside-room* 5)
    (elt *pose-outside-room* 6)
    (elt *pose-outside-room* 7)
    :wait t
    )

  (if (not (call-look-at-target "~look_at" *door-frame-id*))
    (progn
        (ros::ros-error "look at failed")
        (exit 1)
        )
    (progn
      (ros::ros-info "look at success")
      )
    )

  (send *ri* :speak-jp "ごめんください")

  (ros::rate 1)
  (while (ros::ok)
    (ros::spin-once)
    (ros::sleep)
    (ros::ros-info "waiting for door to open: ~A" *door-is-open*)
    (if *door-is-open*
      (return))
    )

  (send *ri* :speak-jp "失礼します")
  (send-move-base-goal
    (elt *pose-inside-room* 0)
    (elt *pose-inside-room* 1)
    (elt *pose-inside-room* 2)
    (elt *pose-inside-room* 3)
    (elt *pose-inside-room* 4)
    (elt *pose-inside-room* 5)
    (elt *pose-inside-room* 6)
    (elt *pose-inside-room* 7)
    :wait t
    )

  (ros::ros-info "Finished.")
  )

(main)
