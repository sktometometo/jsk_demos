#!/usr/bin/env roseus

(ros::roseus-add-srvs "elevator_operation")
(ros::roseus-add-msgs "move_base_msgs")
(ros::roseus-add-msgs "std_msgs")

(load "package://fetcheus/fetch-interface.l")
(load "package://elevator_operation/euslisp/utils.l")

(setq *elevator-movement* "")
(setq *elevator-movement-change-flag* nil)
(defun callback-elevator-movement (msg)
  (if (not (equal *elevator-movement* (send msg :data)))
    (setq *elevator-movement-change-flag* t)
    )
  (setq *elevator-movement* (send msg :data))
  )

(setq *current-floor* nil)
(defun callback-current-floor (msg)
  (setq *current-floor* (send msg :data))
  )

(defun main()
  (fetch-init)
  (ros::subscribe "/elevator_state_publisher/elevator_movement" std_msgs::String #'callback-elevator-movement)
  (ros::subscribe "/elevator_state_publisher/current_floor" std_msgs::Int16 #'callback-current-floor)

  ;;(setq *pose-inside-elevator* (ros::get-param "~pose_inside_elevator"))

  ;;(add-collision-objects)

  ;;(move-arm-pre-pose)

  ;;(call-look-at-target "~look_at" *door-frame-id*)

  (ros::rate 10)
  (while (ros::ok)
    (ros::sleep)
    (ros::spin-once)
    (ros::ros-info "em: ~A cf: ~A" *elevator-movement* *current-floor*)
    (if *elevator-movement-change-flag*
      (progn
        (cond
          ((equal *elevator-movement* "halt")
             (send *ri* :speak-jp (format nil "~A階です" *current-floor*))
             (ros::ros-info "~A階です" *current-floor*)
             )
          ((equal *elevator-movement* "down_accel")
             (send *ri* :speak-jp "下へ参ります")
             (ros::ros-info "下へ参ります")
             )
          ((equal *elevator-movement* "up_accel")
             (send *ri* :speak-jp "上へ参ります")
             (ros::ros-info "上へ参ります")
             )
          )
        (setq *elevator-movement-change-flag* nil)
        )
      )
    )

  (send *ri* :speak-jp "到着しました")
  (ros::ros-info "Finished.")
  (exit)
  )

(main)
