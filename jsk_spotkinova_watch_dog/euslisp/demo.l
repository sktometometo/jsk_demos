#!/usr/bin/env roseus

(load "package://spotkinovaeus/spotkinova-interface.l")

(ros::roseus-add-msgs "geometry_msgs")

(defun look-at (target-coords)
  (send *spotkinova* :look-at-target target-coords)
  (send *ri* :angle-vector (send *spotkinova* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

(defun fold-arm ()
  (send *spotkinova* :kinova-rest-pose)
  (send *ri* :angle-vector (send *spotkinova* :angle-vector) 2000)
  (send *ri* :wait-interpolation)
  )

(defun get-person-pose ()
  (let (target-coords msg)
    (ros::spin-once)
    (setq msg (one-shot-subscribe "/spot_recognition_person_tracker/people_pose_array" geometry_msgs::PoseArray :timeout 1000))
    (if (and msg (> (length (send msg :poses)) 0))
        (progn
          (dolist (pose (send msg :poses))
            (setq target-coords (ros::tf-pose->coords pose))
            (if (> (elt (send target-coords :pos) 0) 100)
                (return-from get-person-pose target-coords)
                nil)
            ))
        nil)
    (return-from get-person-pose nil)
    ))

(defun main ()
  (spotkinova-init)
  (send *spotkinova* :reset-pose)
  (send *ri* :angle-vector (send *spotkinova* :angle-vector) 5000)
  (let (target-coords)
    (while (ros::ok)
      (setq target-coords (get-person-pose))
      (if target-coords
          (progn
            (ros::ros-info "Found a person")
            (look-at target-coords)
            (send *ri* :speak-jp "こんにちは")
            )
          (progn
            (ros::ros-info "No person")
            (fold-arm)))
      )))

(main)
