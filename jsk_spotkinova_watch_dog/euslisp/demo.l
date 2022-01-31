#!/usr/bin/env roseus

(load "package://spotkinovaeus/spotkinova-interface.l")

(ros::roseus-add-msgs "geometry_msgs")

(defun look-at (target-coords)
  (let ((target-pos (send (send target-coords :translate #f(0 0 500)) :pos))
        ret)
    ;; link-list args seems no effect...
    ;;(setq ret (send *spotkinova* :head :look-at target-pos :link-list (list (send *spotkinova* :link "kinova_shoulder_link") (send *spotkinova* :link "kinova_forearm_link"))))
    (setq ret (send *spotkinova* :head :look-at target-pos))
    (if (not ret)
      (return-from look-at nil)
      nil)
    (setq ret (send *ri* :angle-vector (send *spotkinova* :angle-vector) 3000))
    (if (not ret)
      (return-from look-at nil)
      nil)
    (send *ri* :wait-interpolation)
    t))

(defun fold-arm ()
  (send *spotkinova* :kinova-rest-pose)
  (send *ri* :angle-vector (send *spotkinova* :angle-vector) 5000)
  (send *ri* :wait-interpolation)
  )

(defun move-to-look-at-init ()
  (setq look-at-init-pose #f(0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 45.0 -90.0 0.0 20.0 90.0 -90.0 -20.0 -90.0))
  (send *ri* :angle-vector look-at-init-pose 5000)
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
  (ros::ros-info "Move to look-at-init-pose")
  (move-to-look-at-init)
  (let (target-coords)
    (while (ros::ok)
      (setq target-coords (get-person-pose))
      (if target-coords
          (progn
            (ros::ros-info "Found a person")
            (if (look-at target-coords)
              (send *ri* :speak-jp "こんにちは" :wait t)
              nil)
            )
          (progn
            (ros::ros-info "No person")
            (fold-arm)
            ))
      )
    (fold-arm)
    ))

(main)
