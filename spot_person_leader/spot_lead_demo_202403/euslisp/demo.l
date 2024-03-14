#!/usr/bin/env roseus

(ros::load-ros-package "jsk_recognition_msgs")
(load "package://spoteus/spot-interface.l")

(setq *current-target-label* nil)
(setq *odom-vision-to-body* nil)

(defun convert-robot-based-coords (pose-vision)
  (transform-coords
   (send *odom-vision-to-body* :inverse-transformation)
   (ros::tf-pose->coords pose-vision)))

(defun move-robot (target-coords-from-robot)
  (let ((target-pose (send target-coords-from-robot :pos)))
    (ros::ros-info (format nil "target pos: ~A~%"))
  ))

(defun callback-bbox-array (msg)
  (ros::ros-info "callback-bbox-array")

  (if (= (length (send msg :boxes)) 0)
      (return-from callback-bbox-array)
      )

  (if *current-target-label*
      (let* (target)
        (dolist (bbox (send msg :boxes))
          (if (= *current-target-label* (send bbox :label))
              (setq target bbox)
              ))
        (if target
            (progn
              (setq target-coords
                    (convert-robot-based-coords
                     (ros::tf-pose->coords (send target :pose))))
              (move-robot target-coords)
              (return-from callback-bbox-array)
              )
            )
        ))

  (let (nearest-bbox)
    (dolist (bbox (send msg :boxes))
      (if (not nearest-bbox)
          (setq nearest-bbox bbox)
          (progn
            (if (< (distance
                     (send (ros::tf-pose->coords (send bbox :pose)) :pos)
                     (send *odom-vision-to-body* :pos))
                   (distance
                     (send (ros::tf-pose->coords (send nearest-bbox :pose)) :pos)
                     (send *odom-vision-to-body* :pos)))
                (setq nearest-bbox bbox)))))

    (setq *current-target-label*
          (send nearest-bbox :label))
    (move-robot
     (convert-robot-based-coords
      (ros::tf-pose->coords (send nearest-bbox :pose))))
    )
  )

(defun callback-odom (msg)
  (ros::ros-info "callback odom")
  (setq *odom-vision-to-body*
        (ros::tf-pose->coords
         (send msg :pose :pose)))
  )


(defun main ()
  (spot-init)
  (send *ri* :power-on)
  (send *ri* :stand)
  (ros::subscribe "/spot/odometry" nav_msgs::Odometry #'callback-odom)
  (ros::subscribe "/spot/tracked_world_objects" jsk_recognition_msgs::BoundingBoxArray #'callback-bbox-array)
  (ros::ros-info "Initialized.")
  (ros::spin)
  )

(main)
