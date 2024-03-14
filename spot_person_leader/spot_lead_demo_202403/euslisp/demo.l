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
  (
  )

(defun callback-bbox-array (msg)
  (if (or (not *current-target-label*)
          (= (length (send msg :boxes) 0)))
      (return-from callback)
      )

  (if *current-target-label*
      (let* (target
             (targets
              (remove-if-not
               (lambda (bbox) (= *current-target-label* (send bbox :label)))
               (send msg :boxes)
               ))
             )
        (if (> (length targets) 0)
            (progn
              (setq target-coords
                    (convert-robot-based-coords
                     (ros::tf-pose->coords (send (elt targets 0) :pose))))
              (move-robot target-coords)
              (return-from callback)
              )
            )
        ))

  (let (nearest-bbox)
    (setq nearest-bbox
          (reduce
           (lambda (bbox-a bbox-b)
             (if (< (distance
                     (send (ros::tf-pose->coords (send bbox-a :pose)) :pos)
                     (send *odom-vision-to-body* :pos))
                    (distance
                     (send (ros::tf-pose->coords (send bbox-b :pose)) :pos)
                     (send *odom-vision-to-body* :pos)))
                 bbox-a
                 bbox-b))

           ))
    (setq *current-target-label*
          (send nearest-bbox :label))
    (move-robot
     (convert-robot-based-coords
      (ros::tf-pose->coords (send nearest-box :pose))))
    )
  )

(defun callback-odom (msg)
  (setq *odom-vision-to-body*
        (ros::tf-pose->coords
         (send msg :pose :pose)))
  )


(defun main ()
  (spot-init)
  (ros::subscribe "/spot/odometry" nav_msgs::Odometry #'callback-odom)
  (ros::subscribe "/spot/tracked_world_objects" jsk_recognition_msgs::BoundingBoxArray #'callback-bbox-array)
  (ros::spin)
  )

(main)
