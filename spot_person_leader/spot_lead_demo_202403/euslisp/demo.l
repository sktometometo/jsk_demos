#!/usr/bin/env roseus

(ros::load-ros-package "jsk_recognition_msgs")
(load "package://spoteus/spot-interface.l")

(setq *current-target-label* nil)
(setq *odom-vision-to-body* nil)

(setq *distance-min* 500)
(setq *distance-max* 10000.0)

(setq *min-lateral-speed* 500)
(setq *max-lateral-speed* 2000.0)

(defun convert-robot-based-coords (pose-vision)
  (transform-coords


   (send *odom-vision-to-body* :inverse-transformation)
   (ros::tf-pose->coords pose-vision)))

(defun calc-vtheta (target-direction)
  (* 1.0 target-direction))

(defun calc-vx (target-distance)
  (* 0.0005 target-distance))

(defun move-robot (target-coords-from-robot)
  (let* (vx vy vtheta
         (target-pos (send target-coords-from-robot :pos))
         (target-x (elt target-pos 0))
         (target-y (elt target-pos 1))
         (target-direction (atan target-y target-x))
         (target-distance (norm (float-vector target-x target-y 0)))
         )
    (ros::ros-info (format nil "target pos: ~A" target-pos))
    (ros::ros-info (format nil "target distance: ~A" target-distance))
    (ros::ros-info (format nil "target direction: ~A" target-direction))
    (if (or (> target-distance *distance-max*)
            (< target-distance *distance-min*))
        (return-from move-robot))

    (setq vtheta (calc-vtheta target-direction))
    (setq vx (calc-vx target-distance))
    (if (or (> target-direction 1.5)
            (< target-direction -1.5))
        (progn
          (send *ri* :send-cmd-vel-raw 0 0 vtheta :topic-name "/spot/cmd_vel_unsmoothed")
          (ros::ros-info "Sent velocitay [0 0 ~A]" vtheta))
        (progn
          (send *ri* :send-cmd-vel-raw vx 0 vtheta :topic-name "/spot/cmd_vel_unsmoothed")
          (ros::ros-info "Sent velocity [~A 0 ~A]" vx vtheta)))
    ))

(defun callback-bbox-array (msg)
  (if (or (not *odom-vision-to-body*)
          (= (length (send msg :boxes)) 0))
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
                     (send target :pose)))
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
      (send nearest-bbox :pose)))
    )
  )

(defun callback-odom (msg)
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
