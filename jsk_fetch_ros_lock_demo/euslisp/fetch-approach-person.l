#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_recognition_msgs")

(load "package://fetcheus/fetch-interface.l")
(load "package://ros_lock/euslisp/ros_lock.l")

(ros::roseus "fetch_approach_person")
(setq *ri* (instance fetch-interface :init :move-base-action-name "move_base_no_recovery"))
(fetch-init)

(if (not (boundp `*tfl*))
  (setq *tfl* (instance ros::transform-listener :init)))

(setq *fixed-frame-id* (ros::get-param "~fixed_frame_id" "odom"))
(setq *base-frame-id* (ros::get-param "~base_frame_id" "base_link"))

(setq *ros-lock* (instance ros-lock :init "base"))
(send *ros-lock* :wait-for-server)

(setq *target-person-coord-data* nil)
(setq *target-person-coord-stamp* nil)
(setq *last-stamp-approach* (ros::time))

(defun callback (msg)
  (let* (bbox-coord
         (bbox-frame-id (send msg :header :frame_id))
         (coord-base-to-bbox
            (send *tfl* :lookup-transform
                  *base-frame-id*
                  bbox-frame-id
                  (ros::time)))
         (people-coords-fixed-based nil)
         )
    (if (not (send msg :boxes))
      (progn
        (setq *target-person-coord-data* nil)
        (setq *target-person-coord-stamp* (send msg :header :stamp))
        )
      (if coord-base-to-bbox
        (progn
          (setq people-coords-base-based
                  (mapcar
                     #'(lambda (bbox)
                         (transform-coords
                           coord-base-to-bbox
                           (ros::tf-pose->coords (send bbox :pose))))
                     (send msg :boxes))
                )
          (setq *target-person-coord-data*
              (elt
                (sort people-coords-base-based
                      #'(lambda (coord-a coord-b)
                          (<= (norm (send coord-a :pos))
                              (norm (send coord-b :pos))
                              )
                          ))
                0))
          (setq *target-person-coord-stamp* (send msg :header :stamp))
          ))
      )
    ))

(ros::subscribe "~input_bbox_array" jsk_recognition_msgs::BoundingBoxArray #'callback)
(ros::advertise "~output" geometry_msgs::PoseStamped)

(ros::rate 1)
(while (ros::ok)
  (ros::spin-once)
  (send *ri* :spin-once)
  (ros::sleep)
  (ros::ros-info "*target-person-coord-data*: ~A" *target-person-coord-data*)
  (ros::ros-info "*target-person-coord-stamp*: ~A" *target-person-coord-stamp*)
  (if (and *target-person-coord-data* *target-person-coord-stamp*)
    (let* (ret
           (pos-x (* 0.001 (elt (send *target-person-coord-data* :pos) 0)))
           (pos-y (* 0.001 (elt (send *target-person-coord-data* :pos) 1)))
           (distance (norm (send *target-person-coord-data* :pos)))
           (delay (ros::time- (ros::time-now) *target-person-coord-stamp*))
           (duration-from-last (ros::time- (ros::time-now) *last-stamp-approach*))
           (target-x (* pos-x (/ (- distance 0.5) distance)))
           (target-y (* pos-y (/ (- distance 0.5) distance)))
           (target-theta (atan pos-y pos-x))
           )
      (ros::ros-info "distance: ~A" distance)
      (ros::ros-info "delay: ~A" delay)
      (if (and (ros::time< delay (send (instance ros::time :init) :from-sec 2.0))
               (ros::time> duration-from-last (send (instance ros::time :init) :from-sec 10.0))
               ;;(> pos-y 0)
               )
        (progn
          ;; Acquire lock
          (setq ret (send *ros-lock* :acquire :timeout 10))
          (if (not ret)
            (progn
              (ros::ros-error "Unable to acquire lock for base")
              (exit 1)
              )
            )
          ;; aproach to person
          (ros::ros-info "*target-person-coord-data*: ~A" *target-person-coord-data*)
          (send *ri* :go-pos target-x target-y (rad2deg target-theta))
          (ros::spin-once)
          (if *target-person-coord-data*
              (send *ri* :point-head (send *target-person-coord-data* :pos) :wait t)
            )
          (send *ri* :speak-jp "こんにちは")
          ;; Release lock
          (send *ros-lock* :release)
          (ros::ros-error "released")
          ;;
          (setq *last-stamp-approach* *target-person-coord-stamp*)
          )
        )
      )
    (ros::ros-error "Unable to find person")
    ))
