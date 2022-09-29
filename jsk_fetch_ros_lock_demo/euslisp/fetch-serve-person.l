#!/usr/bin/env roseus

(ros::load-ros-manifest "jsk_recognition_msgs")

(load "package://fetcheus/fetch-interface.l")
(load "package://ros_lock/euslisp/ros_lock.l")

(ros::roseus "fetch_serve_person")
(fetch-init)

(if (not (boundp `*tfl*))
  (setq *tfl* (instance ros::transform-listener :init)))

(setq *fixed-frame-id* (ros::get-param "~fixed_frame_id" "odom"))
(setq *base-frame-id* (ros::get-param "~base_frame_id" "base_link"))

(setq *ros-lock-for-head* (instance ros-lock :init "head"))
(send *ros-lock-for-head* :wait-for-server)
(setq *ros-lock-for-arm* (instance ros-lock :init "arm"))
(send *ros-lock-for-arm* :wait-for-server)
(setq *ros-lock-for-base* (instance ros-lock :init "base"))
(send *ros-lock-for-base* :wait-for-server)

(setq *nearest-people-coord-data* nil)
(setq *nearest-people-coord-stamp* nil)
(setq *last-stamp-approach* (ros::time-now))

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
        (setq *nearest-people-coord-data* nil)
        (setq *nearest-people-coord-stamp* (send msg :header :stamp))
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
          (setq *nearest-people-coord-data*
              (elt
                (sort people-coords-base-based
                      #'(lambda (coord-a coord-b)
                          (<= (norm (send coord-a :pos))
                              (norm (send coord-b :pos))
                              )
                          ))
                0))
          (setq *nearest-people-coord-stamp* (send msg :header :stamp))
          ))
      )
    ))

(ros::subscribe "~input_bbox_array" jsk_recognition_msgs::BoundingBoxArray #'callback)

(ros::rate 1)
(while (ros::ok)
  (ros::spin-once)
  (send *ri* :spin-once)
  (ros::sleep)
  (ros::ros-info "*nearest-people-coord-data*: ~A" *nearest-people-coord-data*)
  (ros::ros-info "*nearest-people-coord-stamp*: ~A" *nearest-people-coord-stamp*)
  (if (and *nearest-people-coord-data* *nearest-people-coord-stamp*)
    (let* (ret
           (pos-x (elt (send *nearest-people-coord-data* :pos) 0))
           (pos-y (elt (send *nearest-people-coord-data* :pos) 1))
           (distance (* 0.001 (norm (send *nearest-people-coord-data* :pos))))
           (delay (ros::time- (ros::time-now) *nearest-people-coord-stamp*))
           (duration-from-last (ros::time- (ros::time-now) *last-stamp-approach*))
           (target-x (* pos-x (/ (- distance 0.5) distance)))
           (target-y (* pos-y (/ (- distance 0.5) distance)))
           (target-theta (atan pos-y pos-x))
           )
      (ros::ros-info "delay: ~A" delay)
      (ros::ros-info "target-theta: ~A" target-theta)
      (if (and  (ros::time< delay (send (instance ros::time :init) :from-sec 2.0))
                (ros::time> duration-from-last (send (instance ros::time :init) :from-sec 10.0))
                ;;(< (rad2deg (abs target-theta)) 45)
                )
        (progn
          ;; Acquire lock
          (setq ret (send *ros-lock-for-arm* :acquire :timeout 10))
          (if (not ret)
            (progn
              (ros::ros-error "Unable to acquire lock for arm")
              (exit 1)
              ))
          (setq ret (send *ros-lock-for-head* :acquire :timeout 10))
          (if (not ret)
            (progn
              (ros::ros-error "Unable to acquire lock for head")
              (exit 1)
              ))
          (setq ret (send *ros-lock-for-base* :acquire))
          (if (not ret)
            (progn
              (ros::ros-error "Unable to acquire lock for base")
              (exit 1)
              ))
          ;; Hold out basket to person
          (send *ri* :go-pos-no-wait target-x target-y (rad2deg target-theta))
          (send *fetch* :inverse-kinematics (make-cascoords #f(500 0 500)) :rotation-axis :x)
          (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
          (send *ri* :speak-jp "お菓子はいかがですか")
          (send *ri* :wait-interpolation)
          (unix:sleep 5)
          (send *fetch* :reset-pose)
          (send *ri* :angle-vector (send *fetch* :angle-vector) 5000)
          (send *ri* :wait-interpolation)
          ;; Release lock
          (send *ros-lock-for-base* :release)
          (send *ros-lock-for-head* :release)
          (send *ros-lock-for-arm* :release)
          (ros::ros-error "released")
          ;;
          (setq *last-stamp-approach* *nearest-people-coord-stamp*)
          )
        (ros::ros-error "Unable to find person")
        ))))
