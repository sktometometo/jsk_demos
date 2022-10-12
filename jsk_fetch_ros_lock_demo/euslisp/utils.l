(require "package://fetcheus/fetch-interface.l")
(require "package://ros_lock/euslisp/ros_lock.l")

(defun jsk-fetch-ros-lock-init (&key (move-base-action-name "move_base_no_recovery_with_lock"))
  (setq *ri* (instance fetch-interface :init :move-base-action-name move-base-action-name))
  (fetch-init)

  (if (not (boundp `*tfl*))
    (setq *tfl* (instance ros::transform-listener :init)))

  (setq *fixed-frame-id* (ros::get-param "~fixed_frame_id" "odom"))
  (setq *base-frame-id* (ros::get-param "~base_frame_id" "base_link"))

  (setq *ros-lock* (instance ros-lock :init))
  (send *ros-lock* :wait-for-server)

  (setq *target-person-coord-robotbased-data* nil)
  (setq *target-person-coord-robotbased-stamp* nil)
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
          (setq *target-person-coord-robotbased-data* nil)
          (setq *target-person-coord-robotbased-stamp* (send msg :header :stamp))
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
            (setq *target-person-coord-robotbased-data*
                (elt
                  (sort people-coords-base-based
                        #'(lambda (coord-a coord-b)
                            (<= (norm (send coord-a :pos))
                                (norm (send coord-b :pos))
                                )
                            ))
                  0))
            (setq *target-person-coord-robotbased-stamp* (send msg :header :stamp))
            ))
        )
      ))

  (ros::subscribe "~input_bbox_array" jsk_recognition_msgs::BoundingBoxArray #'callback)
  )

;; TODO
;;  - 通常のnavigation
;;  - 頭だけ動かす
;;  - 手で挨拶
;;  - お菓子を差し出す
