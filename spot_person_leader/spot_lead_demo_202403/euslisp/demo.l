#!/usr/bin/env roseus

(ros::load-ros-package "jsk_recognition_msgs")
(ros::load-ros-package "speech_recognition_msgs")
(ros::load-ros-package "spot_lead_demo_202403")

(load "package://spoteus/spot-interface.l")
(load "package://ros_lock/euslisp/ros_lock.l")

(setq *follow-active* t)
(setq *current-target-label* nil)
(setq *odom-vision-to-body* nil)

(setq *last-target-coords-from-vision* nil)
(setq *last-target-stamp* nil)

(setq *distance-min* 800)
(setq *distance-max* 20000.0)

(setq *max-lateral-speed* 2000.0)
(setq *max-rotation-speed* 1.0)

(setq *timeout-duration* 1.0)

(setq *ros-lock-mobility* (instance ros-lock :init "mobility"))
(setq *ros-lock-arm* (instance ros-lock :init "arm"))

(defun handler-start-follow 
  (req)
  (let
    (
      (res 
        (instance spot_lead_demo_202403::StartFollowResponse :init))
      (target-label 
        (send req :target_label)))
    (setq *current-target-label* target-label)
    (setq *follow-active* t)
    (send res :success t)
res))

(defun convert-robot-based-coords 
  (pose-vision)
  (transform-coords
    (send *odom-vision-to-body* :inverse-transformation)
    (ros::tf-pose->coords pose-vision)))

(defun calc-vtheta 
  (target-direction)
  (let 
    (
      (vtheta 
        (* 1.0 target-direction)))
    (if 
      (> vtheta *max-rotation-speed*)
      (setq vtheta *max-rotation-speed*))
    (if 
      (< vtheta 
        (* -1.0 *max-rotation-speed*))
      (setq vtheta 
        (* -1.0 *max-rotation-speed*)))

vtheta))

(defun calc-vx 
  (target-distance)
  (* 0.0005 target-distance))

(defun move-robot 
  (target-coords-from-robot &key (update-last-info t) (z-offset 1.5))
  (let* 
    (vx vy vtheta msg_target_bbox
      (target-pos 
        (v+ (send target-coords-from-robot :pos) (float-vector 0 0 z-offset)))
      (target-x 
        (elt target-pos 0))
      (target-y 
        (elt target-pos 1))
      (target-direction 
        (atan target-y target-x))
      (target-distance 
        (norm 
          (float-vector target-x target-y 0)))
        )
    (ros::ros-debug 
      (format nil "target pos: ~A" target-pos))
    (ros::ros-debug 
      (format nil "target distance: ~A" target-distance))
    (ros::ros-debug 
      (format nil "target direction: ~A" target-direction))
    (if
      (or
        (> target-distance *distance-max*)
        (< target-distance *distance-min*))
      (progn
        (ros::ros-info 
          (format nil "Target distanct ~A out of range." target-distance))
        (return-from move-robot)))

    (if (and nil (send *ros-lock-arm* :acquire :timeout 2))
      (progn
        (send *spot* :angle-vector (send *ri* :state :potentio-vector))
        (send *spot* :head :look-at target-pos)
        (send *ri* :angle-vector (send *spot* :angle-vector) 1000)
        (send *ri* :wait-interpolation)
        (send *ros-lock-arm* :release)
        ))

    (setq msg_target_bbox
      (instance jsk_recognition_msgs::BoundingBox :init))
    (send msg_target_bbox :header :stamp 
      (ros::time-now))
    (send msg_target_bbox :header :frame_id "vision")
    (send msg_target_bbox :pose
      (ros::coords->tf-pose 
        (transform-coords *odom-vision-to-body* target-coords-from-robot)))
    (send msg_target_bbox :dimensions :x 1.0)
    (send msg_target_bbox :dimensions :y 1.0)
    (send msg_target_bbox :dimensions :z 1.5)
    (ros::publish "/spot/target_bbox" msg_target_bbox)

    (if update-last-info
      (progn
        (setq *last-target-coords-from-vision*
              (transform-coords *odom-vision-to-body* target-coords-from-robot))
        (setq *last-target-stamp* (ros::time-now))
        ))

    (setq vtheta 
      (calc-vtheta target-direction))
    (setq vx 
      (calc-vx target-distance))

    (if (not (send *ros-lock-mobility* :acquire :timeout 2))
      (progn
        (ros::ros-info
          (format nil "Failed to acquire lock"))
        (return-from move-robot)))
    (if 
      (or 
        (> target-direction 1.5)
        (< target-direction -1.5))
      (progn
        (send *ri* :send-cmd-vel-raw 0 0 vtheta :topic-name "/demo_input/cmd_vel")
        (ros::ros-debug "Sent velocitay [0 0 ~A]" vtheta))
      (progn
        (send *ri* :send-cmd-vel-raw vx 0 vtheta :topic-name "/demo_input/cmd_vel")
        (ros::ros-debug "Sent velocity [~A 0 ~A]" vx vtheta)))
    (send *ros-lock-mobility* :release)
))

(defun callback-bbox-array 
  (msg)
  (if 
    (or 
      (not *odom-vision-to-body*)
      (= 
        (length 
          (send msg :boxes)) 0)
      (not *follow-active*))
    (return-from callback-bbox-array))

  (if *current-target-label*
    (let* 
      (target)
      (dolist 
        (bbox 
          (send msg :boxes))
        (if 
          (= *current-target-label* 
            (send bbox :label))
          (setq target bbox)))
      (if target
        (progn
          (setq target-coords
            (convert-robot-based-coords
              (send target :pose)))
          (move-robot target-coords)
          (return-from callback-bbox-array))
        (if (and *last-target-stamp*
                 (ros::time<
                   (ros::time- (ros::time-now) *last-target-stamp*)
                   (ros::time *timeout-duration*)))
          (let ((last-target-coords-from-robot
                (transform-coords
                  (send *odom-vision-to-body* :inverse-transformation)
                  *last-target-coords-from-vision*)))
            (move-robot last-target-coords-from-robot :update-last-info nil)
            (return-from callback-bbox-array))
          (progn
            (setq *last-target-stamp* nil)
            )
          )
        )))

  (let
    (nearest-bbox
     (reference-coords-on-vision
       (if *last-target-coords-from-vision*
           (send *last-target-coords-from-vision* :pos)
         (send *odom-vision-to-body* :pos)))
       )
    (dolist
      (bbox
        (send msg :boxes))
      (if 
        (not nearest-bbox)
        (setq nearest-bbox bbox)
        (progn
          (if 
            (< 
              (distance
                (send 
                  (ros::tf-pose->coords 
                    (send bbox :pose)) :pos)
                reference-coords-on-vision)
              (distance
                (send 
                  (ros::tf-pose->coords 
                    (send nearest-bbox :pose)) :pos)
                reference-coords-on-vision)
                ))
            (setq nearest-bbox bbox))))
    (setq *current-target-label*
      (send nearest-bbox :label))
    (move-robot
      (convert-robot-based-coords
        (send nearest-bbox :pose))))
)

(defun callback-odom 
  (msg)
  (setq *odom-vision-to-body*
    (ros::tf-pose->coords
      (send msg :pose :pose)))
)

(defun callback-stt 
  (msg)
  (if 
    (= 
      (length 
        (send msg :transcript)) 0)
    (return-from callback-stt))

  (let 
    (
      (text 
        (elt 
          (send msg :transcript) 0)))
    (ros::ros-info 
      (format nil "stt: ~A" text))
    (cond
      (
        (or 
          (string= text "起きて")
          (string= text "起きろ"))
        (progn
          (setq *follow-active* nil)
          (send *ri* :undock)
          (send *ri* :claim)
          (send *ri* :power-on)))
      (
        (or 
          (string= text "休んで")
          (string= text "休め"))
        (progn
          (setq *follow-active* nil)
          (send *ri* :stow-arm)
          (send *ri* :dock 521)
          (send *ri* :sit)
          ))
      (
        (or 
          (string= text "座って")
          (string= text "すわって"))
        (progn
          (setq *follow-active* nil)
          (send *ri* :sit)))
      (
        (or 
          (string= text "立って")
          (string= text "立て")
          (string= text "たって"))
        (progn
          (setq *follow-active* nil)
          (send *ri* :stand)))
      (
        (or 
          (string= text "ちょっと待ってて")
          (string= text "ちょっと待って")
          (string= text "ここで待ってて")
          (string= text "ここで待って")
          (string= text "待って")
          (string= text "止まって")
          (string= text "止まれ")
          (string= text "待て"))
        (progn
          (send *ri* :speak-jp "ここで待ちます" :wait t)
          (setq *follow-active* nil)
          (setq *current-target-label* nil)
          (send *ri* :stow-arm)
          (send *ri* :stand)))
      (
        (or 
          (string= text "こっちへ来て")
          (string= text "来て")
          (string= text "来い")
          (string= text "来て")
          (string= text "こっち来て"))
        (progn
          (send *ri* :speak-jp "ついていきます" :wait t)
          (setq *current-target-label* nil)
          (setq *last-target-stamp* nil)
          (setq *last-target-coords-from-vision* nil)
          (setq *follow-active* t)
          (send *spot* :reset-pose)
          (send *ri* :angle-vector (send *spot* :angle-vector) 1000)
          (send *ri* :stand)))
      (t 
        (progn
          (ros::ros-error "Unknown text")))
)
)
)


(defun main 
  ()
  (ros::roseus "spot_lead_demo_main")

  (setq *auto-start* (ros::get-param "~auto_start" t))
  (if *auto-start*
    (setq *follow-active* t)
    (setq *follow-active* nil))

  (spot-init)
  (ros::subscribe "/spot/odometry" nav_msgs::Odometry #'callback-odom)
  (ros::subscribe "/spot/tracked_world_objects" jsk_recognition_msgs::BoundingBoxArray #'callback-bbox-array)
  (ros::subscribe "/speech_to_text" speech_recognition_msgs::SpeechRecognitionCandidates #'callback-stt)
  (ros::advertise "/spot/target_bbox" jsk_recognition_msgs::BoundingBox)
  (ros::advertise-service "/spot/start_follow" spot_lead_demo_202403::StartFollow #'handler-start-follow)
  (ros::ros-info "Initialized.")
  (ros::spin)
)

(main)
