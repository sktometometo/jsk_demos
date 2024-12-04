#!/usr/bin/env roseus

(ros::load-ros-package "jsk_recognition_msgs")

(load "package://spoteus/spot-interface.l")

;; Variables
(setq *current-target-label* nil)
(setq *odom-vision-to-body* nil)
(setq *last-target-coords-from-vision* nil)
(setq *last-target-stamp* nil)

;; Parameters
(setq *param-distance-min** 800) ;; mm
(setq *param-distance-max** 20000.0) ;; mm
(setq *max-lateral-speed* 2.0) ;; m/sec
(setq *max-rotation-speed* 1.0) ;; rad/sec
(setq *timeout-duration* 1.0) ;; sec


(defun convert-vision-based-pose-to-robot-based-coords (pose-vision)
  "Convert vision-based tf-pose to robot-based coordinates."
  (convert-vision-based-coords-to-robot-based-coords (ros::tf-pose->coords pose-vision)))

(defun convert-vision-based-coords-to-robot-based-coords (coords-vision)
  "Convert vision-based coords to robot-based coordinates."
  (transform-coords
    (send *odom-vision-to-body* :inverse-transformation)
    coords-vision))

(defun calc-vtheta (target-direction &key (scale 1.0))
  "Calculate angular velocity (rad/sec) based on target direction (rad)."
  (let ((vtheta (* scale target-direction)))
    (if (> vtheta *max-rotation-speed*)
      *max-rotation-speed*
      (if (< vtheta (* -1.0 *max-rotation-speed*))
        (* -1.0 *max-rotation-speed*)
        vtheta))))

(defun calc-vx (target-distance &key (scale 0.0005))
  "Calculate linear velocity (m/sec) based on target distance (mm)"
  (let ((vtheta (* scale target-distance)))
    (if (> vtheta *max-lateral-speed*)
      *max-lateral-speed*
      vtheta)))

(defun publish-target-bbox (target-coords-from-robot)
  (let (msg-target-bbox)
    ;; Publish target bbox
    (setq msg-target-bbox (instance jsk_recognition_msgs::BoundingBox :init))
    (send msg-target-bbox :header :stamp (ros::time-now))
    (send msg-target-bbox :header :frame_id "vision")
    (send msg-target-bbox :pose (ros::coords->tf-pose (transform-coords *odom-vision-to-body* target-coords-from-robot)))
    (send msg-target-bbox :dimensions :x 1.0)
    (send msg-target-bbox :dimensions :y 1.0)
    (send msg-target-bbox :dimensions :z 1.5)
    (ros::publish "/spot/target_bbox" msg-target-bbox)))

(defun move-robot (target-coords-from-robot &key (z-offset 1.5))
  (let* (vx
         vy
         vtheta
         (target-pos (v+ (send target-coords-from-robot :pos) (float-vector 0 0 z-offset)))
         (target-x (elt target-pos 0))
         (target-y (elt target-pos 1))
         (target-direction (atan target-y target-x)) ;; rad
         (target-distance (norm (float-vector target-x target-y 0))) ;; mm
         )
    (ros::ros-debug (format nil "target pos: ~A" target-pos))
    (ros::ros-debug (format nil "target distance: ~A rad" target-distance))
    (ros::ros-debug (format nil "target direction: ~A mm" target-direction))
    (if (or (> target-distance *param-distance-max**)
            (< target-distance *param-distance-min**))
      (progn
        (ros::ros-info (format nil "Target distanct ~A out of range." target-distance))
        (return-from move-robot)))

    ;; calc command and send
    (setq vtheta (calc-vtheta target-direction)) ;; rad/sec
    (setq vx (calc-vx target-distance)) ;; m/sec
    (if (or (> target-direction 1.5) (< target-direction -1.5))
      (progn
        (send *ri* :send-cmd-vel-raw 0 0 vtheta :topic-name "/demo_input/cmd_vel")
        (ros::ros-debug "Sent velocity [0 0 ~A]" vtheta))
      (progn
        (send *ri* :send-cmd-vel-raw vx 0 vtheta :topic-name "/demo_input/cmd_vel")
        (ros::ros-debug "Sent velocity [~A 0 ~A]" vx vtheta)))))


(defun callback-bbox-array (msg)
  "Call back func for world object array. Calculate target label and store its coordinate on fixed frame"
  (if (or (not *odom-vision-to-body*)
          (= (length (send msg :boxes)) 0))
    (return-from callback-bbox-array))

  (if *current-target-label*
    ;; When already target label is set, move to the target.
    (let* (target-bbox target-coords)
      (dolist (bbox (send msg :boxes))
        (if (= *current-target-label* (send bbox :label))
          (setq target-bbox bbox)))
      (if target-bbox
        (let ((target-coords-from-robot (convert-vision-based-pose-to-robot-based-coords (send target-bbox :pose))))
          ;(move-robot target-coords-from-robot)
          ;(publish-target-bbox target-coords-from-robot)
          (setq *last-target-coords-from-vision* (transform-coords *odom-vision-to-body* target-coords-from-robot))
          (setq *last-target-stamp* (ros::time-now))
          (return-from callback-bbox-array))
        (if (and *last-target-coords-from-vision*
                 *last-target-stamp*
                 (ros::time<
                   (ros::time- (ros::time-now) *last-target-stamp*)
                   (ros::time *timeout-duration*)))
          (let ((last-target-coords-from-robot (transform-coords
                                                 (send *odom-vision-to-body* :inverse-transformation)
                                                 *last-target-coords-from-vision*)))
            ;(move-robot last-target-coords-from-robot)
            ;(publish-target-bbox last-target-coords-from-robot)
            (return-from callback-bbox-array))
          (progn
            (ros::ros-info "Target not found.")
            ;; Target not found and timeout
            (setq *last-target-stamp* nil)
            (setq *last-target-coords-from-vision* nil)
            (setq *current-target-label* nil)))))
    ;; Choose nearest bbox to ref coords as target when target label is not set.
    (let
      (nearest-bbox
      (reference-coords-on-vision
          (if *last-target-coords-from-vision*
              (send *last-target-coords-from-vision* :pos)
            (send *odom-vision-to-body* :pos)))
          )
      (dolist (bbox (send msg :boxes))
        (if (not nearest-bbox)
          (setq nearest-bbox bbox)
          (progn
            (if (< (distance
                    (send (ros::tf-pose->coords (send bbox :pose)) :pos)
                    reference-coords-on-vision)
                  (distance
                    (send (ros::tf-pose->coords (send nearest-bbox :pose)) :pos) 
                    reference-coords-on-vision))
              (setq nearest-bbox bbox)))))
      (if nearest-bbox
        (progn
          (setq *current-target-label* (send nearest-bbox :label))
          (ros::ros-info (format nil "Target label is set to ~A" *current-target-label*)))
        (progn
          (ros::ros-info "No target found.")
          (return-from callback-bbox-array))))
    )
)

(defun callback-odom (msg)
  (setq *odom-vision-to-body* (ros::tf-pose->coords (send msg :pose :pose))))

(defun main ()
  (ros::roseus "spot_lead_demo_main")
  (spot-init :auto-start nil)
  (ros::advertise "/spot/target_bbox" jsk_recognition_msgs::BoundingBox)
  (ros::subscribe "/spot/odometry" nav_msgs::Odometry #'callback-odom)
  (ros::subscribe "/spot/tracked_world_objects" jsk_recognition_msgs::BoundingBoxArray #'callback-bbox-array)
  (ros::ros-info "Initialized.")
  (while (ros::ok)
    (ros::spin-once)
    (if (and *odom-vision-to-body*
             *last-target-coords-from-vision*
             *last-target-stamp*
             (ros::time<
               (ros::time- (ros::time-now) *last-target-stamp*)
               (ros::time *timeout-duration*)))
      (let ((last-target-coords-from-robot
              (convert-vision-based-coords-to-robot-based-coords
                *last-target-coords-from-vision*)))
        (move-robot last-target-coords-from-robot)
        (publish-target-bbox last-target-coords-from-robot)
        )
      )
    )
  )

(main)
