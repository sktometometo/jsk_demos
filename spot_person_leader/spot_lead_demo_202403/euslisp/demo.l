#!/usr/bin/env roseus

(ros::load-ros-package "jsk_recognition_msgs")
(ros::load-ros-package "speech_recognition_msgs")
(load "package://spoteus/spot-interface.l")

(setq *follow-active* t)
(setq *current-target-label* nil)
(setq *odom-vision-to-body* nil)

(setq *distance-min* 800)
(setq *distance-max* 20000.0)

(setq *max-lateral-speed* 2000.0)
(setq *max-rotation-speed* 1.0)

(defun convert-robot-based-coords (pose-vision)
  (transform-coords


   (send *odom-vision-to-body* :inverse-transformation)
   (ros::tf-pose->coords pose-vision)))

(defun calc-vtheta (target-direction)
  (let ((vtheta (* 1.0 target-direction)))
   (if (> vtheta *max-rotation-speed*)
     (setq vtheta *max-rotation-speed*))
   (if (< vtheta (* -1.0 *max-rotation-speed*))
     (setq vtheta (* -1.0 *max-rotation-speed*)))
    vtheta))

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
          (= (length (send msg :boxes)) 0)
          (not *follow-active*))
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

(defun callback-stt (msg)
  (if (= (length (send msg :transcript)) 0)
      (return-from callback-stt))

  (let ((text (elt (send msg :transcript) 0)))
    (ros::ros-info (format nil "stt: ~A" text))
    (cond
      ((or (string= text "起きて")
           (string= text "起きろ"))
       (progn
         (setq *follow-active* nil)
         (send *ri* :undock)
         (send *ri* :claim)
         (send *ri* :power-on)))
      ((or (string= text "休んで")
           (string= text "休め"))
       (progn
         (setq *follow-active* nil)
         (send *ri* :dock 521)
         (send *ri* :sit)
         (send *ri* :power-off)
         (send *ri* :release)))
      ((or (string= text "座って")
           (string= text "すわって"))
       (progn
         (setq *follow-active* nil)
         (send *ri* :sit)))
      ((or (string= text "立って")
           (string= text "立て")
           (string= text "たって"))
       (progn
         (setq *follow-active* nil)
         (send *ri* :stand)))
      ((or (string= text "ちょっと待ってて")
           (string= text "ちょっと待って")
           (string= text "ここで待ってて")
           (string= text "ここで待って")
           (string= text "待って")
           (string= text "待て"))
       (progn
         (setq *follow-active* nil)
         (setq *current-target-label* nil)
         (send *ri* :stand)))
      ((or (string= text "こっちへ来て")
           (string= text "来て")
           (string= text "来い")
           (string= text "来て")
           (string= text "こっち来て"))
       (progn
         (setq *current-target-label* nil)
         (setq *follow-active* t)
         (send *ri* :stand)))
      (t (progn
           (ros::ros-error "Unknown text")))
      )
    )
  )


(defun main ()
  (spot-init)
  (send *ri* :sit)
  (send *ri* :power-off)
  (send *ri* :release)
  (send *ri* :claim)
  (send *ri* :power-on)
  (send *ri* :stand)
  (ros::subscribe "/spot/odometry" nav_msgs::Odometry #'callback-odom)
  (ros::subscribe "/spot/tracked_world_objects" jsk_recognition_msgs::BoundingBoxArray #'callback-bbox-array)
  (ros::subscribe "/speech_to_text" speech_recognition_msgs::SpeechRecognitionCandidates #'callback-stt)
  (ros::ros-info "Initialized.")
  (ros::spin)
  )

(main)
