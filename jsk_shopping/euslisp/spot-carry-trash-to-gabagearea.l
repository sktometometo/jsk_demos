#!/usr/bin/env roseus

(load "package://spoteus/spot-interface.l")
(ros::load-ros-manifest "speech_recognition_msgs")


(defvar *spot-autowalk-data-path-73b2-to-7felevator*
  "/home/spot/spot_ws/src/jsk-ros-pkg/jsk_robot/jsk_spot_robot/spot_autowalk_data")
(defvar *spot-autowalk-data-path-7felevator-to-1felevator*
  "/home/spot/spot_ws/src/jsk-ros-pkg/jsk_robot/jsk_spot_robot/spot_autowalk_data")
(defvar *spot-autowalk-data-path-1felevator-to-gabagearea*
  "/home/spot/spot_ws/src/jsk-ros-pkg/jsk_robot/jsk_spot_robot/spot_autowalk_data")

(defun main (&key (enr-mode nil))
  (spot-init)

  ;; Wait for trigger
  (let (lift)
    (while (not lift)
      (when (substringp "運んでください" (send *ri* :speech-to-text 100000))
        (setq lift t)))
    )

  ;; Pick up trash
  ;; TODO
  (send *ri* :stop-grasp)
  (send *ri* :unstow-arm)
  (send *ri* :speak-jp "わかりました" :wait t)

  ;; find object
  ;; get object coord to object-coords
  (let (object-coords)
    (setq object-coords (find-object))
    (send *ri* :pick-object object-coords) ;; TODO
    (send *spot* :angle-vector (send *ri* :state :potentio-vector))
    (send *spot* :inverse-kinematics (make-coords :pos #f(800 0 300) :rpy #f(0 0 0)))
    (send *ri* :angle-vector (send *spot* :angle-vector))
    (send *ri* :speak-jp "ゴミ袋を受け取りました" :wait t)
    )

  ;; Carry it to gabage area
  (send *ri* :speak-jp "ゴミ袋を運搬します" :wait t)
  (send *ri* :upload-graph *spot-autowalk-data-path-73b2-to-7felevator*)
  (send *ri* :set-localization-fiducial)
  (send *ri* :navigate-to "")

  (send *ri* :upload-graph *spot-autowalk-data-path-7felevator-to-1felevator*)
  ;;; TODO: Call elevator
  ;;; TODO: When it arrives
  (send *ri* :navigate-to "")
  ;;; TODO: When it arrives 1F
  (send *ri* :navigate-to "")

  (send *ri* :upload-graph *spot-autowalk-data-path-1felevator-to-gabagearea*)
  (send *ri* :navigate-to "")

  ;;
  (send *ri* :speak-jp "ゴミを持ってきました．受け取ってください" :wait t)
  (send *spot* :reset-pose)
  (send *ri* :angle-vector (send *spot* :angle-vector))
  (send *ri* :wait-interpolation)
  ;;; TODO: When it recognizes it is received.
  (send *ri* :navigate-to "")
  )

(main)
