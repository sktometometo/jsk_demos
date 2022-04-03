#!/usr/bin/env roseus

(ros::roseus "watchdog_demo")

(load "package://roseus_smach/src/state-machine-ros.l")

(load "package://jsk_watchdog_demo/scripts/demo-report-interface.l")
(load "package://jsk_watchdog_demo/scripts/watchdog-interface.l")


(defun watchdog-init (&optional (create-viewer))
  ;; robot init
  (load "package://spoteus/euslisp/spot-interface.l")
  (unless (boundp '*spot*) (spot) (send *spot* :reset-pose))
  (unless (boundp '*ri*) (setq *ri* (instance spot-interface :init)))
  ;; demo init
  (unless (boundp '*wi*) (setq *wi* (instance watchdog-interface :init)))
  (unless (boundp '*report-interface*) (setq *report-interface* (instance demo-report-interface :init)))

  (ros::spin-once)
  (send *ri* :spin-once)

  (when create-viewer (objects (list *spot*)))
  )

(defun func-day-basic (userdata-alist)
  (let ((ret nil))
    (ros::ros-debug "day motion, luminance: ~A, stand duration: ~A min"
                    (send *wi* :get-val 'luminance)
                    (/ (send (ros::time- (ros::time-now) (send *wi* :get-val 'stand-start-time)) :to-sec) 60))
    (when (null (send *wi* :bright-surrounding))
      (send *ri* :sit)
      (send *ri* :power-off)
      (ros::ros-info "sit down because surrounding is dark")
      (setq ret :light-off))
    (unless (ros::ok)
      (setq ret :preempted))
    ret))

(defun func-rest (userdata-alist)
  (let ((ret nil))
    (send *report-interface* :upload-images)
    (while (not ret)
      (ros::spin-once)
      (send *ri* :spin-once)
      (setq ret (func-day-basic userdata-alist))
      (if ret (return-from func-rest ret))
      (when (and (send *wi* :ready-stand-temp) 
                 (> (/ (send (ros::time- (ros::time-now) (send *wi* :get-val 'stand-start-time)) :to-sec) 60)
                    (send *wi* :get-val 'param-rest-time-per-hour)))
        (ros::ros-info  "finish rest")
        (send *wi* :set-val 'stand-start-time (ros::time-now))
        (send *ri* :power-on)
        (send *ri* :stand)
        (setq ret :stand))
      (unix::sleep 1) ;; (ros::sleep)
      )
    ret))

(defun func-watch (userdata-alist)
  (let ((ret nil))
    ;;(ros::rate (/ 1 (float (send *wi* :get-val 'param-change-body-duration))))
    (while (not ret)
      (ros::spin-once)
      (send *ri* :spin-once)
      (setq ret (func-day-basic userdata-alist))
      (if ret (return-from func-watch ret))
      ;; rest
      (when (> (/ (send (ros::time- (ros::time-now) (send *wi* :get-val 'stand-start-time)) :to-sec) 60)
             (- 60  (send *wi* :get-val 'param-rest-time-per-hour)))
        (ros::ros-info "have a rest")
        (send *wi* :set-val 'stand-start-time (ros::time-now))
        (send *ri* :sit)
        (send *ri* :power-off)
        (setq ret :sit)
        (return-from func-watch ret))
      ;; check battery temperature
      (when (send *wi* :high-battery-temp)
        (ros::ros-warn "the battery temperature is higher than the threshold: ~A vs ~A, sit down and turn off servo" (send *wi* :get-val 'battery-temp) (send *wi* :get-val 'param-threshold-temp-sit))
        (send *wi* :set-val 'stand-start-time (ros::time-now))
        (send *ri* :sit)
        (send *ri* :power-off)
        (setq ret :sit)
        (return-from func-watch ret))
      ;; watch person
      (let
          ((max-person-height 0)
           (target-person nil)
           (param-threshold-human-height (send *wi* :get-val 'param-threshold-human-height))
           (image-file-name nil)
           (current-time (unix:localtime))
           target-rpy-euler)
        ;; workaround to handle the smaller bbox height when robot look up (head up).
        (if (< (elt (send *wi* :get-val 'ypr-euler) 1) (* (send *wi* :get-val 'param-lookup-angle) 0.5))
            (setq human_height_threshold (* (send *wi* :get-val 'param-threshold-human-height)  0.8)))
        (dolist (rect (send *wi* :get-val 'human-rects))
          (when (and (< (send rect :width) (* (send *wi* :get-val 'param-threshold-human-width) (send rect :height)))
                   (> (send rect :height) (* (send *wi* :get-val 'param-threshold-human-height) (send *wi* :get-val 'paranoma-height)))
                   (> (send rect :height) max-person-height))
            (setq max-person-height (send rect :height))
            (setq target-person rect)))
        (if (null target-person) (setq target-rpy-euler '(0 0 0))
          (progn
            (setq rel-yaw-angle (/ (* (* (- (/ (send *wi* :get-val 'paranoma-width) 2)  (+ (send target-person :x) (/ (send target-person :width) 2)))   2.0) pi) (send *wi* :get-val 'paranoma-width)))
            (if (< (abs rel-yaw-angle) (send *wi* :get-val 'param-max-tracking-yaw))
                (progn
                  (setq target-rpy-euler  (list 0  (send *wi* :get-val 'param-lookup-angle)  (- (+ rel-yaw-angle  (elt (send *wi* :get-val 'ypr-euler) 0)) (send *wi* :get-val 'yaw-offset))))
                  (ros::ros-debug "find good person to look at: target-rpy-euler: ~A, rel-yaw: ~A body yaw: ~A, yaw offset: ~A" target-rpy-euler rel-yaw-angle (elt (send *wi* :get-val 'ypr-euler) 0) (send *wi* :get-val 'yaw-offset))
                  (setq image-file-name
                        (format nil "watchdog-demo-~A-~A-~A-~A-~A-~A.jpg"
                                (+ 1900 (elt current-time 5))
                                (+ 4 (elt current-time 4))
                                (elt current-time 3)
                                (elt current-time 2)
                                (elt current-time 1)
                                (elt current-time 0)
                                ))
                  (send *report-interface* :capture-image image-file-name)
                  (send *ri* :play-sound (send *wi* :get-val 'param-bark-sound-file)))
              (setq target-rpy-euler '(0 0 0)))))
        (send *ri* :body-pose target-rpy-euler))
      (unix::usleep (floor (* (send *wi* :get-val 'param-change-body-duration) 1000000))) ;; single thred, so consider the image luminance calculation (~0.2s)
      )
    ret))

(defun func-dark (userdata-alist)
  (let ((ret nil))
    (send *report-interface* :upload-images)
    (while (not ret)
      (ros::spin-once)
      (send *ri* :spin-once)
      (ros::ros-debug "dark motion, luminance: ~A, stand duration: ~A min"
                      (send *wi* :get-val 'luminance)
                      (/ (send (ros::time- (ros::time-now) (send *wi* :get-val 'stand-start-time)) :to-sec) 60))
      (when (send *wi* :bright-surrounding)
        (ros::ros-info "stand up because surrounding is bright")
        (send *wi* :set-val 'stand-start-time (ros::time-now))
        (send *ri* :power-on)
        (send *ri* :stand)
        (setq ret :light-on))
      (unless (ros::ok)
        (setq ret :preempted))
      (unix::sleep 1))
    ret))


(defun watchdog-sm ()
  (let (sm sm-day)
    (setq sm-day
          (make-state-machine
           '((:watch :preempted :preempted) ;; transitions
             (:watch :sit :rest)
             (:watch :light-off :light-off)
             (:rest :preempted :preempted)
             (:rest :stand :watch)
             (:rest :light-off :light-off))
           '((:watch 'func-watch) ;; function maps
             (:rest 'func-rest))
           '(:watch)      ;; initial
           '(:light-off :preempted) ;; goal
           ))
    (setq sm
          (make-state-machine
           '((:dark-motion :light-on :day-motion) ;; transitions
             (:dark-motion :preempted :preempted)
             (:day-motion :light-off :dark-motion)
             (:day-motion :preempted :preempted))
           `((:dark-motion 'func-dark)  ;; functon maps
             (:day-motion ,sm-day))   ;; set "nestaed state machine"
           '(:dark-motion)      ;; initial
           '(:preempted) ;; goal
           ))
    sm))

;; create robot interface
(watchdog-init nil)

;; state machine
(exec-state-machine (watchdog-sm))

