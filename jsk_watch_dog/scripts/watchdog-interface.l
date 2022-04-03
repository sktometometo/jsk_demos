(defclass watchdog-interface
    :super propertied-object
  :slots (luminance
          ypr-euler
          stand-start-time
          yaw-offset
          battery-temp
          human-rects
          param-threshold-luminance
          param-threshold-temp-sit
          param-threshold-temp-stand
          param-threshold-human-width
          param-threshold-human-height
          param-rest-time-per-hour
          param-max-tracking-yaw
          param-lookup-angle
          param-change-body-duration
          param-bark-sound-file
          paranoma-width
          paranoma-height
          ))

(defmethod watchdog-interface
    (:init
     (&rest args)
     (prog1
         (send-super* :init args)
       ;; rosparam
       (setq param-threshold-luminance (ros::get-param "~luminance_threshold" 0))
       (setq param-threshold-temp-sit (ros::get-param "~temp_threshold_sit" 50))
       (setq param-threshold-temp-stand (ros::get-param "~temp_threshold_stand" 40))
       (setq param-threshold-human-width (ros::get-param "~human_width_threshold" 0.0))
       (setq param-threshold-human-height (ros::get-param "~human_height_threshold" 0.0))
       (setq param-rest-time-per-hour (ros::get-param "~rest_time_per_hour" 0))
       (setq param-max-tracking-yaw (ros::get-param "~max_tracking_yaw" 0.0))
       (setq param-lookup-angle  (ros::get-param "~lookup_angle" -0.4)) ;; radian
       (setq param-change-body-duration (ros::get-param "~change_body_duration" 1.0))
       (setq param-bark-sound-file (pathname (ros::get-param "~bark_sound" "bark.wav")))
       ;; other param
       (let ((ros-img (one-shot-subscribe "~panorama_image" sensor_msgs::Image)))
         (setq paranoma-width (send ros-img :width))
         (setq paranoma-height (send ros-img :height)))
       ;; init other members
       (setq stand-start-time (ros::time-now))
       (setq yaw-offset 0)
       (setq luminance 0)
       (setq battery-temp 0)
       (setq ypr-euler '(0 0 0))
       ;; subscribe
       (ros::subscribe "~input" sensor_msgs::Image #'send self :image-cb)
       (ros::subscribe "~human" jsk_recognition_msgs::RectArray #'send self :detect-cb)
       (ros::subscribe "~battery_states" spot_msgs::BatteryStateArray #'send self :battery-states-cb)
       (ros::subscribe "~odom" nav_msgs::Odometry #'send self :odom-cb)
       ))

  (:image-cb (msg)
   (let ((img (instance grayscale-image :init
                        (send msg :width)
                        (send msg :height)
                        (copy-object (send msg :data)))))
     (dotimes (w (send img :width))
       (dotimes (h (send img :height))
         (setq luminance (+ luminance (send img :pixel w h)))))
     (setq luminance (/ luminance (* (send img :width) (send img :height))))
     (ros::ros-debug "receive ~A, luminance is ~A" (send (send msg :header) :frame_id) luminance)
     t)
   )

  (:detect-cb (msg)
   (setq human-rects (send msg :rects)))

  (:battery-states-cb  (msg)
   (setq battery-temp (apply #'max (coerce (send (elt (send msg :battery_states) 0) :temperatures) cons)))
   (ros::ros-debug "battery temp: ~A" battery-temp)
   )

  (:odom-cb (msg)
   (let ()
     (setq q (float-vector
              (send msg :pose :pose :orientation :w)
              (send msg :pose :pose :orientation :x)
              (send msg :pose :pose :orientation :y)
              (send msg :pose :pose :orientation :z)))
     (setq ypr-euler (elt (rpy-angle (quaternion2matrix q)) 0))
     (if (and
          (< (abs (elt (send *ri* :get-val 'ypr-euler) 2)) 0.05)
          (< (abs (elt (send *ri* :get-val 'ypr-euler) 1)) 0.05))
         (send *ri* :set-val 'yaw-offset (elt (send *ri* :get-val 'ypr-euler) 0)))
     (ros::ros-debug  "body quaternion is ~A, YPR euler is ~A ~%" q ypr-euler)
     ))

  (:bright-surrounding ()
   (if (> luminance param-threshold-luminance) t nil))

  (:ready-stand-temp ()
   (if (< battery-temp param-threshold-temp-stand)
       (progn
         t)
       (progn
         (ros::ros-info "bat temp: ~A; temp thresh: ~A" battery-temp param-threshold-temp-stand)
         nil)))

  (:high-battery-temp ()
   (if (> battery-temp param-threshold-temp-sit)
       (progn
         (ros::ros-info "bat temp: ~A; temp thresh: ~A" battery-temp param-threshold-temp-sit)
         t)
       nil))
  )
