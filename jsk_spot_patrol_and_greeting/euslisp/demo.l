#!/usr/bin/env roseus


(defun main ()
  (ros::roseus "spot-patrol-and-greeting")
  (setq *with-kinova* (ros::get-param "~with_kinova" nil))
  (if *with-kinova*
      (progn
        (load "package://spotkinovaeus/spotkinova-interface.l")
        (spotkinova-init)
        (send *spotkinova* :kinova-rest-pose)
        (send *ri* :angle-vector (send *spotkinova* :angle-vector) 5000)
        (send *ri* :wait-interpolation))
      (progn
        (load "package://spoteus/spot-interface.l")
        (spot-init)))

  (setq patrol-list (ros::get-param "~patrol_list" nil))
  (setq home-id (ros::get-param "~home_id" "eng2_73B2"))
  (setq dock-id (ros::get-param "~dock_id" 520))
  (setq duration-spot-stay (ros::get-param "~duration_spot_stay" 30))

  ;; undock
  (send *ri* :undock)

  (dolist (target-id patrol-list)
    (ros::ros-info "Going to ~A" target-id)
    (setq ret (send *ri* :go-to-spot target-id))
    (if ret
        (unix::sleep duration-spot-stay)
      (ros::ros-warn "Failed to reach ~A" target-id))
    )

  ;; go back to home and dock
  (send *ri* :go-to-spot home-id)
  (send *ri* :dock dock-id)
  )

(main)
(exit)
