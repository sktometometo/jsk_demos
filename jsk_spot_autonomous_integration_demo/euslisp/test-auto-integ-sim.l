#!/usr/bin/env roseus

(require "package://jsk_spot_autonomous_integration_demo/euslisp/json-loader.l")
(require "package://jsk_spot_autonomous_integration_demo/euslisp/env.l")


(defun load-sim-param (param-file)
    (let* ((param (parse-json-file param-file))
           (functions (access-alist param "functions"))
           (conditions (access-alist param "conditions")))
        (format t "param: ~A~%" param)
        (cons functions conditions)))

(defun show-env (functions robot-pos robot-direction)
    (let* ((robot-pos-x (* 1000 (elt robot-pos 0)))
           (robot-pos-y (* 1000 (elt robot-pos 1)))
           (robot-pos-z (* 1000 (elt robot-pos 2)))
           (robot-pos-theta robot-direction)
           (objs (init-auto-integ-sim-env robot-pos-x robot-pos-y robot-pos-z robot-pos-theta functions))
           )
        (format t "objs: ~A~%" objs)
        (objects objs)
        ))

(defun main ()
    (let* ((test-json-path
             (format nil "~A/test_data/auto_integ_sim.json"
                 (ros::rospack-find "jsk_spot_autonomous_integration_demo")))
           (params (load-sim-param test-json-path))
              (functions (car params))
                (conditions (cdr params))
                (condition (elt conditions 0))
            )
        (show-env functions (access-alist condition "robot_position") (access-alist condition "robot_direction"))
        ))

(main)
