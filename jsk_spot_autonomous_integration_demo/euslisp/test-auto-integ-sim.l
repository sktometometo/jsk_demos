#!/usr/bin/env roseus

(require "package://jsk_spot_autonomous_integration_demo/euslisp/json-loader.l")
(require "package://jsk_spot_autonomous_integration_demo/euslisp/env.l")


(defun load-sim-param (param-file)
    (let* ((param (parse-json-file param-file))
           (functions (access-alist param "functions"))
           (conditions (access-alist param "conditions")))
        (format t "param: ~A~%" param)
        (cons functions conditions)))

(defun show-env (conditions functions)
    (let* (
           (objs (init-auto-integ-sim-env conditions functions))
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
           )
        (show-env conditions functions)
        ;(setq gl::*perspective-far* 50000.0)
        ;(setq gl::*perspective-near* 0.1)
        ))

(main)
