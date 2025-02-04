
(require "package://spoteus/spot-utils.l")

;; https://github.com/euslisp/EusLisp/blob/master/models/room73b2-scene.l
(require "models/room73b2-scene.l")
(require "models/room73a3-scene.l")
(require "models/human-robot.l")
(require "models/eng2-7f-southwing-scene.l")
(require "models/load-irt-all-scenes.l")

(load "package://jsk_spot_autonomous_integration_demo/euslisp/json-loader.l")

(load "package://spoteus/spot.l")
(load "package://pr2eus/pr2.l")


(defun init-rooms ()
  (room73b2)
  (room73a3)
  (send *room73a3* :translate #f(-3000 0 0) :local)
  (send *room73a3* :rotate 3.14 :z)
  (setq *floor-collidor* (make-cube 3000 10000 10 :color #f(0.2 0.2 0.2)))
  (send *floor-collidor* :translate #f(-1500 -500 -5))
  (setq *floor-wall-end* (make-cube 3000 300 2600 :color :white))
  (send *floor-wall-end* :translate #f(-1500 4500 1300))
  (setq *floor-wall-side* (make-cube 100 4500 2600 :color :white))
  (send *floor-wall-side* :translate #f(-3000 2200 1300))
  (list *room73b2* *room73a3* *floor-collidor* *floor-wall-end* *floor-wall-side*))

(defun init-auto-integ-sim-env (robot-x robot-y robot-z robot-theta functions)

  (setq ans (init-rooms))
  (setq *robot* (instance spot-robot :init))
  (send *robot* :translate (float-vector robot-x robot-y robot-z))
  (send *robot* :rotate robot-theta :z)
  (push *robot* ans)

  (format t "functions: ~A~%" functions)
  (dotimes (i (length functions))
    (let* ((f (elt functions i))
           (device (make-sphere 200))
           (device-position (access-alist f "position"))
           (device-name (access-alist f "name"))
           (device-description (access-alist f "description"))
           (device-pos-x (* 1000 (elt device-position 0)))
           (device-pos-y (* 1000 (elt device-position 1)))
           (device-pos-z (* 1000 (elt device-position 2)))
           )
      (format t "f: ~A~%" f)
      (send device :set-color #f(1.0 1.0 0.2))
      (send device :translate (float-vector device-pos-x device-pos-y device-pos-z))
      (push device ans)
      ))
  ans)

