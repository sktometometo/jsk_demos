
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
  )

(defun init-auto-integ-sim-env (robot-x robot-y robot-z robot-theta functions)

  (init-rooms)
  (setq *robot* (instance spot-robot :init))
  (send *robot* :translate (float-vector robot-x robot-y robot-z))
  (send *robot* :rotate robot-theta :z)
  (setq ans (list *robot* *room73b2* *room73a3* ))

  (dolist (f functions)
    (let ((device (make-sphere 500))
          (device-position (access-alist f "position"))
          (device-pos-x (* 1000 (elt device-position 0)))
          (device-pos-y (* 1000 (elt device-position 1)))
          (device-pos-z (* 1000 (elt device-position 2)))
          (device-name (access-alist f "name"))
          (device-description (access-alist f "description"))
          )
      (send device :set-color #f(1 0 0))
      (send device :translate (float-vector device-pos-x device-pos-y device-pos-z))
      (push device ans)
      ))
  ans)

(defun init-scene-spatial-device ()
  (init-rooms)

  (setq *human-01* (instance human-robot :init))
  (send *human-01* :translate #f(7000 2000 0))
  (send *human-01* :rotate -1.57 :z)
  (send *human-01* :rarm :inverse-kinematics (send (send (send *human-01* :rarm :end-coords) :copy-worldcoords) :translate #f(500 0 300)) :rotation-axis nil)

  (setq *human-02* (instance human-robot :init))
  (send *human-02* :translate #f(-5000 -1000 0))
  (send *human-02* :rotate 1.57 :z)

  (let ((rleg-coords (send *human-02* :rleg :end-coords :copy-worldcoords))
        (lleg-coords (send *human-02* :lleg :end-coords :copy-worldcoords))
        (rarm-coords
         (send (send *human-02*
                     :rarm :end-coords :copy-worldcoords)
               :translate #f(500 0 300)))
        (larm-coords
         (send (send *human-02*
                     :larm :end-coords :copy-worldcoords)
               :translate #f(500 0 300))))
    (send *human-02* :fullbody-inverse-kinematics 
                (list rleg-coords
                      lleg-coords
                      rarm-coords
                      larm-coords)
            :move-target
                (list (send *human-02* :rleg :end-coords)
                      (send *human-02* :lleg :end-coords)
                      (send *human-02* :rarm :end-coords)
                      (send *human-02* :larm :end-coords))
            :link-list
            (list (send *human-02* :link-list (send *human-02* :rleg :end-coords :parent))
                  (send *human-02* :link-list (send *human-02* :lleg :end-coords :parent))
                  (send *human-02* :link-list (send *human-02* :rarm :end-coords :parent))
                  (send *human-02* :link-list (send *human-02* :larm :end-coords :parent)))
            :translation-axis (list t t t t)
            :rotation-axis (list nil nil nil nil)
            :target-centroid-pos
              (midpoint 0.5
                (send (send (send *human-02* :rleg :end-coords :copy-worldcoords) :translate #f(-200 0 0)) :worldpos)
                (send (send (send *human-02* :lleg :end-coords :copy-worldcoords) :translate #f(-200 0 0)) :worldpos))
            :cog-translation-axis :z)
    )

  (setq *spot-collidor* (instance spot-robot :init))
  (send *spot-collidor* :translate #f(-1500 -3000 600))
  (send *spot-collidor* :rotate 1.57 :z)

  (setq *spot-73b2-01* (instance spot-robot :init))
  (send *spot-73b2-01* :translate #f(1500 0 600))
  (send *spot-73b2-01* :rotate 0 :z)

  (setq *spot-73b2-02* (instance spot-robot :init))
  (send *spot-73b2-02* :translate #f(7000 1000 600))
  (send *spot-73b2-02* :rotate 1.57 :z)

  (setq *spot-73b2-03* (instance spot-robot :init))
  (send *spot-73b2-03* :translate #f(1500 2500 600))
  (send *spot-73b2-03* :rotate 0 :z)

  (setq *spot-73b2-04* (instance spot-robot :init))
  (send *spot-73b2-04* :translate #f(4500 1000 600))
  (send *spot-73b2-04* :rotate 0 :z)
  )

(defun visualize-spatial-device ()
  (objects (list *room73b2* *room73a3*
                 *human-01* *human-02*
                 *spot-collidor*
                 *spot-73b2-01* *spot-73b2-02* *spot-73b2-03* *spot-73b2-04*))
  )

(defun init-scene-device-localization ()
  (init-rooms)

  (setq *human-01* (instance human-robot :init))
  (send *human-01* :translate #f(11000 1000 0))
  (send *human-01* :rotate 3.14 :z)
  (send *human-01* :rarm :inverse-kinematics (send (send (send *human-01* :rarm :end-coords) :copy-worldcoords) :translate #f(500 0 300)) :rotation-axis nil)

  (setq *spot-01* (instance spot-robot :init))
  (send *spot-01* :translate #f(10000 3000 600))
  (send *spot-01* :rotate 3.14 :z)

  (setq *spot-02* (instance spot-robot :init))
  (send *spot-02* :translate #f(8000 2500 600))
  (send *spot-02* :rotate -2.2 :z)
  (send *spot-02* :head :look-at (send (send (send *human-01* :head :end-coords) :copy-worldcoords) :pos))

  (setq *spot-03* (instance spot-robot :init))
  (send *spot-03* :translate #f(9500 1000 600))
  (send *spot-03* :rotate 0 :z)
  (send *spot-03* :head :look-at (send (send (send *human-01* :head :end-coords) :copy-worldcoords) :pos))

  (setq *spot-04* (instance spot-robot :init))
  (send *spot-04* :translate #f(8200 -1200 600))
  (send *spot-04* :rotate 3.8 :z)
  (send *spot-04* :arm :inverse-kinematics
        (send (send (send *spot-04* :arm :end-coords) :copy-worldcoords) :translate #f(0 0 300))
        )

  (setq *spot-05* (instance spot-robot :init))
  (send *spot-05* :translate #f(6000 -1000 600))
  (send *spot-05* :rotate 2.2 :z)
  (send *spot-05* :arm :inverse-kinematics
        (send (send (send *spot-05* :arm :end-coords) :copy-worldcoords) :translate #f(0 0 300))
        )

  (setq *spot-06* (instance spot-robot :init))
  (send *spot-06* :translate #f(4500 800 600))
  (send *spot-06* :rotate 0 :z)
  (send *spot-06* :arm :inverse-kinematics
        (send (send (send *spot-06* :arm :end-coords) :copy-worldcoords) :translate #f(0 0 300))
        )
  )

(defun visualize-device-localization ()
  (objects (list *room73b2*
                 *human-01*
                 *spot-01* *spot-02* *spot-03* *spot-04* *spot-05* *spot-06*))
  )

(defun run-spatial-device ()
  (init-scene-spatial-device)
  (visualize-spatial-device)
  )

(defun run-device-localization ()
  (init-scene-device-localization)
  (visualize-device-localization)
  )
