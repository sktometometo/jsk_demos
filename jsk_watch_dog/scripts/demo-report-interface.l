(require "package://image_view2/euslisp/image-capture-utils.l")
(require "package://gdrive_ros/euslisp/gdrive-ros-utils.l")
(require "package://jsk_robot_startup/euslisp/email-topic-client.l")

(defclass demo-report-interface
    ""
  :super propertied-object
  :slots (list-upload-queue
          image-directory
          gdrive-parents-path))

(defmethod demo-report-interface
    ""
  (:init (&rest args)
   (prog1
       (send-super* :init args)
     ;; rosparam
     (setq gdrive-parents-path (ros::get-param "~gdrive_parents_path"))
     (setq image-directory (ros::get-param "~image_directory" "/tmp"))
     (setq report-mail-address (ros::get-param "~report_mail_address"))

     (setq list-upload-queue (list))
     (init-mail)
     ))

  (:capture-image (image-topic image-directory image-base-name)
   (let ((image-file-path (format nil "~A/~A" image-directory image-base-name)))
     (ros::ros-debug "Capture a image to ~A" image-file-path)
     (capture-image image-topic image-file-path)
     (setq list-upload-queue (append list-upload-queue (list (list image-file-path image-base-name))))
     ))

  (:upload-images (mail-title mail-body-header)
   (let (res
         (mail-body (format nil "~A ~%" mail-body-header)))
     (dolist (upload-item list-upload-queue)
       (ros::ros-debug "Upload file ~A" (elt upload-item 0))
       (setq res (upload-file (elt upload-item 0) (elt upload-item 1) :parents-path gdrive-parents-path))
       (if (elt res 0)
           (setq mail-body (concatenate string mail-body (format nil "~A : ~A ~%" (elt upload-item 1) (elt res 2))))
           nil)
       )
     (if list-upload-queue
         (send-mail mail-title
                    report-mail-address
                    mail-body)
         nil)
     (setq list-upload-queue nil)
     (ros::ros-debug "Sent a mail to ~A : ~A" report-mail-address mail-body)
     )
   )
  )
