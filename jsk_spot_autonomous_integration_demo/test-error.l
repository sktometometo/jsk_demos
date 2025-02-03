(defun test-error ()
  (let* ((original-str "1234 5678")
         (strm (make-string-input-stream original-str))
         )
    (loop
      (let ((c (peek-char strm)))
        (return (read-from-string (concatenate string "" (vector c))))))))

(setq ans (test-error))
(format t "ans: ~A~%" ans)
