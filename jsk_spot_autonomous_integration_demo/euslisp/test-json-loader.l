#!/usr/bin/env roseus

(load "package://jsk_spot_autonomous_integration_demo/euslisp/json-loader.l")

(setq json-obj (parse-json-file "./test_data/test2.json"))
(format t "Loaded results obj: ~A~%" json-obj)

;; Access test
;;; Access alist
(format t
        "obj[\"abc\"]: ~A~%"
        (cdr (assoc "abc" json-obj :test 'string-equal)))
;;; Access vector
(format
  t
  "obj[\"abc\"][\"bef\"][1]: ~A~%"
  (elt
      (cdr (assoc "def"
                  (cdr (assoc "abc" json-obj :test 'string-equal))
                  :test 'string-equal))
      1))

