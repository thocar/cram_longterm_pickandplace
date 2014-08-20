(defun test-subscriber (topic type)
  (roslisp:subscribe
   topic type
   (lambda (msg)
     (format t "In~%")
     (roslisp:with-fields (name values) msg
       (format t "~a: ~a~%" name values)))))
