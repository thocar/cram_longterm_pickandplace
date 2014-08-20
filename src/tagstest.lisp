(def-top-level-cram-function navigate-to-detect-object ()
  (let ((close-enough-to-see (cpl:make-fluent :name "perception result"
                                              :allow-tracing nil
                                              :value nil)))
    (with-tags
      (cpl:partial-order
          ((:tag navigate
             (format t "The navigation process~%")
             (sleep 2)
             (cpl:pulse close-enough-to-see))
           (:tag detect
             (format t "The perception process~%")))
        (:order close-enough-to-see detect)))))


(def-top-level-cram-function navigate-to-detect-object ()
  (let ((close-enough-to-see (cpl:make-fluent :name "perception result"))
        (navigation-failed (cpl:make-fluent :name "navigation failed"))
        (object-found (cpl:make-fluent :name "object found"))
        (object-not-found (cpl:make-fluent :name "object not found")))
    (with-tags
      (cpl:partial-order
          ((:tag navigate
             (with-failure-handling
                 ((location-not-reached (f)
                    (cpl:pulse navigation-failed)
                    (return)))
               (perform navigation))
             (cpl:pulse close-enough-to-see))
           (:tag detect
             (let ((objects (perform perception)))
               (if objects
                   (cpl:pulse object-found)
                   (cpl:pulse object-not-found)))))
        (:order close-enough-to-see detect)))))
