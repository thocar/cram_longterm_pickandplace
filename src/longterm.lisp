;;; Copyright (c) 2014, Jan Winkler <winkler@cs.uni-bremen.de>
;;; All rights reserved.
;;;
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;;
;;; * Redistributions of source code must retain the above copyright
;;; notice, this list of conditions and the following disclaimer.
;;; * Redistributions in binary form must reproduce the above copyright
;;; notice, this list of conditions and the following disclaimer in the
;;; documentation and/or other materials provided with the distribution.
;;; * Neither the name of the Institute for Artificial Intelligence/
;;; Universitaet Bremen nor the names of its contributors may be used to 
;;; endorse or promote products derived from this software without specific 
;;; prior written permission.
;;;
;;; THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
;;; AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
;;; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
;;; ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
;;; LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
;;; CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
;;; SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
;;; INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
;;; CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
;;; ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
;;; POSSIBILITY OF SUCH DAMAGE.

(in-package :cram-longterm-pickandplace)

(defvar *loc-on-kitchen-island* nil)
(defvar *loc-on-sink-block* nil)
(defvar *loc-on-cupboard* nil)
(defvar *pancake-mix* nil)

(def-top-level-cram-function longterm (&optional (runs 1))
  (with-process-modules
    (loop for i from 0 below runs
          do (ensure-arms-up)
             (cpl:with-failure-handling
                 (((or cram-plan-failures:object-not-found
                       cram-plan-failures:manipulation-failure
                       cram-plan-failures:location-not-reached-failure) (f)
                    (declare (ignore f))
                    (cpl:retry)))
               (beliefstate:enable-logging nil)
               (prepare-settings)
               (beliefstate:enable-logging t)
               (let* ((possible-putdown-locations
                        `(,*loc-on-sink-block*
                          ,*loc-on-kitchen-island*))
                      (object *pancake-mix*)
                      (loc-target
                        (nth (random
                              (length possible-putdown-locations))
                             possible-putdown-locations)))
                 ;(perceive-scene)
                 (pick-object object)
                 (place-object object loc-target))))))

(def-top-level-cram-function table-setting ()
  (with-process-modules
    (ensure-arms-up)
    (cpl:with-failure-handling
        (((or cram-plan-failures:object-not-found
              cram-plan-failures:manipulation-failure
              cram-plan-failures:location-not-reached-failure) (f)
           (declare (ignore f))
           (cpl:retry)))
      (beliefstate:enable-logging nil)
      (prepare-settings)
      (beliefstate:enable-logging t)
      (with-designators
          ((plate (object `((desig-props::type
                             desig-props::dinnerplate)
                            (desig-props::at
                             ,*loc-on-sink-block*))))
           (milkbox (object `((desig-props::type
                               desig-props::milkbox)
                              (desig-props::at
                             ,*loc-on-sink-block*)))))
        (pick-object milkbox)
        (place-object milkbox *loc-on-kitchen-island*)
        (pick-object plate)
        (place-object plate *loc-on-kitchen-island*)))))

(defun publish-pose (pose &optional (topic "/object"))
  (let ((adv (roslisp:advertise topic "geometry_msgs/PoseStamped")))
    (roslisp:publish adv (tf:pose-stamped->msg pose))))

(defun prepare-settings ()
  ;; NOTE(winkler): This validator breaks IK based `to reach' and `to
  ;; see' location resolution. Disabling it, since everything works
  ;; just nicely without it. Gotta look into this later.
  (setf designators-ros::*fixed-frame* "/map") ;; CHANGED THESE FROM /map TO /odom_combined
  (setf location-costmap::*fixed-frame* "/map")
  (cram-designators:disable-location-validation-function
   'bullet-reasoning-designators::check-ik-solution)
  ;(cram-designators:disable-location-validation-function
  ; 'spatial-relations-costmap::potential-field-costmap-pose-function)
  ;(cram-designators:disable-location-validation-function
  ; 'spatial-relations-costmap::collision-pose-validator)
  (cram-designators:disable-location-validation-function
   'bullet-reasoning-designators::validate-designator-solution)
  (cram-uima::config-uima)
  ;; Setting the timeout for action server responses to a high
  ;; value. Otherwise, the (very long, > 2.0 seconds) motion planning
  ;; process will just drop the connection and never execute.
  (setf actionlib::*action-server-timeout* 20)
  ;(beliefstate::enable-logging t)
  (init-ms-belief-state :debug-window t)
  (setf btr::*bb-comparison-validity-threshold* 0.1)
  (moveit:clear-collision-environment)
  ;; Twice, because sometimes a ROS message for an object gets lost.
  (sem-map-coll-env:publish-semantic-map-collision-objects)
  (sem-map-coll-env:publish-semantic-map-collision-objects)
  (setf *loc-on-kitchen-island*
        (make-designator
         'location
         `((desig-props:on Cupboard)
           (desig-props:name "kitchen_island"))))
  (setf *loc-on-sink-block*
        (make-designator
         'location
         `((desig-props:on Cupboard)
           (desig-props:name "kitchen_sink_block"))))
  (setf *loc-on-cupboard*
        (make-designator
         'location
         `((desig-props:on Cupboard))))
  (setf *pancake-mix*
        (make-designator
         'object
         `((desig-props:at ,*loc-on-kitchen-island*);,*loc-on-cupboard*);,*loc-on-sink-block*)
           (desig-props::type desig-props::pancakemix)
           (desig-props::max-handles 1)
           ,@(mapcar
              (lambda (handle-object)
                `(desig-props:handle ,handle-object))
              (make-handles
               0.04
               :segments 2
               :offset-angle (/ pi 2)
               :ax (/ pi 2)
               :center-offset
               (tf:make-3d-vector 0.02 0.0 0.0)))))))

(def-top-level-cram-function model-tasks ()
  (cpl:with-retry-counters ((manip-cnt 5)
                            (loc-cnt 3))
    (cpl:with-failure-handling
        ((cram-plan-failures:manipulation-failure (f)
           (declare (ignore f))
           (ros-info
            (model-tasks) "Top-level manipulation failure")
           ;(cpl:do-retry manip-cnt
             (cpl:retry))
         (cram-plan-failures:location-not-reached-failure (f)
           (declare (ignore f))
           (ros-info
            (model-tasks) "Top-level location failure")
           ;(cpl:do-retry loc-cnt
             (cpl:retry)))
      (with-designators ((param-1 (action `()))
                         (param-2 (object `()))
                         (param-3 (location `())))
        (declare (ignore param-1))
        (when (random 2)
          (subtask-1 param-2 param-3))
        (subtask-2 param-2)))))

(def-cram-function subtask-1 (object location))

(def-cram-function subtask-2 (object)
  (beliefstate::annotate-parameters `((navigate-to-x ,(random 10))
                                      (navigate-to-y ,(random 10))
                                      (navigate-to-z ,(random 10))))
  (cpl:with-retry-counters ((loc-cnt 4))
    (cpl:with-failure-handling
        ((cram-plan-failures:location-not-reached-failure (f)
           (declare (ignore f))
           (ros-info
            (model-tasks) "Subtask 2 location failure")
           (cpl:do-retry loc-cnt
             (cpl:retry))))
      (let ((d-val (random 4)))
        (case d-val
          (0 (subtask-4 object nil))
          (1 (subtask-3 nil))
          (2 (cpl:fail 'cram-plan-failures:location-not-reached-failure))
          (3 (cpl:fail 'cram-plan-failures:manipulation-failure)))))))

(def-cram-function subtask-extra ()
  )

(def-cram-function subtask-3 (location)
  ;(subtask-extra)
  (let ((d-val (random 2)))
    (case d-val
      (0 (cpl:fail 'cram-plan-failures:location-not-reached-failure)))))

(def-cram-function subtask-4 (object location)
  (let ((d-val (random 3)))
    (case d-val
      (0 (cpl:fail 'cram-plan-failures:location-not-reached-failure))
      (1 (cpl:fail 'cram-plan-failures:manipulation-failure)))))

(defun start-model-tasks (&optional (amount 1))
  (loop for i from 0 below amount
        do (cpl:with-failure-handling
               (((or cram-plan-failures:manipulation-failure
                     cram-plan-failures:location-not-reached-failure) (f)
                  (declare (ignore f))
                  (cpl:retry)))
             (format t "Starting model task ~a~%" i)
             (model-tasks))))

(def-top-level-cram-function linear-model-task ()
  (with-designators ((action (action `())))
    (linear-model-task-1))
  (linear-model-task-2))

(def-cram-function linear-model-task-1 ()
  (beliefstate::annotate-parameters `((navigate-to-x ,(random 10))
                                      (navigate-to-y ,(random 10))
                                      (navigate-to-z ,(random 10))))
  (linear-model-task-2))

(def-cram-function linear-model-task-2 ()
  (with-designators ((action (action `())))))

(def-top-level-cram-function empty-one ()
  )

(defparameter *obj* nil)

(defun record-grasps ()
  (let ((gripper-frame "r_wrist_roll_link")
        (gripper-poses nil))
    (unless *obj*
      (setf *obj* (see-object nil)))
    (block ret
      (format t "Enter 'q' when you're done, enter to continue.~%")
      (loop while t
            do (let ((val (read-line)))
                 (when (string= val "q")
                   (return-from ret))
                 (let ((gripper-pose
                         (moveit:ensure-pose-stamped-transformed
                          (tf:pose->pose-stamped
                           gripper-frame
                           (roslisp:ros-time)
                           (tf:make-identity-pose))
                          "/map" :ros-time t)))
                   (push
                    gripper-pose
                    gripper-poses)))))
    gripper-poses))

(defvar *rob-loc* nil)

(defmacro let*-while (vars prediction-parameters failures max-probability predict
                     &body body)
  `(let ((first-run t)
         (prediction-fail nil)
         (return-val nil))
     (loop while (or first-run prediction-fail)
           do (when prediction-fail
                (format t "Restart due to prediction.~%"))
              (setf first-run nil)
              (let* ,vars
                (cond (,predict
                       (let* ((prediction (first (beliefstate::predict
                                                  ,prediction-parameters)))
                              (pred-fails (desig-prop-value prediction
                                                            'desig-props::failures))
                              (pred-vals
                                (mapcar
                                 (lambda (fail)
                                   (cadr (find
                                          fail pred-fails
                                          :test
                                          (lambda (x y)
                                            (eql x (intern
                                                    (symbol-name
                                                     (car y))
                                                    'cram-plan-failures))))))
                                 ,failures)))
                         (setf prediction-fail t)
                         (when (or (and pred-vals
                                        (reduce
                                         #'(lambda (x y)
                                             (and x y))
                                         (mapcar
                                          (lambda (x)
                                            (or (not x) (<= x ,max-probability)))
                                          pred-vals)))
                                   (not pred-vals))
                           (setf prediction-fail nil)
                           (setf return-val
                                 (progn ,@body)))))
                      (t (setf return-val (progn ,@body))))))
     return-val))

(def-top-level-cram-function model-plan ()
  (setf *rob-loc* (make-designator 'location `((desig-props::x 0)
                                               (desig-props::y 0))))
  (with-designators ((obj (object `((desig-props::x ,(random 6))
                                    (desig-props::y ,(random 6))))))
    (cpl:with-failure-handling
        ((cram-plan-failures:manipulation-failure (f)
           (declare (ignore f))
           (ros-info
            (model-plan) "Top-level manipulation failure")
           (cpl:retry))
         (cram-plan-failures:location-not-reached-failure (f)
           (declare (ignore f))
           (ros-info
            (model-plan) "Top-level location failure")
           (cpl:retry))
         (cram-plan-failures:object-not-found (f)
           (declare (ignore f))
           (ros-info
            (model-plan) "Top-level object not found failure")
           (cpl:retry)))
      (sleep 0.05)
      (perform-plan-tasks obj))))

(def-cram-function perform-plan-tasks (obj)
  (with-designators ((loc (location `((desig-props::x ,(random 10))
                                      (desig-props::y ,(random 10))))))
    ;; (let* ((loc (make-designator 'location
    ;;                              `((desig-props::x ,(random 10))
    ;;                                (desig-props::y ,(random 10)))))
    ;;        (x (desig-prop-value loc 'desig-props::x))
    ;;        (y (desig-prop-value loc 'desig-props::y)))
      (let*-while ((loc (make-designator 'location
                                         `((desig-props::x ,(random 10))
                                           (desig-props::y ,(random 10)))))
                   (x (desig-prop-value loc 'desig-props::x))
                   (y (desig-prop-value loc 'desig-props::y)))
          `((navigate-to-x ,x)
            (navigate-to-y ,y))
          `(cram-plan-failures:location-not-reached-failure
            cram-plan-failures:manipulation-failure
            cram-plan-failures:object-not-found)
          0.5 t
      (go-to x y loc)
      (find-obj obj)
      (grasp obj))))

(def-cram-function go-to (x y loc)
  (cpl:with-retry-counters ((loc-retry 2))
    (cpl:with-failure-handling
        ((cram-plan-failures:location-not-reached-failure (f)
           (declare (ignore f))
           (cpl:do-retry loc-retry
             (cpl:retry))))
      (beliefstate::annotate-parameters `((navigate-to-x ,x)
                                          (navigate-to-y ,y)))
      (unless (and (< x 8)
                   (< y 6))
        (cpl:fail 'cram-plan-failures:location-not-reached-failure))
      (setf *rob-loc* loc))))

(def-cram-function find-obj (obj)
  (let* ((x-obj (desig-prop-value obj 'desig-props::x))
         (y-obj (desig-prop-value obj 'desig-props::y))
         (x-rob (desig-prop-value *rob-loc* 'desig-props::x))
         (y-rob (desig-prop-value *rob-loc* 'desig-props::y))
         (obj-dist (tf:v-dist (tf:make-3d-vector x-obj y-obj 0.0)
                              (tf:make-3d-vector x-rob y-rob 0.0))))
    (beliefstate::annotate-parameters `((obj-dist ,obj-dist)
                                        (navigate-to-x ,x-rob)
                                        (navigate-to-y ,y-rob)))
    (unless (< obj-dist 5)
      (cpl:fail 'cram-plan-failures:object-not-found))))

(def-cram-function grasp (obj)
  (let* ((x-obj (desig-prop-value obj 'desig-props::x))
         (y-obj (desig-prop-value obj 'desig-props::y))
         (x-rob (desig-prop-value *rob-loc* 'desig-props::x))
         (y-rob (desig-prop-value *rob-loc* 'desig-props::y))
         (obj-dist (tf:v-dist (tf:make-3d-vector x-obj y-obj 0.0)
                              (tf:make-3d-vector x-rob y-rob 0.0))))
    (beliefstate::annotate-parameters `((obj-dist ,obj-dist)
                                        (navigate-to-x ,x-rob)
                                        (navigate-to-y ,y-rob)))
    (unless (<= obj-dist 3)
      (cpl:fail 'cram-plan-failures:manipulation-failure))))

(defun predict (parameters failures)
  (let* ((prediction (first (beliefstate::predict parameters)))
         (failures-returned (desig-prop-value prediction
                                              'desig-props::failures))
         (failures-interned
           (cond ((listp failures-returned)
                  (mapcar (lambda (failure-returned)
                            `(,(intern (symbol-name
                                        (car failure-returned))
                                       'cram-plan-failures)
                              ,(cdr failure-returned)))
                          failures-returned))
                 (t nil))))
    (mapcar (lambda (failure)
              (let ((failure-value
                      (caadr (find failure failures-interned
                                   :test (lambda (needle stray)
                                           (eql needle (car stray)))))))
                (or failure-value 0.0d0)))
            failures)))

(defmacro let*-predict (definitions failure-tolerances tries
                        &body body)
  `(let ((tried 0))
     (tagbody prediction-beginning
        (let* ,(mapcar (lambda (definition)
                         (destructuring-bind
                             (symbol value annotation)
                             definition
                           (declare (ignore annotation))
                           `(,symbol ,value)))
                definitions)
          (let ((prediction-results
                  (predict
                   (list ,@(mapcar
                            (lambda (definition)
                              (destructuring-bind
                                  (symbol value annotation)
                                  definition
                                (declare (ignore value))
                                `(list ',annotation ,symbol)))
                            definitions))
                   ',(mapcar
                      (lambda (failure)
                        (car failure))
                      failure-tolerances))))
            (when (find t (mapcar (lambda (result tolerance)
                                    (> result (cadr tolerance)))
                                  prediction-results
                                  ',failure-tolerances))
              (when (> ,tries 0)
                (incf tried))
              (when (or (= ,tries 0)
                        (< tried ,tries))
                (go prediction-beginning))
              ,@body))))))

(defun run-experiments (&key (amount 10))
  (loop for i from 0 below amount
        do (model-plan)
           (sleep 0.1)
           (beliefstate:extract-files)
           (sleep 0.1)
           (beliefstate::start-new-experiment)
           (sleep 3.0)))

(def-top-level-cram-function artif-wfh-case ()
  (let ((failed nil))
    (cpl:with-failure-handling
        ((cram-plan-failures:manipulation-failed (f)
           (declare (ignore f))
           (setf failed t)
           (cpl:retry)))
      (unless failed
        (cpl:with-failure-handling
            ((cram-plan-failures:manipulation-failed (f)
               (declare (ignore f))))
          (with-designators ((act (action `())))
            (format t "~a~%" act)
            (cpl:fail 'cram-plan-failures:manipulation-failed)))))))

(defun avoid-collision-environment-validator (desig pose)
  (let ((to (desig-prop-value desig 'desig-props:to)))
    (if (or (eql to 'desig-props:see)
            (eql to 'desig-props:reach))
        (if (moveit::check-base-pose-validity
             (tf:pose->pose-stamped
              "map" 0.0 pose))
            (prog1 :accept (format t "I say yes~%"))
            (prog1 :reject (format t "I say no~%")))
        (prog1 :unknown (format t "I say unknown~%")))))

(defun make-access-restriction-cost-function ()
  (let ((min-x -0.9)
        (min-y -0.95)
        (max-x  1.5)
        (max-y  1.4))
    (lambda (x y)
      (if (and (>= x min-x)
               (<= x max-x)
               (>= y min-y)
               (<= y max-y))
          1.0d0
          0.0d0))))

(defun make-area-restriction-cost-function ()
  (let ((min-x -0.9)
        (min-y -0.95)
        (max-x  1.5)
        (max-y  1.7)
        (sink-block-min-y 0.0)
        (sink-block-max-y 1.0))
    (lambda (x y)
      (if (> x max-x)
          0.0d0 ;; Invalid due to too large x
          (if (and (> x 0.0)
                   (> y min-y)
                   (< y sink-block-max-y)
                   (> y sink-block-min-y))
              1.0d0 ;; Valid region on sink block
              (if (and (<= x 0.0) ;; Other side of kitchen
                       (> x min-x)
                       (> y min-y)
                       (< y max-y))
                  (if (> y 0.4) ;; On kitchen island side
                      (if (and (< x -0.6) ;; Lower boundary for kitchen island
                               (> x -0.7))
                          0.0d0
                          (if (and (> y 0.65)
                                   (< y 2.5))
                              1.0d0
                              0.0d0))
                      (if (and (< x -0.6) ;; Lower boundary for pancake table
                               (> x -0.95))
                          0.0d0
                          1.0d0))
                  0.0d0))))))

(defmethod costmap-generator-name->score ((name (common-lisp:eql 'area-restriction-distribution)))
  107)

(defmethod costmap-generator-name->score ((name (common-lisp:eql 'access-restriction-distribution)))
  108)

(def-fact-group demo-costmap-desigs (desig-costmap)

  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (desig-props:on ?_))
    (costmap ?cm)
    (costmap-add-function area-restriction-distribution
                          (make-area-restriction-cost-function)
                          ?cm))

  (<- (desig-costmap ?desig ?cm)
    (or (desig-prop ?desig (desig-props:to desig-props:see))
        (desig-prop ?desig (desig-props:to desig-props:reach)))
    (costmap ?cm)
    (costmap-add-function access-restriction-distribution
                          (make-access-restriction-cost-function)
                          ?cm)))

(def-top-level-cram-function choose-test (&optional (predict-outcome t))
  (setf beliefstate::*enable-prediction* predict-outcome)
  (beliefstate:enable-logging t)
  (with-process-modules
    (with-designators ((loc (location `((desig-props::on Cupboard)
                                        (desig-props::name "kitchen_island"))))
                       (obj-1 (object `((desig-props:at ,loc)
                                        (desig-props::type
                                         desig-props::pancakemix))))
                       (obj-2 (object `((desig-props:at ,loc)
                                        (desig-props::type
                                         desig-props::spatula))))
                       (obj-3 (object `((desig-props:at ,loc)
                                        (desig-props::type
                                         desig-props::dinnerplate))))
                       (obj-4 (object `((desig-props:at ,loc)
                                        (desig-props::type
                                         desig-props::ketchup)))))
      (let ((i 0)
            (steps-taken 0)
            (objects `(,obj-1 ,obj-2 ,obj-3 ,obj-4)))
        (loop while (< i (* 25 (length objects)))
              do (cpl:with-failure-handling
                     ((cram-plan-failures:object-not-found (f)
                        (declare (ignore f))
                        (format t "Failure: Object not found~%")
                        (cpl:retry)))
                   (format t "Step: ~a~%" i)
                   (incf i)
                   (incf steps-taken)
                   (find-object (nth (mod i (length objects)) objects)
                                "kitchen_island")))
        steps-taken))))

(def-cram-function find-object (obj area)
  (let ((loc (desig-prop-value obj 'desig-props:at)))
    (cut:choose
     test-find-object
     :generators (((looking-at-pose)
                   `(,(reference
                       (cram-plan-library::next-solution
                        (desig:current-desig loc)))))
                  ((area) `(,area))
                  ((robot-pose) `(,(moveit:ensure-pose-stamped-transformed
                                    (tf:make-pose-stamped
                                     "/base_footprint" 0.0
                                     (tf:make-identity-vector)
                                     (tf:make-identity-rotation))
                                    "/map"))))
     :features ((look-x (tf:x (tf:origin looking-at-pose)))
                (look-y (tf:y (tf:origin looking-at-pose)))
                (obj-type (desig-prop-value obj 'desig-props::type)))
     :constraints ((cram-plan-failures::objectnotfound
                    (or (< cut::predicted-failure 0.5))))
     :predicting (time)
     :attempts 10
     :body
     (progn
       (achieve `(cram-plan-library:looking-at ,looking-at-pose))
       (let ((found-objects
               (cram-plan-library:perceive-object
                'cram-plan-library:currently-visible
                obj)))
         (unless found-objects
           (cpl:fail 'cram-plan-failures:object-not-found))
         found-objects)))))

(def-top-level-cram-function choose-test-offline (&optional (predict t) (trials 100))
  (beliefstate:enable-logging t)
  (setf cram-prediction::*enable-prediction* predict)
  (with-process-modules
    (with-designators ((loc (location `((desig-props::pose
                                         ,(tf:make-pose-stamped
                                           "/map" 0.0
                                           (tf:make-3d-vector 1 2 3)
                                           (tf:make-identity-rotation)))))))
      (let ((i 0))
        (loop while (< i trials)
              do (cpl:with-failure-handling
                     ((cram-plan-failures:location-not-reached-failure (f)
                        (declare (ignore f))
                        (format t "Failure: Location not reached~%")
                        (cpl:retry))
                      (cram-plan-failures:object-not-found (f)
                        (declare (ignore f))
                        (format t "Failure: Object not found~%")
                        (cpl:retry)))
                   (format t "Step: ~a~%" i)
                   (incf i)
                   (navigate-to)))))))

(def-cram-function navigate-to ()
  (cut:choose
   navigate
   :features ((nav-x (/ (random 100) 10.0))
              (nav-y (/ (random 100) 10.0)))
   :constraints ((desig-props::locationnotreached
                  (or (not cut::predicted-failure)
                      (< cut::predicted-failure
                         0.5))))
   :predicting (time)
   :attempts 2
   :body
   (progn
     (unless (or (< 2 nav-x 8)
                 (< 3 nav-y 9))
       (cpl:fail
        'cram-plan-failures:location-not-reached-failure))
     (look-at nav-x nav-y))))

(def-cram-function look-at (nav-x nav-y)
  (prediction:choose
   look-at
   :features ((look-x (/ (random 100) 10.0))
              (look-y (/ (random 100) 10.0))
              (distance (tf:v-dist (tf:make-3d-vector nav-x nav-y 0)
                                   (tf:make-3d-vector look-x look-y 0))))
   :constraints ((desig-props::objectnotfound
                  (progn
                    (format t "~a~%" cut::predicted-failure)
                    (or (not cut::predicted-failure)
                        (< cut::predicted-failure
                           0.5)))))
   :predicting (time)
   :attempts 2
   :body
   (unless (< distance 5)
     (cpl:fail 'cram-plan-failures:object-not-found))))

(def-top-level-cram-function test-series ()
  (loop for j from 0 below 50
        do (with-designators ()
             (with-designators ()
               (with-designators ()
                 (loop for i from 0 below 10
                       do (with-designators ()
                            (with-designators ()
                              (with-designators ()
                                )))))))))

(defun run-experiment (&key (steps 10) (trials-step 2))
  (loop for i from 0 below steps
        do (progn
             (format t "Unpredicted run. Press Enter to continue.~%")
             (read-line)
             (sample-data-evaluation :predict nil :trials (* (1+ i) trials-step))
             (beliefstate:set-metadata
              :experiment "Unpredicted Sample Data Recording")
             (beliefstate:extract-files)
             (format t "Predicted run. Press Enter to continue.~%")
             (read-line)
             (prediction::load-decision-tree "/home/winkler/dtree-model.json")
             (prediction::load-model "/home/winkler/task-model.json")
             (sample-data-evaluation :predict t :trials (* (1+ i) trials-step))
             (beliefstate:set-metadata
              :experiment "Predicted Sample Data Recording")
             (beliefstate:extract-files))))

(def-top-level-cram-function sample-data-evaluation (&key (predict t)
                                                          (trials 100))
  (beliefstate:enable-logging t)
  (setf cram-prediction::*enable-prediction* predict)
  (let ((i trials))
    (block nil
      (loop while (> (decf i) 0)
            do (format t "Step ~a / ~a~%" i trials)
               (cpl:with-failure-handling
                   ((cram-plan-failures::simple-plan-failure (f)
                      (declare (ignore f))
                      (when (< i trials) (return nil))
                      (cpl:retry)))
                 (sample-pick))
               (cpl:with-failure-handling
                   ((cram-plan-failures::simple-plan-failure (f)
                      (declare (ignore f))
                      (when (< i trials) (return nil))
                      (cpl:retry)))
                 (sample-place))))))

(def-cram-function sample-pick ()
  (:tag pick
    (cpl:with-retry-counters ((cnt-perc 3)
                              (cnt-nav 3)
                              (cnt-grasp 3))
      (let ((obj-position
              (cpl:with-failure-handling
                  ((cram-plan-failures:object-not-found (f)
                     (declare (ignore f))
                     (cpl:do-retry cnt-perc
                       (cpl:retry))
                     (cpl:fail 'cram-plan-failures::simple-plan-failure)))
                (sample-perceive))))
        (unless obj-position
          (cpl:fail 'cram-plan-failures::simple-plan-failure))
        (cpl:with-failure-handling
            ((cram-plan-failures:location-not-reached-failure (f)
               (declare (ignore f))
               (cpl:do-retry cnt-nav
                 (cpl:retry))
               (cpl:fail 'cram-plan-failures::simple-plan-failure)))
          (sample-navigate obj-position))
        (cpl:with-failure-handling
            ((cram-plan-failures:manipulation-pose-unreachable (f)
               (declare (ignore f))
               (cpl:do-retry cnt-grasp
                 (cpl:retry))
               (cpl:fail 'cram-plan-failures::simple-plan-failure)))
          (sample-grasp))))))

(def-cram-function sample-perceive ()
  (prediction:choose
   perceive
   :generators (((look-position)
                 `(,(tf:make-3d-vector (/ (random 100) 10.0)
                                       (/ (random 100) 10.0)
                                       0.0))))
   :features ((look-x (tf:x look-position))
              (look-y (tf:y look-position)))
   :constraints ((cram-plan-failures::objectnotfound
                  (or (not cut::predicted-failure)
                      (< cut::predicted-failure
                         0.5))))
   :predicting (time)
   :attempts 2
   :body
   (progn
     (unless (and (< look-x 5)
                  (< look-y 9))
       (cpl:fail
        'cram-plan-failures:object-not-found))
     look-position)))

(def-cram-function sample-navigate (obj-position)
  (format t "Go in: ~a~%" obj-position)
  (cut:choose
   navigate
   :generators (((nav-position)
                 `(,(tf:make-3d-vector (/ (random 100) 10.0)
                                       (/ (random 100) 10.0)
                                       0.0))))
   :features ((nav-x (tf:x nav-position))
              (nav-y (tf:y nav-position))
              (distance (tf:v-dist nav-position
                                   obj-position)))
   :constraints ((cram-plan-failures::locationnotreached
                  (or (not cut::predicted-failure)
                      (< cut::predicted-failure
                         0.5)))
                 (cram-plan-failures::manipulationposeunreachable
                  (or (not cut::predicted-failure)
                      (< cut::predicted-failure
                         0.5))))
   :predicting (time)
   :attempts 2
   :body
   (progn
     (unless (and (< 2 nav-x 8)
                  (< 3 nav-y 9))
       (cpl:fail
        'cram-plan-failures:location-not-reached-failure))
     (unless (< distance 3)
       (cpl:fail
        'cram-plan-failures:manipulation-pose-unreachable)))))

(def-cram-function sample-grasp ()
  (:tag grasp))

(def-cram-function sample-place ()
  (:tag place))

(def-top-level-cram-function test-rethrow ()
  (let ((fail t))
    (cpl:with-failure-handling
        ((cram-plan-failures:object-not-found (f)
           (declare (ignore f))
           (cpl:retry)))
      (cpl:with-failure-handling
          ((cram-plan-failures:object-not-found (f)
             (declare (ignore f))))
        (when fail
          (setf fail nil)
          (cpl:fail 'cram-plan-failures:object-not-found))))))

(def-top-level-cram-function test-equate ()
  (with-designators ((obj-1 (object `()))
                     (obj-2 (object `()))
                     (obj-3 (object `())))
    (with-designators ((obj-4 (object `())))
      (equate obj-3 obj-4))))

(def-top-level-cram-function new-perception-interface (&key (types nil))
  (let ((results (robosherlock-pm::perceive
                  (cram-designators:make-designator 'object nil))))
    (cond (types
           (dolist (result results)
             (format t "~a~%" (desig-prop-value result 'desig-props::type))))
          (t results))))
