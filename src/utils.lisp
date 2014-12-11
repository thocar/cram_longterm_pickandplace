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
(defvar *tf2* nil)

(defparameter *wait-for-trigger* nil)

(defun init-longterm-pickandplace ()
  (setf *tf2* (make-instance 'cl-tf2:buffer-client)))

(roslisp-utilities:register-ros-init-function init-longterm-pickandplace)

(defmacro try-forever (&body body)
  `(cpl:with-failure-handling
       (((or cram-plan-failures:object-not-found
             cram-plan-failures:manipulation-failure
             cram-plan-failures:location-not-reached-failure) (f)
          (declare (ignore f))
          (cpl:retry)))
     ,@body))

(defmacro try-n-times (n &body body)
  `(cpl:with-retry-counters ((retry-count ,n))
     (cpl:with-failure-handling
         (((or cram-plan-failures:object-not-found
               cram-plan-failures:manipulation-failure
               cram-plan-failures:location-not-reached-failure) (f)
            (declare (ignore f))
            (cpl:do-retry retry-count
              (cpl:retry))))
       ,@body)))

(defun publish-pose (pose &optional (topic "/object"))
  (let ((adv (roslisp:advertise topic "geometry_msgs/PoseStamped")))
    (roslisp:publish adv (tf:pose-stamped->msg pose))))

(defun set-locations ()
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
         `((desig-props:on Cupboard)))))

(defun prob-prop (symbol values threshold &key dont-list-wrap)
  "With the probability `threshold', a property key/value pair with the key `symbol' and a value from the list `values' is generated. Each entry of `values' is either a symbol, or a cons cell contanining the value symbol and its individual occurrence probability. If `threshold' is not reached, `NIL' is returned.
  
  A sample call to `prob-prop' looks like this:
  
  (prob-prop 'color
             `(red (blue 0.1) green yellow white)
             0.4)
  
  Which translates to: With 40% probability, generate the property `color', containing one of the values `red', `blue', `yellow', or `white'. The relative occurrence each value is 1/4.1, except for blue, which occurs with a relative probability of 0.1/4.1."
  (when (<= (random 1.0) threshold)
    (let* ((values-span
             (reduce (lambda (list-so-far new-entry)
                       (append list-so-far
                               `(,(cons
                                   (car new-entry)
                                   (+ (cdr new-entry)
                                      (cond ((> (length list-so-far) 0)
                                             (cdr (car (last list-so-far))))
                                            (t 0)))))))
                     (mapcar (lambda (value)
                               (cond ((listp value)
                                      (cons (car value)
                                            (cadr value)))
                                     (t (cons value 1.0))))
                             values)
                     :initial-value nil))
           (decided-prob (random (cdar (last values-span))))
           (decided-value (reduce (lambda (x y)
                                    (cond ((or
                                            (not x)
                                            (< (cdr x) decided-prob (cdr y)))
                                           y)
                                          (t x)))
                                  values-span
                                  :initial-value nil)))
      (cond (dont-list-wrap
             `(,symbol ,(car decided-value)))
            (t `((,symbol ,(car decided-value))))))))

(defmacro perceive-a (object &key stationary (move-head t))
  `(cpl:with-failure-handling
       ((cram-plan-failures:object-not-found (f)
          (declare (ignore f))
          (ros-warn (longterm) "Object not found. Retrying.")
          (cpl:retry)))
     (cond (,stationary
            (let ((at (desig-prop-value ,object 'desig-props:at)))
              (when ,move-head
                (achieve `(cram-plan-library:looking-at ,(reference at))))
              (first (perceive-object
                      'cram-plan-library:currently-visible
                      ,object))))
           (t (cpl:with-failure-handling
                  ((cram-plan-failures:location-not-reached-failure (f)
                     (declare (ignore f))
                     (cpl:retry)))
                (perceive-object 'cram-plan-library:a ,object))))))

(defun perceive-scene ()
  (with-designators ((act-perc-scn (action `((desig-props:to
                                              desig-props:perceive)
                                             (desig-props:obj
                                              desig-props:scene)))))
    (perform act-perc-scn)))

(defmacro pick-object (object &key stationary)
  `(cpl:with-retry-counters ((retry-task 0))
     (cpl:with-failure-handling
         ((cram-plan-failures:manipulation-failure (f)
            (declare (ignore f))
            (cpl:do-retry retry-task
              (ensure-arms-up)
              (cpl:retry)))
          (cram-plan-failures:location-not-reached-failure (f)
            (declare (ignore f))
            (cpl:do-retry retry-task
              (ros-warn (longterm) "Cannot reach location. Retrying.")
              (cpl:retry)))
          (cram-plan-failures:object-not-found (f)
            (declare (ignore f))
            (cpl:do-retry retry-task
              (ros-warn (longterm) "Object not found. Retrying.")
              (cpl:retry))))
       ,(cond (stationary
               `(achieve `(cram-plan-library:object-picked ,,object)))
              (t
               `(achieve `(cram-plan-library:object-in-hand ,,object)))))))

(defmacro place-object (object location &key stationary)
  `(cpl:with-failure-handling
       ((cram-plan-failures:manipulation-pose-unreachable (f)
          (declare (ignore f))
          (cram-plan-library::retry-with-updated-location
           ,location (next-solution ,location)))
        (cram-plan-failures:location-not-reached-failure (f)
          (declare (ignore f))
          (ros-warn (longterm) "Cannot reach location. Retrying.")
          (cpl:retry)))
     (let ((side (var-value
                  '?side
                  (lazy-car (crs:prolog `(cram-plan-library:object-in-hand ,,object ?side))))))
       (prog1
           ,(cond (stationary
                   `(achieve `(cram-plan-library::object-put ,,object ,,location)))
                  (t
                   `(achieve `(cram-plan-library::object-placed-at ,,object ,,location))))
         (ensure-arms-up side)))))

(defmacro open-container (container)
  )

(defun container-for (object)
  (let* ((at (desig-prop-value object 'desig-props:at))
         (in (desig-prop-value at 'desig-props:in))
         (name (desig-prop-value at 'desig-props:name)))
    (when in
      name)))

(defun move-arms-up (&key allowed-collision-objects side ignore-collisions)
  (when (or (eql side :left) (not side))
    (pr2-manip-pm::execute-move-arm-pose
     :left
     (tf:make-pose-stamped
      "base_link" (roslisp:ros-time)
      (tf:make-3d-vector 0.3 0.5 1.3)
      (tf:euler->quaternion :ax 0));pi))
     :ignore-collisions ignore-collisions
     :allowed-collision-objects allowed-collision-objects))
  (when (or (eql side :right) (not side))
    (pr2-manip-pm::execute-move-arm-pose
     :right
     (tf:make-pose-stamped
      "base_link" (roslisp:ros-time)
      (tf:make-3d-vector 0.3 -0.5 1.3)
      (tf:euler->quaternion :ax 0));pi))
     :ignore-collisions ignore-collisions
     :allowed-collision-objects allowed-collision-objects)))

(defun make-handles (distance-from-center
                     &key
                       (segments 1)
                       (ax 0.0) (ay 0.0) (az 0.0)
                       (offset-angle 0.0)
                       grasp-type
                       (center-offset
                        (tf:make-identity-vector)))
  (loop for i from 0 below segments
        as current-angle = (+ (* 2 pi (float (/ i segments)))
                              offset-angle)
        as handle-pose = (tf:make-pose
                          (tf:make-3d-vector
                           (+ (* distance-from-center (cos current-angle))
                              (tf:x center-offset))
                           (+ (* distance-from-center (sin current-angle))
                              (tf:y center-offset))
                           (+ 0.0
                              (tf:z center-offset)))
                          (tf:euler->quaternion
                           :ax ax :ay ay :az (+ az current-angle)))
        as handle-object = (make-designator
                            'cram-designators:object
                            (append
                             `((desig-props:type desig-props:handle)
                               (desig-props:at
                                ,(a location `((desig-props:pose
                                                ,handle-pose)))))
                             (when grasp-type
                               `((desig-props:grasp-type ,grasp-type)))))
        collect handle-object))

(defgeneric init-ms-belief-state (&key debug-window objects))

(defmethod init-ms-belief-state (&key debug-window objects)
  (ros-info (bullet) "Clearing bullet world")
  (crs:prolog `(btr:clear-bullet-world))
  (let* ((test-0 (ros-info (bullet) "Gathering data"))
         (robot-pose (get-robot-pose))
         (test-01 (ros-info (bullet) "Robot pose OK"))
         (urdf-robot
           (cl-urdf:parse-urdf
            (roslisp:get-param "robot_description_lowres")))
         (test-02 (ros-info (bullet) "Robot description OK"))
         (urdf-kitchen
           (cl-urdf:parse-urdf
            (roslisp:get-param "kitchen_description")))
         (test-03 (ros-info (bullet) "Kitchen description OK"))
         (scene-rot-quaternion (tf:euler->quaternion :az pi))
         (scene-rot `(,(tf:x scene-rot-quaternion)
                      ,(tf:y scene-rot-quaternion)
                      ,(tf:z scene-rot-quaternion)
                      ,(tf:w scene-rot-quaternion)))
         (scene-trans `(-3.45 -4.35 0))
         (robot-pose robot-pose)
         (robot-rot `(,(tf:x (tf:orientation robot-pose))
                      ,(tf:y (tf:orientation robot-pose))
                      ,(tf:z (tf:orientation robot-pose))
                      ,(tf:w (tf:orientation robot-pose))))
         (robot-trans `(,(tf:x (tf:origin robot-pose))
                        ,(tf:y (tf:origin robot-pose))
                        ,(tf:z (tf:origin robot-pose))))
         (test-1 (ros-info (bullet) "Asserting scene"))
         (bdgs
           (car
            (force-ll
             (crs:prolog
              `(and (btr:clear-bullet-world)
                    (btr:bullet-world ?w)
                    (btr:assert (btr:object
                                 ?w btr:static-plane floor
                                 ((0 0 0) (0 0 0 1))
                                 :normal (0 0 1) :constant 0))
                    ,@(when debug-window
                       `((btr:debug-window ?w)))
                    (btr:robot ?robot)
                    ,@(loop for object in objects
                            for obj-name = (car object)
                            for obj-pose = (cdr object)
                            collect `(btr:assert
                                      (btr:object
                                       ?w btr:box ,obj-name
                                       ((,(tf:x (tf:origin obj-pose))
                                         ,(tf:y (tf:origin obj-pose))
                                         ,(tf:z (tf:origin obj-pose)))
                                        (,(tf:x (tf:orientation obj-pose))
                                         ,(tf:y (tf:orientation obj-pose))
                                         ,(tf:z (tf:orientation obj-pose))
                                         ,(tf:w (tf:orientation obj-pose))))
                                       :mass 0.0 :size (0.1 0.1 0.1))))
                    (assert (btr:object
                             ?w btr:urdf ?robot
                             (,robot-trans ,robot-rot)
                             :urdf ,urdf-robot))
                    (assert (btr:object
                             ?w btr:semantic-map sem-map-kitchen
                             (,scene-trans ,scene-rot)
                             :urdf ,urdf-kitchen))))))))
    (ros-info (bullet) "Check binding for robot")
    (var-value
     '?pr2
     (lazy-car
      (crs:prolog
       `(and (btr:robot ?robot)
             (btr:%object ?w ?robot ?pr2)) bdgs)))
    (ros-info (bullet) "Check binding for kitchen")
    (var-value
     '?sem-map
     (lazy-car
      (crs:prolog
       `(btr:%object ?w sem-map-kitchen ?sem-map) bdgs)))
    (ros-info (bullet) "Done")
    (robosherlock-pm::ignore-bullet-object 'sem-map-kitchen)
    (robosherlock-pm::ignore-bullet-object 'common-lisp::floor)
    (robosherlock-pm::ignore-bullet-object 'cram-pr2-knowledge::pr2)))

(defmethod init-belief-state-temp ()
  (crs:prolog
   `(and (btr:clear-bullet-world)
         (btr:bullet-world ?w)
         (btr:assert (btr:object
                      ?w btr:static-plane floor
                      ((0 0 0) (0 0 0 1))
                      :normal (0 0 1) :constant 0))
         (btr:debug-window ?w)
         (btr:robot ?robot)
         (assert (btr:object
                  ?w btr:urdf ?robot
                  ((0 0 0) (0 0 0 1))
                  :urdf ,(cl-urdf:parse-urdf
                          (roslisp:get-param "robot_description_lowres")))))))

(defmacro with-process-modules (&body body)
  `(cpm:with-process-modules-running
       (pr2-manipulation-process-module:pr2-manipulation-process-module
        pr2-navigation-process-module:pr2-navigation-process-module
        point-head-process-module:point-head-process-module
        robosherlock-process-module:robosherlock-process-module)
     ,@body))

(defun get-robot-pose ()
  (cl-tf2:ensure-pose-stamped-transformed
   *tf2*
   (tf:make-pose-stamped
    "/base_link"
    0.0
    (tf:make-identity-vector)
    (tf:make-identity-rotation))
   "/map" :use-current-ros-time t))

(defun drive-to-pose (pose-stamped)
  (block nav
    (with-designators ((loc (location `((desig-props:pose ,pose-stamped))))
                       (act (action `((desig-props:type desig-props:navigation)
                                      (desig-props:goal ,loc)))))
      (cpl:with-failure-handling ((cram-plan-failures:location-not-reached-failure (f)
                                (declare (ignore f))
                                (return-from nav)))
        (cpl:pursue
          (sleep 25)
          (perform act))))))

(def-top-level-cram-function see-object (description)
  (with-process-modules
    (with-designators ((obj (object description)))
      (cram-plan-library:perceive-object
       'cram-plan-library:currently-visible obj))))

(def-top-level-cram-function see-scene ()
  (with-process-modules
    (with-designators ((ps-action (action `((desig-props:to desig-props:perceive)
                                            (desig-props:obj desig-props:scene)))))
      (perform ps-action))))

(defun ensure-arms-up (&optional (side (list :left :right)))
  (cond ((listp side)
         (dolist (s side)
           (ensure-arms-up s)))
        (t
         (let ((ignore-collisions nil))
           (cpl:with-failure-handling
               ((cram-plan-failures:manipulation-failure (f)
                  (declare (ignore f))
                  (setf ignore-collisions t)
                  (cpl:retry)))
             (move-arms-up :side side
                           :ignore-collisions ignore-collisions))))))

(defun renew-collision-environment ()
  (moveit:clear-collision-environment)
  (sem-map-coll-env:publish-semantic-map-collision-objects)
  (sem-map-coll-env:publish-semantic-map-collision-objects))

(defun prepare-settings ()
  (setf *wait-for-trigger* nil)
  (setf location-costmap::*fixed-frame* "/map")
  ;; NOTE(winkler): This validator breaks IK based `to reach' and `to
  ;; see' location resolution. Disabling it, since everything works
  ;; just nicely without it. Gotta look into this later.
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
  (beliefstate::enable-logging t)
  (ros-info (longterm) "Init Belief State")
  (init-ms-belief-state :debug-window t)
  (setf btr::*bb-comparison-validity-threshold* 0.1)
  (moveit:clear-collision-environment)
  ;; Twice, because sometimes a ROS message for an object gets lost.
  ;(sem-map-coll-env:publish-semantic-map-collision-objects)
  (sem-map-coll-env:publish-semantic-map-collision-objects))

(defun wait-for-external-trigger (&key force)
  (when (or *wait-for-trigger* force)
    (roslisp:ros-info (demo-control) "Waiting for external trigger to continue.")
    (let* ((waiter-fluent (cpl:make-fluent))
           (subscriber (roslisp:subscribe
                        "/waiter_fluent"
                        "std_msgs/Empty"
                        (lambda (msg)
                          (declare (ignore msg))
                          (cpl:setf (cpl:value waiter-fluent) t)))))
      (loop until (cpl:value waiter-fluent) do (sleep 0.1))
      (roslisp:unsubscribe subscriber))
    (roslisp:ros-info (demo-control) "External trigger arrived.")))

(defun object-color (colors color)
  (let ((color-pair (find color colors :test (lambda (x y) (eql x (car y))))))
    (if color-pair
        (cadr color-pair)
        0.0d0)))

(def-fact-group object-refinement-facts (infer-object-property
                                         object-handle
                                         cram-language::grasp-effort)
  
  (<- (cram-language::grasp-effort ?object 80)
    (desig-prop ?object (desig-props::is desig-props::fragile)))
  
  (<- (make-handles ?segments ?offset-angle ?handles)
    (symbol-value pi ?pi)
    (crs:lisp-fun / ?pi 2 ?pi-half)
    (make-handles 0.04 ?segments ?offset-angle 'desig-props::push
                  ?pi-half 0 0 0 0 0 ?handles))
  
  (<- (make-handles ?distance-from-center ?segments ?offset-angle ?grasp-type
                    ?hand-ax ?hand-ay ?hand-az ?co-x ?co-y ?co-z ?handles)
    (crs:lisp-fun tf:make-3d-vector ?co-x ?co-y ?co-z ?co)
    (crs:lisp-fun make-handles ?distance-from-center
                  :segments ?segments
                  :offset-angle ?offset-angle
                  :grasp-type ?grasp-type
                  :ax ?hand-ax
                  :ay ?hand-ay
                  :az ?hand-az
                  :center-offset ?co
                  ?handles))
  
  (<- (type-property desig-props:pot desig-props::carry-handles 2))
  
  (<- (type-property desig-props:pot desig-props::lid t))
  
  (<- (object-color ?object ?color ?value)
    (desig-prop ?object (desig-props:color ?colors))
    (crs:lisp-fun object-color ?colors ?color ?value))
  
  (<- (infer-object-property ?object ?key ?value)
    (desig-prop ?object (desig-props:type ?type))
    (type-property ?type ?key ?value))
  
  (<- (infer-object-property ?object desig-props:type desig-props::pancakemix)
    (object-color ?object desig-props:yellow ?yellow)
    (object-color ?object desig-props:red ?red)
    (> ?yellow 0.5)
    (> ?red 0.1))
  
  (<- (infer-object-property ?object desig-props:type desig-props::bowl)
    (object-color ?object desig-props:white ?white)
    (> ?white 0.7))
  
  (<- (infer-object-property ?object desig-props:type desig-props::muesli)
    (object-color ?object desig-props:yellow ?yellow)
    (object-color ?object desig-props:red ?red)
    (> ?yellow 0.3)
    (> ?red 0.2))
  
  (<- (infer-object-property ?object desig-props:type desig-props::milkbox)
    (object-color ?object desig-props:yellow ?yellow)
    (> ?yellow 0.1))
  
  (<- (infer-object-property ?object desig-props:type desig-props::dinnerplate)
    (desig-prop ?object (desig-props:color (desig-props::white ?white)))
    (> ?white 0.8))
  
  (<- (infer-object-property ?object desig-props:handle ?handle)
    (infer-object-property ?object desig-props:type ?type)
    (object-handle ?type ?handles-list)
    (member ?handle ?handles-list))
  
  (<- (infer-object-property ?object desig-props::is desig-props::fragile)
    (or (desig-prop ?object (desig-props::type desig-props::bowl))
        (desig-prop ?object (desig-props::type desig-props::dinnerplate))))
  
  (<- (infer-object-property ?object desig-props::category desig-props::container)
    (desig-prop ?object (desig-props::type desig-props::bowl)))

  (<- (infer-object-property ?object desig-props::is desig-props::stackable)
    (desig-prop ?object (desig-props::type desig-props::dinnerplate)))
  
  (<- (object-handle desig-props::milkbox ?handles-list)
    (symbol-value pi ?pi)
    (crs:lisp-fun / ?pi 2 ?pi-half)
    (make-handles 2 ?pi-half ?handles-list))
  
  (<- (object-handle desig-props::muesli ?handles-list)
    (symbol-value pi ?pi)
    (crs:lisp-fun / ?pi 2 ?pi-half)
    (make-handles 2 ?pi-half ?handles-list))

  (<- (object-handle desig-props::pancakemix ?handles-list)
    (symbol-value pi ?pi)
    (crs:lisp-fun / ?pi 2 ?pi-half)
    (make-handles 2 ?pi-half ?handles-list))
  
  (<- (object-handle desig-props::dinnerplate ?handles-list)
    (symbol-value pi ?pi)
    (crs:lisp-fun / ?pi 4 ?tilt)
    (crs:lisp-fun / ?pi -2.5 ?pi-half)
    (make-handles -0.07 8 ?pi-half 'desig-props::push ?pi ?tilt 0 0 0 -0.015 ?handles-list))

  (<- (object-handle desig-props::bowl ?handles-list)
    (symbol-value pi ?pi)
    (crs:lisp-fun / ?pi 4 ?tilt)
    (crs:lisp-fun / ?pi -2.5 ?pi-half)
    (make-handles -0.015 4 ?pi-half 'desig-props::push ?pi ?tilt 0 0.1 0.0 -0.0 ?handles-list))

  (<- (infer-object-property ?object desig-props::carry-handles ?carry-handles)
    (infer-object-property ?object desig-props:type ?type)
    (object-carry-handles ?type ?carry-handles))
  
  ;; Dinnerplate: Carry with 2 arms
  (<- (object-carry-handles desig-props::dinnerplate 2))
  ;; Bowl: Carry with 2 arms
  (<- (object-carry-handles desig-props::bowl 2))
  ;; Muesli: Carry with 1 arm
  (<- (object-carry-handles desig-props::muesli 1))
  ;; Milkbox: Carry with 1 arm
  (<- (object-carry-handles desig-props::milkbox 1)))
