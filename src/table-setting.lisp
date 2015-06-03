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

;;;
;;; Note
;;;

;; Planned experiment elements:
;;  - Grasp object with one hand
;;  - Grasp fragile object with one hand (less effort)
;;  - Grasp object with two hands (multiple minimum handles)
;;  - Grasp heavy object (thus infering multiple minimum handles)
;;  - Get object from drawer
;;  - Place multiple objects on a kitchen counter
;;  - Put object into drawer
;;  - Open drawer
;;  - Close drawer
;;  - Grasp two different objects


;;;
;;; Definitions
;;;

(defvar *scene-context* (make-hash-table))

;;;
;;; Helper Functions
;;;

(defun pose-link-origin-distance (pose-stamped link)
  (let ((pose-in-link (cl-tf2:ensure-pose-stamped-transformed
                       *tf2* pose-stamped link)))
    (tf:v-dist (tf:origin pose-in-link)
               (tf:make-identity-vector))))

(defun clear-scene ()
  (setf *scene-context* (make-hash-table)))

(defun set-scene-detail (detail value)
  (setf (gethash detail *scene-context*) value))

(defun scene-detail (detail)
  (gethash detail *scene-context*))

(defun set-scene-1 ()
  (set-scene-detail 'guests `(bob))
  (set-scene-detail 'meal-time 'breakfast)
  (set-scene-detail 'week-day 'saturday))

;; EIGENE SZENEN ANFANG
(defun set-scene-thomasthesis-test-01 ()
  (set-scene-detail 'guests `(tim mary))
  (set-scene-detail 'meal-time 'lunch)
  (set-scene-detail 'week-day 'sunday))

(defun set-scene-thomasthesis-test-02 ()
  (set-scene-detail 'guests `(bob))
  (set-scene-detail 'meal-time 'lunch)
  (set-scene-detail 'week-day 'sunday))

(defun set-scene-thomasthesis-test-03 ()
  (set-scene-detail 'guests `(tim mary bob))
  (set-scene-detail 'meal-time 'dinner)
  (set-scene-detail 'week-day 'sunday))

(defun set-scene-thomasthesis-test-04 ()
  (set-scene-detail 'guests `(tim mary bob alice))
  (set-scene-detail 'meal-time 'lunch)
  (set-scene-detail 'week-day 'sunday))

(defun set-scene-thomasthesis-test-05 ()
  (set-scene-detail 'guests `(alice bob))
  (set-scene-detail 'meal-time 'breakfast)
  (set-scene-detail 'week-day 'sunday))

(defun set-scene-thomasthesis-one-object ()
  (set-scene-detail 'guests `(carol))
  (set-scene-detail 'meal-time 'breakfast)
  (set-scene-detail 'week-day 'sunday))

(defun set-scene-thomasthesis-experiment-01 ()
  (set-scene-detail 'guests `(tim))
  (set-scene-detail 'meal-time 'breakfast)
  (set-scene-detail 'week-day 'sunday))
;; EIGENE SZENEN ENDE

(defun rectangular-costmap-generator (x-r y-r w h)
  (lambda (x y)
    (if (and (>= x x-r)
             (>= y y-r)
             (< x (+ x-r h))
             (< y (+ y-r w)))
        1.0d0
        0.0d0)))

(defun seat-center (seat)
  (case seat
    (1 '(-0.89 0.85))
    (2 '(-0.89 1.5))
    (3 '(0.89 -1.5))
    (4 '(0.89 -0.85))))

(defun seat-costmap-generator (seat)
  (let ((seat-width 0.5)
        (seat-height 0.4))
    (or (and (> seat 0)
             (< seat 3)
             (let ((seat-center (seat-center seat))
                   (half-width (/ seat-width 2))
                   (half-height (/ seat-height 2)))
               (rectangular-costmap-generator
                (- (first seat-center) half-height)
                (- (second seat-center) half-width)
                seat-width
                seat-height)))
        (lambda (x y)
          (declare (ignore x y))
          0.0d0))))

(defun seat-relative-costmap-generator (seat &key relation)
  (let ((seat-width 0.5)
        (seat-height 0.4))
    (let ((seat-center (seat-center seat))
          (half-width (/ seat-width 2))
          (half-height (/ seat-height 2)))
      (case relation
        (near (rectangular-costmap-generator
               (- (first seat-center) half-height)
               (- (second seat-center) half-width)
               seat-width
               seat-height))
        (left-of (rectangular-costmap-generator
                  (- (first seat-center) half-height)
                  (- (second seat-center) half-width)
                  (* seat-width 0.4)
                  seat-height))
        (right-of (rectangular-costmap-generator
                  (- (first seat-center) half-height)
                  (+ (* seat-width 0.6)
                     (- (second seat-center) half-width))
                  (* seat-width 0.4)
                  seat-height))
        (behind-of (rectangular-costmap-generator
                    (- (first seat-center) half-height)
                    (- (second seat-center) half-width)
                    seat-width
                    (* seat-height 0.4)))
        (center-of (rectangular-costmap-generator
                    (+ (* seat-width 0.4)
                       (- (first seat-center) half-height))
                    (+ (* seat-height 0.3)
                       (- (second seat-center) half-width))
                    (* seat-width 0.4)
                    (* seat-height 0.6)))))))

(defun positions->seat-location (seat positions)
  (make-designator
   'location
   (append `((seat ,seat)
             (desig-props::on Cupboard)
             (desig-props::name "kitchen_island"))
           (mapcar (lambda (position)
                     `(,position seat))
                   positions))))

(defun object-type->object (object-type location guest)
    (make-designator 
     'object
     `((bring-to ,location)
       (type ,object-type)
       (for-guest ,guest))))

(defun order-scene-objects (objects)
  (let* ((object-seat-placement-modifiers
           (mapcar
            (lambda (object)
              (let* ((at (desig-prop-value object 'at))
                     (seat-placements
                       (cpl:mapcar-clean
                        (lambda (property)
                          (when (eql (cadr property) 'seat)
                            (car property)))
                        (description at))))
                (cons object seat-placements)))
            objects)))
    (sort object-seat-placement-modifiers
          (lambda (set-1 set-2)
            (mapcar
             (lambda (modif-1)
               (mapcar
                (lambda (modif-2)
                  (let ((orderings
                          (force-ll
                           (lazy-mapcar
                            (lambda (bdgs)
                              (with-vars-bound (?order) bdgs
                                ?order))
                            (crs:prolog
                             `(seat-place-ordering
                               ,modif-1 ,modif-2
                               ?order))))))
                    (format t "~a ~a: ~a~%" modif-1 modif-2 orderings)
                    orderings))
                (rest set-2)))
             (rest set-1))))))

(defun required-scene-objects ()
  (let* ((lazy-scene-objects (crs:prolog `(required-object ?object)))
         (scene-objects
           (force-ll
            (lazy-mapcar (lambda (bdgs)
                           (with-vars-bound (?object) bdgs
                             ?object))
                         lazy-scene-objects))))
    scene-objects))

;; EIGENE FUNKTIONEN ANFANG

;; Sucht ein Objekt an den vorgesehenen Locations
(defun search-object-at-object-locations (object-designator)
  (loop for bdgs in (force-ll (get-objectlocation-from-object object-designator))
        with done = nil
        when (not done)
          collect
          (with-vars-bound (?obj-loc) bdgs
            (if (or (string= ?obj-loc "fridge1")
                    (string= ?obj-loc "drawer1"))
                (progn
                  (roslisp::ros-info (search-object) "Can't find ~a at this location: ~a" (prin1-to-string (desig-prop-value object-designator 'type)) ?obj-loc)
                  nil)
                (with-designators ((o-loc (location `((desig-props::on Cupboard)
                                                      (desig-props::name ,?obj-loc))))
                                   (obj (object (desig:update-designator-properties `((desig-props::at ,o-loc))
                                                                                    (description object-designator)))))
                  (roslisp:ros-info (search-object) "in top level")
                  (block p-block
                    (cpl-impl:with-failure-handling
                        ((cram-plan-failures:object-not-found (f)
                           (declare (ignore f))
                           (return-from p-block)))
                      (unwind-protect
                           (perceive-a obj :ignore-object-not-found t :stationary t)
                        (setf done t)))))))))

(defun current-robot-pose ()
  (cl-tf2:ensure-pose-stamped-transformed
   *tf2* (tf:pose->pose-stamped "base_footprint" 0.0
                                (tf:make-identity-pose))
   "map"))

(defun semantic-object-pose (object-name)
  (let* ((island (make-designator
                  'object `((desig-props:name ,object-name))))
         (object (first (sem-map-utils:designator->semantic-map-objects
                         island)))
         (pose (sem-map-utils:pose object)))
    pose))

(defun distance-to-semantic-object (object-name)
  (let* ((robot-pose (current-robot-pose))
         (semantic-object-pose (semantic-object-pose object-name)))
    (tf:v-dist (tf:origin robot-pose)
               (tf:origin semantic-object-pose))))

(defun sorted-semantic-object-distances (object-names)
  (sort (mapcar (lambda (object-name)
                  (cons
                   object-name
                   (distance-to-semantic-object object-name)))
                object-names)
        (lambda (obj-1 obj-2)
          (destructuring-bind (obj-1-name . obj-1-distance) obj-1
            (declare (ignore obj-1-name))
            (destructuring-bind (obj-2-name . obj-2-distance) obj-2
              (declare (ignore obj-2-name))
              (< obj-1-distance obj-2-distance))))))

(defmacro in-front-of (location try-indefinitely &body body)
  `(cpl:with-retry-counters ((ctr 3))
     (cpl:with-failure-handling
         ((cram-plan-failures:location-not-reached-failure (f)
            (declare (ignore f))
            (when ,try-indefinitely
              (cpl:do-retry ctr
                (cpl:retry))
              (return t))
            (unless ,try-indefinitely
              (return t))))
       (at-location (,location)
         ,@body))))

(defun go-in-front-of-island ()
  (in-front-of
   (desig:make-designator
    'location
    `((desig-props::pose
       ,(cl-tf:pose->pose-stamped
         "/map" 0.0
         (cl-transforms:make-pose
          (cl-transforms:make-3d-vector -0.323 1.137 0.0)
          (cl-transforms:make-quaternion 0 0 1 0.03))))))
   nil))

(defun look-at-pose (pose)
  (achieve `(cram-plan-library:looking-at ,pose)))

(defun perceive-all (object &key stationary (move-head t))
  (cpl:with-failure-handling
      ((cram-plan-failures:object-not-found (f)
         (declare (ignore f))
         (ros-warn (longterm) "Object not found. Retrying.")
         (cpl:retry)))
    (cond (stationary
           (let ((at (desig-prop-value object 'desig-props:at)))
             (when move-head
               (achieve `(cram-plan-library:looking-at (reference at))))
             (perceive-object
              'cram-plan-library:currently-visible object)))
          (t (cpl:with-failure-handling
                 ((cram-plan-failures:location-not-reached-failure (f)
                    (declare (ignore f))
                    (cpl:retry)))
               (perceive-object 'cram-plan-library:all object))))))

;; Gibt eine Liste von Objekten zurück, die auf dem Tisch perceived wurden
(defun perceive-table ()
  (let* ((pose (semantic-object-pose "kitchen_island"))
         (pose2 (cl-transforms:make-pose
                 (cl-transforms:v+ (cl-transforms:origin pose) (cl-transforms:make-3d-vector 0 0 (cl-transforms:z (cl-transforms:origin pose))))
                 (cl-transforms:orientation pose)))
         (pose3 (tf-types:pose->pose-stamped "map" 0.0 pose2)))
    (look-at-pose pose3))
  (perceive-all (make-designator 'object `((desig-props::type desig-props::pancakemix))) :stationary t :move-head nil)
 )

;; Gibt ein Objekt aus einer Liste zurück, wenn es sich dabei um das gesuchte Objekt handelt
(defun object-in-list (object-designator objects-list)
  (find object-designator objects-list
        :test (lambda (a b)
                (equal (desig-prop-value a 'type)
                       (desig-prop-value b 'type)))))

(defun go-forward (&optional (distance 0.3))
  (labels ((move-base-relative-pose (vector)
             (let* ((base-id
                      (cl-tf:pose->pose-stamped
                       "base_footprint" 0.0
                       (cl-tf:make-pose vector (cl-tf:euler->quaternion))))
                    (action (make-designator
                             'action `((desig-props:type
                                        desig-props:navigation)
                                       (desig-props:goal
                                        ,(make-designator
                                          'location
                                          `((desig-props:pose
                                             ,base-id))))))))
                 (perform action))))
    (move-base-relative-pose (tf:make-3d-vector distance 0 0))))

;; Testfunktionen
(defun test-scene-one-object()
  (set-scene-thomasthesis-one-object)
  (mapcar (lambda (bdgs)
            (destructuring-bind (obj-desig bringto) bdgs
              (declare (ignore bringto))
              (cpl-impl:mapcar-clean #'identity (force-ll (find-object obj-desig)))))
          (extract-objectdesig-and-bringto)))

(defun test-scene-experiment-01()
  (set-scene-thomasthesis-experiment-01)
  (mapcar (lambda (bdgs)
            (destructuring-bind (obj-desig bringto) bdgs
              (declare (ignore bringto))
              (cpl-impl:mapcar-clean #'identity (force-ll (find-object obj-desig)))))
          (extract-objectdesig-and-bringto)))

(defun test-object-in-list ()
  (set-scene-thomasthesis-test-02)
  (let ((obj-list (extract-objectdesig-and-bringto))
        (obj (make-designator 'object `((desig-props::type knife)
                                        (desig-props::for-guest bob)))))
    (object-in-list obj obj-list)))

(defun test-objectlocation-from-object ()
  (set-scene-thomasthesis-experiment-01)
  (let ((obj (make-designator 'object `((desig-props::type milkbox)
                                        (desig-props::for-guest tim)))))
    (get-objectlocation-from-object obj)))

(defun find-object(object-designator)
  (lazy-mapcar (lambda (bdgs)
                 (with-vars-bound (?obj-loc) bdgs
                   (if (or (string= ?obj-loc "fridge1")
                           (string= ?obj-loc "drawer1"))
                       (progn
                         (roslisp::ros-info (find-object) "Can't find ~a at this location: ~a" (prin1-to-string (desig-prop-value object-designator 'type)) ?obj-loc)
                         nil)
                       (top-level
                         (with-designators ((o-loc (location `((desig-props::on Cupboard)
                                                               (desig-props::name ,?obj-loc))))
                                            (obj (object (desig:update-designator-properties `((desig-props::at ,o-loc)) (description object-designator)))))
                           obj)))))
               (get-objectlocation-from-object object-designator)))

(defun bt-selector (loc-list)
  (cond ((equal (first loc-list) "kitchen_island") "Drive to kitchen_island")
        ((equal (first loc-list) "kitchen_sink_block") "Search at kitchen_sink_block")))

(defun bt-sequence ()
  (let ((obj (make-designator 'object `((desig-props::type milkbox)
                                        (desig-props::for-guest tim))))
        (bring-to (location `((desig-props::on Cupboard)
                              (desig-props::name "kitchen_island")))))
    (progn
      (pick-object obj)
      (place-object obj bring-to))))
    
          

;; EIGENE FUNKTIONEN ENDE

;;;
;;; Plans
;;;

;; EIGENE PLÄNE ANFANG

(def-top-level-cram-function set-table-thomasthesis (scene)
  (if (string= scene "one-object")
      (progn
        (set-scene-thomasthesis-one-object)
        (roslisp::ros-info (Thomas-Thesis-Setting) "Set Scene with one object")))
  (if (string= scene "experiment01")
      (progn 
        (set-scene-thomasthesis-experiment-01)
        (roslisp::ros-info (Thomas-Thesis-Setting) "Set Experiment 01 Scene")))
  (lazy-mapcar (lambda (bdgs)
                 (destructuring-bind (obj-desig bring-to) bdgs
                   (let ((object-found (cpl-impl:mapcar-clean #'identity (search-object-at-object-locations obj-desig))))
                     (unless object-found
                       (equate obj-desig object-found)
                       (pick-object object-found :ignore-object-not-found t)
                       (place-object object-found bring-to)))))
               (extract-objectdesig-and-bringto)))


(def-top-level-cram-function set-table-thomasthesis-02 (scene)
  (if (string= scene "one-object")
      (progn
        (set-scene-thomasthesis-one-object)
        (roslisp::ros-info (Thomas-Thesis-Setting) "Set Scene with one object")))
  (if (string= scene "experiment01")
      (progn 
        (set-scene-thomasthesis-experiment-01)
        (roslisp::ros-info (Thomas-Thesis-Setting) "Set Experiment 01 Scene")))
  (lazy-mapcar (lambda (bdgs)
                 (destructuring-bind (obj-desig bring-to) bdgs
                   (let ((checked-table nil)
                         (object-found nil)
                         (retry-counter 0))
                     (loop while (and
                                  (< retry-counter 4)
                                  (not object-found)
                                  (not checked-table))
                           do (if (< retry-counter 3)
                                  (let ((object-found (cpl-impl:mapcar-clean #'identity (search-object-at-object-locations obj-desig))))
                                    (unless object-found
                                      (equate obj-desig object-found)
                                      (pick-object object-found :ignore-object-not-found t)
                                      (place-object object-found bring-to)))
                                  (if (not checked-table)
                                        (with-designators ((o-loc (location `((desig-props:on Cupboard)
                                                                              (desig-props::name "kitchen_island"))))
                                                           (obj (object (desig:update-designator-properties `((desig-props::at ,o-loc))
                                                                                                            (description obj-desig)))))
                                          (let ((object-found (perceive-a obj :ignore-object-not-found t)))
                                            (unless object-found
                                              (equate obj-desig object-found)
                                              (pick-object object-found :ignore-object-not-found t)
                                              (place-object object-found bring-to)))
                                          (setq checked-table t))))
                              (setq retry-counter (1+ retry-counter))))))
               (extract-objectdesig-and-bringto)))


(def-top-level-cram-function set-table-thomasthesis-with-distance-checking ()
  (set-scene-thomasthesis-experiment-01)
  (prepare-settings)
  (let ((checked-table nil)
        (objects-on-table nil))
    (with-process-modules
      (lazy-mapcar (lambda (bdgs)
                     (destructuring-bind (obj-desig bring-to) bdgs
                       (if (and
                            (not checked-table)
                            (equal "kitchen_island" (first (first (sorted-semantic-object-distances '("kitchen_sink_block" "kitchen_island"))))))
                           (progn
                             (roslisp:ros-info (plan) "progn kitchen_island nearest")
                             (go-in-front-of-island)
                             (setq objects-on-table (perceive-table))
                             (setq checked-table t)))
                       (let ((obj-on-table (first objects-on-table))
                             (placed-object nil))
                             ;(setq obj-on-table (first objects-on-table));(object-in-list obj-desig objects-on-table))
                         (if obj-on-table
                             (progn
                               (roslisp:ros-info (plan) "progn obj-on-table")
                               (equate obj-desig obj-on-table)
                               (pick-object obj-on-table :ignore-object-not-found t)
                               (place-object obj-on-table bring-to)
                               (setq placed-object t))
                             (let ((retry-counter 0))
                               (loop while (and
                                            (not placed-object)
                                            (< retry-counter 2))
                                     do (if (< retry-counter 1)
                                            (progn
                                              (roslisp:ros-info (checkksb) "Begin with checking kitchen_sink_block")
                                              (setq retry-counter (+ retry-counter 1))
                                              (let* ((result (search-object-at-object-locations obj-desig))
                                                     (object-found (first (cpl-impl:mapcar-clean #'identity result))))
                                                (format t "done~%")
                                                (when object-found
                                                  (equate obj-desig object-found)
                                                  (pick-object object-found :ignore-object-not-found t)
                                                  (place-object object-found bring-to)
                                                  (setq placed-object t))))
                                            (if (not checked-table)
                                                (progn
                                                  (roslisp:ros-info (checktable) "Begin with checking table")
                                                  (setq retry-counter (+ retry-counter 1))
                                                  (go-in-front-of-island)
                                                  (setq objects-on-table (perceive-table))
                                                  (setq checked-table t)
                                                  (setq obj-on-table (first objects-on-table));(object-in-list obj-desig objects-on-table))
                                                  (format t "OBJECT: ~a~%" obj-on-table)
                                                  (when obj-on-table
                                                    (equate obj-desig obj-on-table)
                                                    (pick-object obj-on-table :ignore-object-not-found t)
                                                    (place-object obj-on-table bring-to)
                                                    (setq placed-object t)))
                                                (setq retry-counter (+ retry-counter 1))))))))))
                   (extract-objectdesig-and-bringto)))))
        

;; EIGENE PLÄNE ENDE

(def-top-level-cram-function set-table ()
  (beliefstate:enable-logging nil)
  (prepare-settings)
  (beliefstate:enable-logging t)
  (set-scene-1)
  (with-process-modules
    (let ((required-objects (required-scene-objects)))
      (dolist (required-object required-objects)
        (with-designators ((object (object
                                    `((desig-props:at
                                       ,(make-designator
                                         'location
                                         `((desig-props::on Cupboard)
                                           (desig-props::name "kitchen_sink_block"))))
                                      ,@(remove 'at (desig:properties required-object)
                                                :key #'car))))
                           (location (location
                                      (description (desig-prop-value
                                                    required-object 'at)))))
          (ensure-arms-up)
          (try-forever
            (pick-object object)
            (place-object object location)))))))

;;;
;;; Facts
;;;

(defmethod costmap-generator-name->score ((name (common-lisp:eql 'seat-distribution))) 18)

(defmethod costmap-generator-name->score ((name (common-lisp:eql 'seat-distribution-relative-left))) 19)
(defmethod costmap-generator-name->score ((name (common-lisp:eql 'seat-distribution-relative-right))) 20)
(defmethod costmap-generator-name->score ((name (common-lisp:eql 'seat-distribution-relative-behind))) 21)
(defmethod costmap-generator-name->score ((name (common-lisp:eql 'seat-distribution-relative-center))) 22)
(defmethod costmap-generator-name->score ((name (common-lisp:eql 'seat-distribution-relative-near))) 23)

(def-fact-group object-validity-facts (perceived-object-invalid)
  
  (<- (invalid-object-link "l_shoulder_pan_link"))
  (<- (invalid-object-link "l_shoulder_lift_link"))
  (<- (invalid-object-link "l_upper_arm_roll_link"))
  (<- (invalid-object-link "l_upper_arm_link"))
  (<- (invalid-object-link "l_elbow_flex_link"))
  (<- (invalid-object-link "l_forearm_roll_link"))
  (<- (invalid-object-link "l_forearm_cam_frame"))
  (<- (invalid-object-link "l_forearm_cam_optical_frame"))
  (<- (invalid-object-link "l_forearm_link"))
  (<- (invalid-object-link "l_wrist_flex_link"))
  (<- (invalid-object-link "l_wrist_roll_link"))
  (<- (invalid-object-link "l_gripper_palm_link"))

  (<- (invalid-object-link "r_shoulder_pan_link"))
  (<- (invalid-object-link "r_shoulder_lift_link"))
  (<- (invalid-object-link "r_upper_arm_roll_link"))
  (<- (invalid-object-link "r_upper_arm_link"))
  (<- (invalid-object-link "r_elbow_flex_link"))
  (<- (invalid-object-link "r_forearm_roll_link"))
  (<- (invalid-object-link "r_forearm_cam_frame"))
  (<- (invalid-object-link "r_forearm_cam_optical_frame"))
  (<- (invalid-object-link "r_forearm_link"))
  (<- (invalid-object-link "r_wrist_flex_link"))
  (<- (invalid-object-link "r_wrist_roll_link"))
  (<- (invalid-object-link "r_gripper_palm_link"))
  
  (<- (perceived-object-invalid ?object)
    (desig-prop ?object (at ?loc))
    (invalid-object-link ?link)
    (crs:lisp-fun reference ?loc ?pose)
    (crs:lisp-fun pose-link-origin-distance ?pose ?link ?distance)
    (<= ?distance 0.2)))

(def-fact-group table-setting-costmap-facts (desig-costmap)
  
  (<- (distribution-symbol left-of seat-distribution-relative-left))
  (<- (distribution-symbol right-of seat-distribution-relative-right))
  (<- (distribution-symbol behind-of seat-distribution-relative-behind))
  (<- (distribution-symbol center-of seat-distribution-relative-center))
  (<- (distribution-symbol near seat-distribution-relative-near))
  
  (<- (desig-costmap ?desig ?cm)
    (desig-prop ?desig (seat ?seat))
    (desig-prop ?desig (?relation seat))
    (costmap ?cm)
    (distribution-symbol ?relation ?distrib)
    (costmap-add-function ?distrib
                          (seat-relative-costmap-generator
                           ?seat :relation ?relation)
                          ?cm)))

(def-fact-group table-setting-facts ()
  
  ;; Housekeeping predicates
  (<- (context-prop ?context ?detail ?value)
    (crs:lisp-fun gethash ?detail ?context ?value))
  
  (<- (context ?context)
    (symbol-value *scene-context* ?context))
  
  (<- (context-prop ?detail ?value)
    (context ?context)
    (context-prop ?context ?detail ?value))
  
  (<- (context-prop-amount ?detail ?amount)
    (context-prop ?detail ?value)
    (length ?value ?amount))
  
  (<- (preference ?context ?person ?preference ?value)
    (crs:fail))
  
  (<- (guest ?guest)
    (context-prop guests ?guests)
    (member ?guest ?guests))
  
  (<- (seat-description ?seat (seat ?seat)))
  
  ;; Who likes which meals during what time of the day
  (<- (preference tim dish muesli)
    (context-prop meal-time breakfast))
  
  (<- (preference tim dish soup)
    (context-prop meal-time lunch))
  
  (<- (preference tim dish bread)
    (context-prop meal-time dinner))

  (<- (preference mary dish soup)
    (context-prop meal-time dinner))
  
  (<- (preference mary dish bread))

  ;; EIGENE PREFERENCES ANFANG
  (<- (preference bob dish cornflakes)
    (context-prop meal-time breakfast))

  (<- (preference bob dish pizza)
    (context-prop meal-time lunch))

  (<- (preference bob dish bread)
    (context-prop meal-time dinner))

  (<- (preference alice dish scrambled-eggs)
    (context-prop meal-time breakfast))

  (<- (preference alice dish pancakes)
    (context-prop meal-time lunch))

  (<- (preference alice dish cornflakes)
    (context-prop meal-time dinner))

  (<- (preference carol dish milk)
    (context-prop meal-time breakfast))

  (<- (preference carol dish milk)
    (context-prop meal-time lunch))

  (<- (preference carol dish milk)
    (context-prop meal-time dinner))

  (<- (preference max dish milk)
    (context-prop meal-time breakfast))
  ;; EIGENE PREFERENCES ENDE
  
  ;; Who sits where
  (<- (preference tim seat 2)
    (context-prop-amount guests ?guest-count)
    (> ?guest-count 1))
  
  (<- (preference tim seat 1)
    (context-prop-amount guests ?guest-count)
    (crs:== ?guest-count 1))

  (<- (preference mary seat 1))

  ;; EIGENE PERSONEN ANFANG
  (<- (preference max seat 1))
  (<- (preference bob seat 3))
  (<- (preference alice seat 4))
  
  (<- (preference carol seat 4)
    (context-prop-amount guests ?guest-count)
    (> ?guest-count 1))
  
  (<- (preference carol seat 1)
    (context-prop-amount guests ?guest-count)
    (crs:== ?guest-count 1))
  ;; EIGENE PERSONEN ENDE
  
  ;; Objects for meals
  ;; (<- (required-meal-object muesli bowl))
  ;; (<- (required-meal-object muesli muesli))
  (<- (required-meal-object muesli milkbox))
  ;; (<- (required-meal-object muesli spoon))
  
  (<- (required-meal-object bread knife))
  (<- (required-meal-object bread plate))
  (<- (required-meal-object bread cup))
  
  (<- (required-meal-object soup bowl))
  (<- (required-meal-object soup spoon))

  ;; EIGENER STUFF ANFANG
  (<- (required-meal-object cornflakes bowl))
  (<- (required-meal-object cornflakes cornflakes))
  (<- (required-meal-object cornflakes milkbox))
  (<- (required-meal-object cornflakes spoon))

  (<- (required-meal-object pizza plate))
  (<- (required-meal-object pizza knife))
  (<- (required-meal-object pizza fork))
  (<- (required-meal-object pizza pizza_cutter))

  (<- (required-meal-object scrambled-eggs plate))
  (<- (required-meal-object scrambled-eggs fork))
  (<- (required-meal-object scrambled-eggs knife))
  (<- (required-meal-object scrambled-eggs salt))

  (<- (required-meal-object pancakes plate))
  (<- (required-meal-object pancakes knife))
  (<- (required-meal-object pancakes fork))
  (<- (required-meal-object pancakes sugar))
  (<- (required-meal-object pancakes applesauce))

  (<- (required-meal-object milk milkbox))
  
  ;; Object Locations
  (<- (object-position milkbox "fridge1"))
  
  (<- (object-position spoon "drawer1"))

  (<- (object-position bowl "drawer1"))

  (<- (object-position muesli "drawer1"))

  (<- (object-position ?_ "kitchen_sink_block"))
  
  (<- (location-details "fridge1" (in container)))
  (<- (location-details "drawer1" (in drawer)))
  (<- (location-details "kitchen_sink_block" (on Cupboard)))
  
  ;; General rules for table setting object placement
  (<- (center-relative-object-table-position bowl center-of))
  (<- (center-relative-object-table-position plate center-of))
  (<- (center-relative-object-table-position fork left-of))
  (<- (center-relative-object-table-position knife right-of))
  (<- (center-relative-object-table-position spoon left-of))
  (<- (center-relative-object-table-position cup left-of))
  (<- (center-relative-object-table-position cup behind-of))
  (<- (center-relative-object-table-position muesli right-of))
  (<- (center-relative-object-table-position muesli behind-of))
  ;; (<- (center-relative-object-table-position milkbox behind-of))

  ;; EIGENE TABLE-POSITIONEN ANFANG
  (<- (center-relative-object-table-position pizza_cutter behind-of))
  (<- (center-relative-object-table-position cornflakes behind-of))
  (<- (center-relative-object-table-position applesauce behind-of))
  (<- (center-relative-object-table-position applesauce left-of))
  (<- (center-relative-object-table-position sugar right-of))
  (<- (center-relative-object-table-position sugar behind-of))
  ;; EIGENE TABLE-POSITIONEN ENDE
  
  (<- (center-relative-object-table-position ?_ near))
  
  ;; Ordering of costmap-based object placement
  (<- (seat-place-ordering center-of left-of :before))
  (<- (seat-place-ordering center-of right-of :before))
  (<- (seat-place-ordering behind-of ?_ :before))
  (<- (seat-place-ordering near ?_ :after))
  
  (<- (seat-place-ordering ?a ?b :before)
    (seat-place-ordering ?b ?a :after))
  (<- (seat-place-ordering ?a ?b :after)
    (seat-place-ordering ?b ?a :before))
  
  ;; Overall collection predicates
  (<- (required-object ?object)
    (guest ?guest)
    (preference ?guest seat ?seat)
    (seat-description ?seat ?seat-description)
    (preference ?guest dish ?meal)
    (required-meal-object ?meal ?object-type)
    (crs:setof ?position (center-relative-object-table-position
                          ?object-type ?position)
               ?positions)
    (crs:lisp-fun positions->seat-location ?seat ?positions ?location)
    (crs:lisp-fun object-type->object
                  ?object-type ?location ?guest ?object))

  (<- (object-loc ?object ?obj-loc)
    (desig-prop ?object (type ?obj-type))
    (object-position ?obj-type ?obj-loc)))
