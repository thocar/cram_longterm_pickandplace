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
  (beliefstate:enable-logging t)
  (let ((possible-putdown-locations
          `(;,*loc-on-sink-block*
            ,*loc-on-kitchen-island*)))
    (with-process-modules
      (ensure-arms-up)
      (let ((object *pancake-mix*))
        (loop for i from 0 below runs
              as loc-target = (nth (random
                                    (length possible-putdown-locations))
                                   possible-putdown-locations)
              do (perceive-scene)
                 (pick-object object)
                 (perceive-scene)
                 (place-object object loc-target))))))

  ;; (with-known-object-types ((obj 'desig-props::pancakemix 'desig-props::pancakemix0))
  ;;   (cond ((on-surface-p obj)
  ;;          ;; Handle object on surface here
  ;;          (format t "Getting object from surface~%")
  ;;          (pick-object obj))
  ;;         ((in-container-p obj)
  ;;          ;; Handle object in container here
  ;;          (format t "Getting object from container~%")
  ;;          (open-container (container-for obj)))
  ;;         (t (format t "Invalid object location~%")))))

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
  (beliefstate::enable-logging t)
  (init-ms-belief-state :debug-window t)
  (setf btr::*bb-comparison-validity-threshold* 0.1)
  (moveit:clear-collision-environment)
  ;; Twice, because sometimes a ROS message for an object gets lost.
  (sem-map-coll-env:publish-semantic-map-collision-objects)
  (sem-map-coll-env:publish-semantic-map-collision-objects)
  (top-level
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
           `((desig-props:at ,*loc-on-kitchen-island*);,*loc-on-sink-block*);,*loc-on-cupboard*)
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
                 (tf:make-3d-vector 0.02 0.0 0.0))))))))

;; (defun try-batch-traj ()
;;   (let* ((trajectories-to-check
;;            (multiple-value-bind
;;                  (state trajectory)
;;                (moveit::move-link-pose
;;                 "r_wrist_roll_link"
;;                 "right_arm"
;;                 (tf:make-pose-stamped
;;                  "/base_link"
;;                  (roslisp:ros-time)
;;                  (tf:make-3d-vector 0.6 -0.2 0.9)
;;                  (tf:make-identity-rotation))
;;                 :plan-only t
;;                 :touch-links (pr2-manip-pm::links-for-arm-side :right))
;;              (vector trajectory)))
;;          (result (roslisp:call-service
;;                   "/BatchTrajectoryCheck"
;;                   'robokata-srv::batchtrajectorycheck
;;                   :split_count 20
;;                   :trajectory_count (length trajectories-to-check)
;;                   :group_name (vector "right_arm")
;;                   :trajectories trajectories-to-check)))
;;     (values trajectories-to-check result)))

;; (defun try-batch-traj-2 (valid-traj)
;;   (let* ((trajectories-to-check
;;            (multiple-value-bind
;;                  (state trajectory)
;;                (moveit::move-link-pose
;;                 "r_wrist_roll_link"
;;                 "right_arm"
;;                 (tf:make-pose-stamped
;;                  "/base_link"
;;                  (roslisp:ros-time)
;;                  (tf:make-3d-vector 0.6 -0.2 1.0)
;;                  (tf:make-identity-rotation))
;;                 :plan-only t
;;                 :touch-links (pr2-manip-pm::links-for-arm-side :right))
;;              (vector valid-traj trajectory)))
;;          (result (roslisp:call-service
;;                   "/BatchTrajectoryCheck"
;;                   'robokata-srv::batchtrajectorycheck
;;                   :split_count 20
;;                   :trajectory_count (length trajectories-to-check)
;;                   :group_name (vector "right_arm")
;;                   :trajectories trajectories-to-check)))
;;     (values trajectories-to-check result)))

;; (defun try-batch-traj-3 (traj)
;;   (let* ((result (roslisp:call-service
;;                   "/BatchTrajectoryCheck"
;;                   'robokata-srv::batchtrajectorycheck
;;                   :split_count 20
;;                   :trajectory_count 1
;;                   :group_name (vector "right_arm")
;;                   :trajectories (vector traj))))
;;     result))

;; (defun try-cap-pose ()
;;   (let ((pose (tf:make-pose-stamped
;;                  "/base_link"
;;                  (roslisp:ros-time)
;;                  (tf:make-3d-vector 0.6 -0.2 0.9)
;;                  (tf:make-identity-rotation))))
;;     (roslisp:call-service
;;      "/CapMapPoseQuery"
;;      'robokata-srv::capmapposequery
;;      :query_count 1
;;      :pose_queries
;;      (vector
;;       (make-message "robokata/PoseQuery"
;;                     :group_name "right_arm"
;;                     :pose (tf:pose-stamped->msg pose)
;;                     :pointing_vector (vector 1 0 0)
;;                     :facing_vector (vector 0 0 1)
;;                     :radius 0.3
;;                     :capable_candidates 10)))))

;; (defun try-dual-arm ()
;;   (setf actionlib::*action-server-timeout* 200)
;;   (moveit::move-links-poses
;;    (list "l_wrist_roll_link" "r_wrist_roll_link")
;;    "both_arms"
;;    (list (tf:make-pose-stamped
;;           "/base_link" (roslisp:ros-time)
;;           (tf:make-3d-vector 0.6 0.2 1.0)
;;           (tf:make-identity-rotation))
;;          (tf:make-pose-stamped
;;           "/base_link" (roslisp:ros-time)
;;           (tf:make-3d-vector 0.6 -0.2 1.0)
;;           (tf:make-identity-rotation)))))

;; (defun test-parabel ()
;;   (block check
;;     (flet ((parabel-value (x &key (y-offset 0) (spread 1) (flatness 1))
;;              (+ y-offset (* (* (* x spread) (* spread x)) flatness))))
;;       (let ((frame "/base_link")
;;             (link-name "r_wrist_roll_link")
;;             (planning-group "right_arm")
;;             (vec-start (tf:make-3d-vector 0.6 -0.2 0.9))
;;             (vec-end (tf:make-3d-vector 0.6 -0.4 0.9))
;;             (orientation (tf:make-identity-rotation))
;;             (stepsize 0.1)
;;             (joint-names (vector "r_shoulder_pan_joint" "r_shoulder_lift_joint"
;;                                  "r_upper_arm_roll_joint" "r_elbow_flex_joint"
;;                                  "r_forearm_roll_joint" "r_wrist_flex_joint" "r_wrist_roll_joint")))
;;         (let* ((swap nil)
;;                (vec-start (if swap vec-end vec-start))
;;                (vec-end (if swap vec-start vec-end)))
;;           (flet ((fraction-vector (fraction)
;;                    (tf:v+ vec-start (tf:v* (tf:v- vec-end vec-start) fraction)
;;                           (tf:make-3d-vector
;;                            0 0 (parabel-value fraction
;;                                               :flatness (- (tf:y vec-end)
;;                                                            (tf:y vec-start))))))
;;                  (fraction-pose (frac-vec)
;;                    (tf:make-pose-stamped
;;                     frame (roslisp:ros-time)
;;                     frac-vec orientation))
;;                  (joint-value-from-ik (ik joint-name)
;;                    (with-fields (joint_state) ik
;;                      (with-fields (name position) joint_state
;;                        (let ((jt-pos (position joint-name name :test #'string=)))
;;                          (elt position jt-pos))))))
;;             (let ((last-ik nil))
;;               (make-message
;;                "moveit_msgs/RobotTrajectory"
;;                :joint_trajectory
;;                (make-message
;;                 "trajectory_msgs/JointTrajectory"
;;                 :joint_names joint-names
;;                 :points (map
;;                          'vector
;;                          (lambda (x)
;;                            (let* ((frac-vec (fraction-vector x))
;;                                   (frac-pose (fraction-pose frac-vec))
;;                                   (ik (moveit:compute-ik
;;                                        link-name planning-group frac-pose
;;                                        :robot-state last-ik)))
;;                              (unless ik
;;                                (return-from check))
;;                              (prog1
;;                                  (let ((traj-pt
;;                                          (make-message
;;                                           "trajectory_msgs/JointTrajectoryPoint"
;;                                           :time_from_start (* x 5)
;;                                           :positions
;;                                           (map 'vector (lambda (joint-name)
;;                                                          (joint-value-from-ik ik joint-name))
;;                                                joint-names))))
;;                                    traj-pt)
;;                                (setf last-ik ik))))
;;                          (loop for x from -1.0 to 1.0 by stepsize
;;                                collect x)))))))))))

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
