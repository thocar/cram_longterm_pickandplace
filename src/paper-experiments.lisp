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
                  ((robot-pose) `(,(cl-tf2:ensure-pose-stamped-transformed
                                    *tf2*
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
    (declare (ignorable obj-1 obj-2 obj-3))
    (with-designators ((obj-4 (object `())))
      (equate obj-3 obj-4))))
