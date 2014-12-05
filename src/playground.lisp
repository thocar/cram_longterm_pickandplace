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
                         (cl-tf2:ensure-pose-stamped-transformed
                          (tf:pose->pose-stamped
                           gripper-frame
                           (roslisp:ros-time)
                           (tf:make-identity-pose))
                          "/map" :use-current-ros-time t)))
                   (push
                    gripper-pose
                    gripper-poses)))))
    gripper-poses))

(plan-lib::declare-goal parameters-logged (action object))

(plan-lib::def-goal (achieve (parameters-logged ?action ?object))
  (format t "Got:~%~a~%~a~%" ?action ?object))

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
                  0.0d0)))
      1.0d0)))

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
