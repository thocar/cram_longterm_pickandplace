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

(define-condition recursion-too-deep () ())

(defclass property ()
  ((name :reader name :initarg :name)
   (sub-properties :reader sub-properties :initarg :sub-properties)
   (singular-property :reader singular-property :initarg :singular-property)
   (transient-property :reader transient-property :initarg :transient-property)
   (optional-property :reader optional-property :initarg :optional-property)
   (possible-values :reader possible-values :initarg :possible-values)))

(defclass value-container ()
  ((value-type :reader value-type :initarg :value-type)
   (contained-value :reader contained-value :initarg :contained-value)))

(defun a-reference-object ()
  (make-instance
   'value-container
   :value-type :object))

(defun a-reference-location (&optional class)
  (make-instance
   'value-container
   :value-type :location
   :contained-value class))

(defun a-pose (&key in-hand)
  (make-instance
   'value-container
   :value-type :pose
   :contained-value (when in-hand :in-hand)))

(defun a-value (value &optional value-type)
  (make-instance
   'value-container
   :value-type value-type
   :contained-value value))

(defun define-property (name &key
                               sub-properties
                               possible-values
                               singular-property
                               transient-property
                               (optional-property nil optional-property-p))
  (make-instance
   'property
   :name name
   :singular-property singular-property
   :sub-properties sub-properties
   :transient-property transient-property
   :optional-property (cond (optional-property-p optional-property)
                            (t (= (length possible-values) 0)))
   :possible-values possible-values))

(defun object-property-tree ()
  (define-property 'object
    :sub-properties
    (list (define-property 'physical
            :transient-property t
            :sub-properties
            (list (define-property 'at
                    :possible-values (list (a-reference-location)))
                  (define-property 'dimensions)
                  (define-property 'texture)
                  (define-property 'weight)))
          (define-property 'perceptual
            :transient-property t
            :sub-properties
            (list (define-property 'size
                    :possible-values
                    (list (a-value 'small)
                          (a-value 'medium)
                          (a-value 'big)))
                  (define-property 'shape
                    :possible-values
                    (list (a-value 'box)
                          (a-value 'cylinder)
                          (a-value 'round)
                          (a-value 'flat)))
                  (define-property 'logo)
                  (define-property 'text)
                  (define-property 'color
                    :possible-values
                    (list (a-value 'red)
                          (a-value 'green)
                          (a-value 'yellow)
                          (a-value 'blue)
                          (a-value 'white)
                          (a-value 'black)))))
          (define-property 'meta
            :transient-property t
            :sub-properties
            (list (define-property 'type
                    :possible-values
                    (list (a-value 'object)
                          (a-value 'spatula)
                          (a-value 'pancakemix)))))
          (define-property 'affordances
            :transient-property t
            :sub-properties
            (list (define-property 'grasp-type
                    :possible-values
                    (list (a-value 'push)
                          (a-value 'slide-down)))
                  (define-property 'handle))))))

(defun location-property-tree ()
  (define-property 'location
    :singular-property t
    :sub-properties
    (list (define-property 'spatial-term
            :transient-property t
            :sub-properties
            (list (define-property 'in
                    :singular-property t
                    :sub-properties
                    (list (define-property 'container
                            :transient-property t
                            :possible-values
                            (list (a-value 'drawer)
                                  (a-value 'fridge)))))
                  (define-property 'on
                    :singular-property t
                    :sub-properties
                    (list (define-property 'surface
                            :transient-property t
                            :possible-values
                            (list (a-value 'cupboard)
                                  (a-value 'counter)))))
                                  ;(a-value 'table)))))
                  (define-property 'name
                    :possible-values
                    (list (a-value "kitchen_sink_block")
                          (a-value "kitchen_island")))
                  (define-property 'left-of
                    :possible-values (list (a-reference-object)))
                  (define-property 'right-of
                    :possible-values (list (a-reference-object)))
                  (define-property 'near
                    :possible-values (list (a-reference-object)))
                  (define-property 'behind
                    :possible-values (list (a-reference-object)))
                  (define-property 'in-front-of
                    :possible-values (list (a-reference-object)))))
          (define-property 'to
            :singular-property t
            :sub-properties
            (list (define-property 'see
                    :possible-values
                    (list (a-reference-location)
                          (a-reference-object)))
                  (define-property 'reach
                    :possible-values
                    (list (a-reference-location)
                          (a-reference-object)))))
          (define-property 'pose-in-hand
            :possible-values (list (a-pose :in-hand t)))
          (define-property 'pose
            :possible-values (list (a-pose))))))

(defun task-property-tree ()
  (define-property 'task
    :singular-property t
    :transient-property t
    :sub-properties
    (list (define-property 'fetch
            :singular-property t
            :sub-properties
            (list (define-property 'reach
                    :transient-property t
                    :possible-values (list (a-reference-object)))
                  (define-property 'grasp
                    :transient-property t
                    :possible-values (list (a-reference-object)))
                  (define-property 'lift
                    :transient-property t
                    :possible-values (list (a-reference-object)))
                  (define-property 'hold
                    :transient-property t
                    :possible-values (list (a-reference-object)))))
          (define-property 'place
            :singular-property t
            :possible-values (list (a-reference-location))
            :sub-properties
            (list (define-property 'reach
                    :transient-property t
                    :possible-values (list (a-reference-object)))
                  (define-property 'put
                    :transient-property t
                    :possible-values (list (a-reference-object)))))
          (define-property 'perceive
            :singular-property t
            :transient-property t
            :sub-properties
            (list (define-property 'detect
                    :possible-values (list (a-reference-object)))
                  (define-property 'examine
                    :possible-values (list (a-reference-object)))
                  (define-property 'perceive-scene))))))

(defun print-property-tree (property &key (indent-level 0))
  (flet ((indent-string (string indentation)
           (let ((indentation (* 2 indentation)))
             (reduce (lambda (x1 x2) (concatenate 'string x1 x2))
                     (append (loop for i from 0 below indentation
                                   collect " ")
                             `(,string))))))
    (format t (indent-string
               (format nil "Property: ~a~a~%" (name property)
                       (cond ((singular-property property)
                              (format nil " (singular)"))
                             (t "")))
               indent-level))
    (when (possible-values property)
      (format t (indent-string "Possible Values:~%" indent-level))
      (dolist (possible-value (possible-values property))
        (cond ((eql (value-type possible-value) :object)
               (format t (indent-string "[Reference Object]~%" (1+ indent-level))))
              ((eql (value-type possible-value) :location)
               (format t (indent-string "[Reference Location]~%" (1+ indent-level))))
              ((eql (value-type possible-value) :pose)
               (format t (indent-string "[Pose]~%" (1+ indent-level))))
              ((eql (value-type possible-value) :property)
               (print-property-tree (contained-value possible-value)
                                    :indent-level (1+ indent-level)))
              (t (format t (indent-string (format nil "~a~%" (contained-value possible-value))
                                          (1+ indent-level)))))))
    (when (sub-properties property)
      (format t (indent-string "Sub-Properties:~%" indent-level))
      (dolist (sub-property (sub-properties property))
        (print-property-tree sub-property :indent-level (+ indent-level 1))))))

(defun print-sample (sample &key (indent-level 0) (stream t))
  (flet ((indent-string (string indentation)
           (let ((indentation (* 2 indentation)))
             (reduce (lambda (x1 x2) (concatenate 'string x1 x2))
                     (append (loop for i from 0 below indentation
                                   collect " ")
                             `(,string))))))
    (let* ((prop-name (name sample))
           (prop-value (first (sub-properties sample)))
           (prop-sub-name (when prop-value (name prop-value))))
      (cond ((eql prop-name 'task)
             (format stream (indent-string
                             (format nil "~a: ~a~%" prop-name prop-sub-name)
                             indent-level))
             (when (possible-values prop-value)
               (dolist (possible-value (possible-values prop-value))
                 (cond ((eql (value-type possible-value) :pose)
                        (format stream (indent-string "[Pose]~%" indent-level)))
                       ((or (eql (value-type possible-value) :property)
                            (eql (value-type possible-value) :object)
                            (eql (value-type possible-value) :location))
                        (print-sample (contained-value possible-value)
                                      :indent-level indent-level
                                      :stream stream))
                       (t (format stream (indent-string
                                          (format nil "~a~%" (contained-value
                                                              possible-value))
                                          indent-level))))))
             (when (sub-properties prop-value)
               (dolist (sub-property (sub-properties prop-value))
                 (print-property-tree sub-property :indent-level (+ indent-level 1)))))
            (t (when prop-name
                 (when (< (length (possible-values sample)) 1)
                   (when (> (length (sub-properties sample)) 0)
                     (format stream (indent-string (format nil "~a~%" prop-name)
                                                   indent-level))))
                 (let* ((possible-value (first (possible-values sample))))
                   (when possible-value
                     (cond ((eql (value-type possible-value) :pose)
                            (format stream (indent-string
                                            (format nil "~a = [Pose]~%"
                                                    (name sample))
                                            indent-level)))
                           ((or (eql (value-type possible-value) :property)
                                (eql (value-type possible-value) :object)
                                (eql (value-type possible-value) :location))
                            (format stream (indent-string (format nil "~a~%" prop-name) indent-level))
                            (print-sample (contained-value possible-value)
                                          :indent-level (1+ indent-level)
                                          :stream stream))
                           (t (format stream (indent-string
                                              (format nil "~a = ~a~%"
                                                      (name sample)
                                                      (contained-value
                                                       possible-value))
                                              indent-level)))))))
                 (when (sub-properties sample)
                   (dolist (sub-property (sub-properties sample))
                     (print-sample
                      sub-property
                      :indent-level (+ indent-level 1)
                      :stream stream))))))))

(defun resolved-property-parameters (property)
  (let ((parameters
          (append
           (mapcar (lambda (val)
                     (let ((val-type (value-type val))
                           (cval (contained-value val)))
                       (cond ((eql val-type :object)
                              (make-designator
                               'object (resolved-property-parameters cval)))
                             ((eql val-type :location)
                              (make-designator
                               'location (resolved-property-parameters cval)))
                             ((eql val-type :property)
                              (resolved-property-parameters cval))
                             (t cval))))
                   (possible-values property))
           (remove-if
            #'not
            (mapcar (lambda (prop)
                      (when (resolved-property-parameters prop)
                        `(,(name prop) ,@(resolved-property-parameters prop))))
                    (sub-properties property))))))
    parameters))

(defun sample->actions (sample)
  (let ((prop-name (name sample)))
    (cond ((eql prop-name 'task)
           (format t "~a~%" prop-name)
           (let* ((task (first (sub-properties sample)))
                  (task-name (name task)))
             (format t "~a~%" task-name)
             (cond (t
                    (resolved-property-parameters task)))))
          (t prop-name))))

(defun endure-sample (&optional (tree (task-property-tree)))
  (loop for sample = (sample tree)
        until sample
        finally (return sample)))

(defun sample (property &optional (level 2))
  (catch 'recursion-too-deep
    (handler-bind ((recursion-too-deep
                     (lambda (condition)
                       (declare (ignore condition))
                       (when (= level 2)
                         (return-from sample nil)))))
      (when (> level 50)
        (error 'recursion-too-deep))
      (let ((singular (singular-property property))
            (name (name property))
            (sub-properties (sub-properties property))
            (possible-values (possible-values property)))
        (let ((sub-properties
                (cond (singular
                       (let ((sampled
                               (sample (nth (random (length sub-properties))
                                            sub-properties)
                                       (1+ level))))
                         (when sampled (list sampled))))
                      (t (loop for sub-property in sub-properties
                               for sampled = (sample sub-property (1+ level))
                               when (and (= (random 2) 0) sampled)
                                 collect sampled))))
              (possible-values
                (when possible-values (list (nth (random (length possible-values)) possible-values)))))
          (let ((sub-properties
                  (loop for sub-property in sub-properties
                        for transient = (transient-property sub-property)
                        if transient
                          append (sub-properties sub-property) into transient-sub-properties
                        else
                          collect sub-property into originary-sub-properties
                        finally
                           (return (append originary-sub-properties transient-sub-properties))))
                (possible-values
                  (loop for sub-property in sub-properties
                        for transient = (transient-property sub-property)
                        if transient
                          append (possible-values sub-property) into transient-possible-values
                        finally
                           (return (append possible-values transient-possible-values)))))
            (define-property name
              :sub-properties sub-properties
              :possible-values
              (mapcar (lambda (value)
                        (let ((type (value-type value)))
                          (cond ((eql type :location)
                                 (a-value (sample (location-property-tree) (1+ level)) :location))
                                ((eql type :object)
                                 (a-value (sample (object-property-tree) (1+ level)) :object))
                                (t value))))
                      possible-values)
              :transient-property (transient-property property))))))))

(defun sample-task ()
  (let ((object (make-designator 'object (resolved-property-parameters
                                          (endure-sample (object-property-tree)))))
        (location (make-designator 'location (resolved-property-parameters
                                              (endure-sample (location-property-tree))))))
    (format t "Fetch: ~a~%" object)
    (format t "Place it at: ~a~%" location)))
