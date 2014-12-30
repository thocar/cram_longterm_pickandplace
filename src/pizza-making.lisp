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
;;; Environment
;;;

(defvar *oven* nil)
(defvar *fridge* nil)
(defvar *drawer* nil)
(defvar *workplace* nil)
(defvar *pizza-dough* nil)
(defvar *pizza* nil)
(defvar *sauce-bottle* nil)
(defvar *spoon* nil)

;;;
;;; Helpers
;;;

(defun set-pizza-making-variables ()
  (setf *oven* (an object `((type oven)
                            (name "kitchen_oven")
                            (subparts
                             ((,(an object `((type door)
                                             (name "kitchen_oven_door"))))))
                            (controls
                             ((,(an object `((type knob)
                                             (for temperature-ctrl)
                                             (state off)))))))))
  (setf *fridge* (an object `((type fridge)
                              (name "kitchen_fridge")
                              (subparts
                               ((,(an object `((type door)
                                               (name "kitchen_fridge_door")))))))))
  (setf *drawer* (an object `((type drawer)
                              (name "kitchen_drawer")
                              (subparts
                               ((,(an object `((type drawer)
                                               (name "kitchen_fridge_drawer")))))))))
  (setf *workplace* (a location `((on Cupboard)
                                  (name "kitchen_island"))))
  (setf *pizza-dough*
        (an object `((type pizza-dough)
                     (at ,*fridge*))))
  (setf *pizza* (an object `((type pizza)
                             (shape round)
                             (shape flat)
                             (at ,*workplace*))))
  (setf *sauce-bottle* (an object `((type bottle)
                                    (shape round)
                                    (color red)
                                    (at ,*fridge*))))
  (setf *spoon* (an object `((type spoon)
                             (shape flat)
                             (color white)
                             (at ,*drawer*)))))

(defun get-list-of-ingredients (task)
  (when (string= task "pizza-making")
    `(,*pizza-dough* ,*sauce-bottle*)))

(defun get-list-of-tools (task)
  (when (string= task "pizza-making")
    `(,*spoon*)))

;;;
;;; Plans
;;;

(defgeneric articulate-environment (indicator location))

(defmethod articulate-environment (indicator (location location-designator))
  (ros-info (pizza-making) "Articulating location ~a (~a)"
            location indicator))

(defmethod articulate-environment (indicator (object object-designator))
  (ros-info (pizza-making) "Articulating object ~a (~a)"
            object indicator))

(def-cram-function gather-tools-and-ingredients (task)
  (let* ((ingredients (get-list-of-ingredients task))
         (tools (get-list-of-tools task))
         (objects (append ingredients tools)))
    (dolist (object objects)
      (let ((at (desig-prop-value object 'at)))
        (articulate-environment 'prepare at)
        ;; (pick-object object)
        (articulate-environment 'cleanup at)))))

(def-cram-function prepare-pizza ()
  )

(def-cram-function put-pizza-in-oven ()
  )

(def-cram-function operate-oven ()
  )

(def-cram-function take-pizza-out-of-oven ()
  )

(def-cram-function serve-pizza ()
  )

(def-top-level-cram-function make-pizza ()
  (set-pizza-making-variables)
  (gather-tools-and-ingredients "pizza-making")
  (prepare-pizza)
  (put-pizza-in-oven)
  (operate-oven)
  (take-pizza-out-of-oven)
  (serve-pizza))
