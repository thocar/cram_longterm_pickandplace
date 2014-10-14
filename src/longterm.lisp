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

(defun generate-environment-location ()
  "Generated a location designator with random properties as defined
in the function."
  (let* ((possible-locations
           `(,*loc-on-sink-block*
             ,*loc-on-kitchen-island*
             ,*loc-on-cupboard*)))
    (nth (random
          (length possible-locations))
         possible-locations)))

(defun generate-object-acted-on (&key filter-function)
  "Generated an object designator with random properties as defined in
the function."
  (block generator-block
    (loop while t
          for object = (make-designator
                        'object
                        (append (prob-prop 'desig-props::type
                                           `(desig-props::pancakemix
                                             desig-props::dinnerplate
                                             desig-props::muesli
                                             (desig-props::ketchup 0.2))
                                           0.8)
                                (prob-prop 'desig-props::color
                                           `(desig-props::red
                                             (desig-props::blue 0.1)
                                             desig-props::green
                                             desig-props::yellow
                                             desig-props::white)
                                           0.4)
                                (prob-prop 'desig-props::shape
                                           `(desig-props::round
                                             desig-props::flat
                                             desig-props::box)
                                           0.2)
                                `((desig-props:at
                                   ,(generate-environment-location)))))
          when (or (not filter-function)
                   (not (funcall filter-function object)))
            do (return-from generator-block object))))

(defun invalid-object-filter-function (object)
  "Returns `T' when the object passed as `object' is invalid. Valid objects are:
 - `red', `round' `ketchup'
 - `yellow', `box', `muesli'
 - `white', `flat', `dinnerplate'"
  (let ((type (desig-prop-value object 'type))
        (color (desig-prop-value object 'color))
        (shape (desig-prop-value object 'shape)))
    (not (or (and (eql type 'ketchup)
                  (eql color 'red)
                  (eql shape 'round))
             (and (eql type 'muesli)
                  (eql color 'yellow)
                  (eql shape 'box))
             (and (eql type 'dinnerplate)
                  (eql color 'white)
                  (eql shape 'flat))))))

(def-top-level-cram-function perform-generated-task ()
  (beliefstate:enable-logging nil)
  (prepare-settings)
  (beliefstate:enable-logging t)
  (with-process-modules
    (ensure-arms-up)
    (try-forever
      (let* ((object (generate-object-acted-on)))
        (try-n-times 3
          (pick-object object))
        (try-forever
          (let* ((location (generate-environment-location)))
            (try-n-times 3
              (place-object object location))))))))

(def-top-level-cram-function longterm (&optional (runs 1))
  (with-process-modules
    (loop for i from 0 below runs
          do (ensure-arms-up)
             (try-forever
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
    (try-forever
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
