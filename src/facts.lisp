;;; Copyright (c) 2014, Jan Winkler <winkler@cs.uni-bremen.de>
;;; All rights reserved.
;;; 
;;; Redistribution and use in source and binary forms, with or without
;;; modification, are permitted provided that the following conditions are met:
;;; 
;;;     * Redistributions of source code must retain the above copyright
;;;       notice, this list of conditions and the following disclaimer.
;;;     * Redistributions in binary form must reproduce the above copyright
;;;       notice, this list of conditions and the following disclaimer in the
;;;       documentation and/or other materials provided with the distribution.
;;;     * Neither the name of Universität Bremen nor the names of its
;;;       contributors may be used to endorse or promote products derived from
;;;       this software without specific prior written permission.
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

(defun make-area-restriction-cost-function ()
  (let ((min-x -1.0)
        (max-x 1.5)
        (min-y 0.25)
        (max-y 1.7))
    (lambda (x y)
      (if (and (>= x min-x)
               (<= x max-x)
               (>= y min-y)
               (<= y max-y))
          1.0d0
          0.0d0))))
  ;; (let ((min-x -1.1)
  ;;       (min-y -0.95)
  ;;       (max-x  1.7)
  ;;       (max-y  2.7)
  ;;       (sink-block-min-y 0.0)
  ;;       (sink-block-max-y 2.0))
  ;;   (lambda (x y)
  ;;     (if (> x max-x)
  ;;         0.0d0 ;; Invalid due to too large x
  ;;         (if (and (> x 0.0)
  ;;                  (> y min-y)
  ;;                  (< y sink-block-max-y)
  ;;                  (> y sink-block-min-y))
  ;;             1.0d0 ;; Valid region on sink block
  ;;             (if (and (<= x 0.0) ;; Other side of kitchen
  ;;                      (> x min-x)
  ;;                      (> y min-y)
  ;;                      (< y max-y))
  ;;                 (if (> y 0.4) ;; On kitchen island side
  ;;                     (if (and (< x -0.6) ;; Lower boundary for kitchen island
  ;;                              (> x -0.7))
  ;;                         0.0d0
  ;;                         (if (and (> y 0.65)
  ;;                                  (< y 0.8))
  ;;                             1.0d0
  ;;                             0.0d0))
  ;;                     (if (and (< x -0.6) ;; Lower boundary for pancake table
  ;;                              (> x -0.95))
  ;;                         0.0d0
  ;;                         1.0d0))
  ;;                 0.0d0)))))

(defmethod costmap-generator-name->score ((name (common-lisp:eql 'area-restriction-distribution)))
  7)

(def-fact-group demo-costmap-desigs (desig-costmap)

  (<- (desig-costmap ?desig ?cm)
    (or (desig-prop ?desig (desig-props:to desig-props:see))
        (desig-prop ?desig (desig-props:to desig-props:reach))
        (desig-prop ?desig (desig-props:on ?_)))
    (costmap ?cm)
    (costmap-add-function area-restriction-distribution
                          (make-area-restriction-cost-function)
                          ?cm)))
