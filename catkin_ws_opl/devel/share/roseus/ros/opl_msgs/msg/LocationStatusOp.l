;; Auto-generated. Do not edit!


(when (boundp 'opl_msgs::LocationStatusOp)
  (if (not (find-package "OPL_MSGS"))
    (make-package "OPL_MSGS"))
  (shadow 'LocationStatusOp (find-package "OPL_MSGS")))
(unless (find-package "OPL_MSGS::LOCATIONSTATUSOP")
  (make-package "OPL_MSGS::LOCATIONSTATUSOP"))

(in-package "ROS")
;;//! \htmlinclude LocationStatusOp.msg.html


(defclass opl_msgs::LocationStatusOp
  :super ros::object
  :slots (_location _cube_at_ds _op_cube_id ))

(defmethod opl_msgs::LocationStatusOp
  (:init
   (&key
    ((:location __location) "")
    ((:cube_at_ds __cube_at_ds) nil)
    ((:op_cube_id __op_cube_id) "")
    )
   (send-super :init)
   (setq _location (string __location))
   (setq _cube_at_ds __cube_at_ds)
   (setq _op_cube_id (string __op_cube_id))
   self)
  (:location
   (&optional __location)
   (if __location (setq _location __location)) _location)
  (:cube_at_ds
   (&optional __cube_at_ds)
   (if __cube_at_ds (setq _cube_at_ds __cube_at_ds)) _cube_at_ds)
  (:op_cube_id
   (&optional __op_cube_id)
   (if __op_cube_id (setq _op_cube_id __op_cube_id)) _op_cube_id)
  (:serialization-length
   ()
   (+
    ;; string _location
    4 (length _location)
    ;; bool _cube_at_ds
    1
    ;; string _op_cube_id
    4 (length _op_cube_id)
    ))
  (:serialize
   (&optional strm)
   (let ((s (if strm strm
              (make-string-output-stream (send self :serialization-length)))))
     ;; string _location
       (write-long (length _location) s) (princ _location s)
     ;; bool _cube_at_ds
       (if _cube_at_ds (write-byte -1 s) (write-byte 0 s))
     ;; string _op_cube_id
       (write-long (length _op_cube_id) s) (princ _op_cube_id s)
     ;;
     (if (null strm) (get-output-stream-string s))))
  (:deserialize
   (buf &optional (ptr- 0))
   ;; string _location
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _location (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;; bool _cube_at_ds
     (setq _cube_at_ds (not (= 0 (sys::peek buf ptr- :char)))) (incf ptr- 1)
   ;; string _op_cube_id
     (let (n) (setq n (sys::peek buf ptr- :integer)) (incf ptr- 4) (setq _op_cube_id (subseq buf ptr- (+ ptr- n))) (incf ptr- n))
   ;;
   self)
  )

(setf (get opl_msgs::LocationStatusOp :md5sum-) "d7e59fda5792b384bfe9ef98e01f915a")
(setf (get opl_msgs::LocationStatusOp :datatype-) "opl_msgs/LocationStatusOp")
(setf (get opl_msgs::LocationStatusOp :definition-)
      "string location
bool cube_at_ds
string op_cube_id

")



(provide :opl_msgs/LocationStatusOp "d7e59fda5792b384bfe9ef98e01f915a")


