#!/usr/bin/env python

import rospy
import std_msgs
from math import atan2, asin, pi, cos, sin, sqrt, copysign, acos
from numpy import sign, array, matmul, absolute
from numpy.linalg import inv, norm
from sensor_msgs.msg import Imu
import numpy as np
from geometry_msgs.msg import Point, Vector3, Pose, Quaternion, PoseStamped
import geometry_msgs.msg
import tf2_ros
from tf.transformations import euler_from_quaternion
from scipy.spatial.transform import Rotation as R
from scipy import special, linalg
import threading


global q
global R_elbow_left
global R_neck
lu = 0.30 #upper_arm
lf = 0.23 #forward_arm
l9 = 0.15 #clavicle
l3 = 0.2 
l2 = 0.25
l1 = 1.0

x0 = y0 = z0  = 0.0
x2 = y2 = z2 = y3 = z3 =  0.0

q=Quaternion(0,0,0,0)
R_elbow_left =R_back= R_shoulder_right= R_elbow=R_shoulder_left=R_neck= [[1,0,0],[0,1,0],[0,0,1]]
lock = threading.Lock()

#quaternion rotation matrix  
def quaternion_rotation_matrix(w,x,y,z):
    r = R.from_quat([x,y,z,w])
    # First row of the rotation matrix
#    r00 = 2 * (q0 * q0 + q1 * q1) - 1
#    r01 = 2 * (q1 * q2 - q0 * q3)
#    r02 = 2 * (q1 * q3 + q0 * q2)
     
    # Second row of the rotation matrix
#    r10 = 2 * (q1 * q2 + q0 * q3)
#    r11 = 2 * (q0 * q0 + q2 * q2) - 1
#    r12 = 2 * (q2 * q3 - q0 * q1)
     
    # Third row of the rotation matrix
#    r20 = 2 * (q1 * q3 - q0 * q2)
#    r21 = 2 * (q2 * q3 + q0 * q1)
#    r22 = 2 * (q0 * q0 + q3 * q3) - 1
    rr=r.as_dcm() 
    #print np.array(rr)                
     
    # 3x3 rotation matrix
    #rot_matrix = np.array([[r00, r01, r02],
    #                       [r10, r11, r12],
    #                       [r20, r21, r22]])
    return np.array(rr)


#quaternion rotation matrix  
def matrix_rotation_quaternion(mat):
    r = R.from_dcm(mat)

    rr=r.as_quat()
    return np.array(rr)
#neck pose
def neck_pose(roll_neck, pitch_neck, yaw_neck):
    x6 = l3*R_back[0,0] #x
    z6 = l3*R_back[2,0] #z
    y6 = l3*R_back[1,0] #y
    return x6,y6,z6

#angles roll,pitch,yaw (radians)
def ang(q0,q1,q2,q3):
    
    t0 = +2.0 * (q3 * q0 + q1 * q2)
    t1 = +1.0 - 2.0 * (q0 * q0 + q1 * q1)
    roll = atan2(t0, t1)  
    t2 = +2.0 * (q3 * q1 - q2 * q0)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = asin(t2)
    t3 = +2.0 * (q3 * q2 + q0 * q1)
    t4 = +1.0 - 2.0 * (q1 * q1 + q2 * q2)
    yaw = atan2(t3, t4)
    return roll, pitch, yaw 
#IMU50
def callback50(data):
    lock.acquire()
    global R_shoulder_right
    
    #rotation matrix
    R_shoulder_right = quaternion_rotation_matrix(data.orientation.w,data.orientation.x,data.orientation.y, data.orientation.z)
    R_yaw= [[0,1,0],[-1,0,0],[0,0,1]]
    R_shoulder_right=matmul(R_yaw,R_shoulder_right);

    R_neck_inv = inv(R_neck)
    R_trazeno = matmul(R_neck_inv,R_shoulder_right,)
    
    #rotation matrix to quaternion
    qn=matrix_rotation_quaternion(R_trazeno)
    q.w = qn[3]
    q.x = qn[0]
    q.y = qn[1]
    q.z = qn[2]

    norm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z +q.w*q.w)
    #print q, norm
    #transform
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "/neck"
    t.child_frame_id = "/shoulder_right"
    
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = l9
   
    t.transform.rotation.x = q.x/norm
    t.transform.rotation.y = q.y/norm
    t.transform.rotation.z = q.z/norm
    t.transform.rotation.w = q.w/norm
    
    br.sendTransform(t)
    shoulder_right = Pose()
    shoulder_right.orientation = data.orientation
    shoulder_right.position.x = t.transform.translation.x
    shoulder_right.position.y = t.transform.translation.y
    shoulder_right.position.z = t.transform.translation.z

    pub50.publish(shoulder_right)
    lock.release()
#IMU51
def callback51(data):
    lock.acquire()
    global R_wrist_left
    
    R_wrist_left = quaternion_rotation_matrix(data.orientation.w,data.orientation.x,data.orientation.y, data.orientation.z)    
    R_elbow_left_inv = inv(R_elbow_left)
    R_trazeno = matmul(R_elbow_left_inv,R_wrist_left)
    

    qn=matrix_rotation_quaternion(R_trazeno)
    q.w = qn[3]
    q.x = qn[0]
    q.y = qn[1]
    q.z = qn[2]
    norm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z +q.w*q.w)
    #transform
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id ="/elbow_left"
    t.child_frame_id ="/wrist_left"
   
    t.transform.translation.x = lf*R_trazeno[0,0]
    t.transform.translation.y = lf*R_trazeno[1,0]
    t.transform.translation.z = lf*R_trazeno[2,0]
    
    t.transform.rotation.x = q.x/norm
    t.transform.rotation.y = q.y/norm
    t.transform.rotation.z = q.z/norm
    t.transform.rotation.w = q.w/norm
    br.sendTransform(t)
    wrist_left = Pose()
    wrist_left.orientation = data.orientation
    wrist_left.position.x = t.transform.translation.x
    wrist_left.position.y = t.transform.translation.y
    wrist_left.position.z = t.transform.translation.z

    pub51.publish(wrist_left)
    lock.release()
#IMU52    
def callback52(data):
    global R_elbow

    lock.acquire()
    R_elbow = quaternion_rotation_matrix(data.orientation.w,data.orientation.x,data.orientation.y, data.orientation.z)
    R_yaw= [[1,0,0],[0,1,0],[0,0,1]]
    R_elbow=matmul(R_yaw,R_elbow);
    
    R_shoulder_right_inv = inv(R_shoulder_right)
    R_trazeno = matmul(R_shoulder_right_inv,R_elbow)

    #transform
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "/shoulder_right"
    t.child_frame_id ="/elbow_right"
   
    qn=matrix_rotation_quaternion(R_trazeno)
    q.w = qn[3]
    q.x = qn[0]
    q.y = qn[1]
    q.z = qn[2]
    norm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z +q.w*q.w)
    
    t.transform.translation.y = lu*R_trazeno[1,0]
    t.transform.translation.x = lu*R_trazeno[0,0]
    t.transform.translation.z = lu*R_trazeno[2,0] 
    
        
    t.transform.rotation.x = q.x/norm
    t.transform.rotation.y = q.y/norm
    t.transform.rotation.z = q.z/norm
    t.transform.rotation.w = q.w/norm
    
    br.sendTransform(t)
    elbow_right = Pose()
    elbow_right.orientation = data.orientation
    elbow_right.position.x = t.transform.translation.x
    elbow_right.position.y = t.transform.translation.y
    elbow_right.position.z = t.transform.translation.z
    
    pub52.publish(elbow_right)
    lock.release()
#IMU53
def callback53(data):
    global R_elbow_left

    lock.acquire()
    
    R_elbow_left = quaternion_rotation_matrix(data.orientation.w,data.orientation.x,data.orientation.y, data.orientation.z)

    R_shoulder_left_inv = inv(R_shoulder_left)
    R_trazeno = matmul(R_shoulder_left_inv, R_elbow_left)
    
    qn=matrix_rotation_quaternion(R_trazeno)
    q.w = qn[3]
    q.x = qn[0]
    q.y = qn[1]
    q.z = qn[2]
    norm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z +q.w*q.w)
    #transform
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "/shoulder_left"
    t.child_frame_id ="/elbow_left"
    
    t.transform.translation.y = lu*R_trazeno[1,0]
    t.transform.translation.x = lu*R_trazeno[0,0]
    t.transform.translation.z = lu*R_trazeno[2,0] 
    
    t.transform.rotation.x = q.x/norm
    t.transform.rotation.y = q.y/norm
    t.transform.rotation.z = q.z/norm
    t.transform.rotation.w = q.w/norm
    
    br.sendTransform(t)
    elbow_left = Pose()
    elbow_left.orientation = data.orientation
    elbow_left.position.x = t.transform.translation.x
    elbow_left.position.y = t.transform.translation.y
    elbow_left.position.z = t.transform.translation.z

    pub53.publish(elbow_left)
    lock.release()
#IMU54    
def callback54(data):
    global R_shoulder_left
    lock.acquire()
    R_shoulder_left = quaternion_rotation_matrix(data.orientation.w,data.orientation.x,data.orientation.y, data.orientation.z)
    R_yaw= [[0,1,0],[-1,0,0],[0,0,1]]
    R_shoulder_left=matmul(R_yaw,R_shoulder_left);

    R_neck_inv = inv(R_neck)
    R_trazeno = matmul(R_neck_inv,R_shoulder_left)
    
    qn=matrix_rotation_quaternion(R_trazeno)
    q.w = qn[3]
    q.x = qn[0]
    q.y = qn[1]
    q.z = qn[2] 
    norm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z +q.w*q.w)
    
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "/neck"
    t.child_frame_id = "/shoulder_left"
    
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = -l9
    
    t.transform.rotation.x = q.x/norm
    t.transform.rotation.y = q.y/norm
    t.transform.rotation.z = q.z/norm
    t.transform.rotation.w = q.w/norm
    
    br.sendTransform(t)
    shoulder_left = Pose()
    shoulder_left.orientation = data.orientation
    shoulder_left.position.x = t.transform.translation.x
    shoulder_left.position.y = t.transform.translation.y
    shoulder_left.position.z = t.transform.translation.z
    pub54.publish(shoulder_left)
    lock.release()
#IMU55 
def callback55(data):
    global R_wrist
     
    lock.acquire()
    R_wrist = quaternion_rotation_matrix(data.orientation.w,data.orientation.x,data.orientation.y, data.orientation.z)
    R_yaw= [[1,0,0],[0,1,0],[0,0,1]]
    R_wrist=matmul(R_yaw,R_wrist);

    R_elbow_inv = inv(R_elbow)
    R_trazeno = matmul(R_elbow_inv,R_wrist)
    
    
    qn=matrix_rotation_quaternion(R_trazeno)
    q.w = qn[3]
    q.x = qn[0]
    q.y = qn[1]
    q.z = qn[2]
    norm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z +q.w*q.w)
       
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id ="/elbow_right"
    t.child_frame_id ="/wrist_right"
    
    t.transform.translation.x = lf*R_trazeno[0,0] 
    t.transform.translation.y = lf*R_trazeno[1,0] 
    t.transform.translation.z = lf*R_trazeno[2,0]
    
    t.transform.rotation.x = q.x/norm
    t.transform.rotation.y = q.y/norm
    t.transform.rotation.z = q.z/norm
    t.transform.rotation.w = q.w/norm
    
    br.sendTransform(t)
    wrist_right = Pose()
    wrist_right.orientation = data.orientation
    wrist_right.position.x = t.transform.translation.x
    wrist_right.position.y = t.transform.translation.y
    wrist_right.position.z = t.transform.translation.z
    pub55.publish(wrist_right)
    lock.release()
#IMU56
def callback56(data):
    global R_neck
    lock.acquire()
    R_neck = quaternion_rotation_matrix(data.orientation.w,data.orientation.x,data.orientation.y, data.orientation.z)
    R_yaw= [[0,1,0],[-1,0,0],[0,0,1]]
    R_neck=matmul(R_yaw,R_neck);


    R_back_inv = inv(R_back)
    R_trazeno = matmul(R_back_inv,R_neck)

    qn=matrix_rotation_quaternion(R_trazeno)
    q.w = qn[3]
    q.x = qn[0]
    q.y = qn[1]
    q.z = qn[2]
    norm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z +q.w*q.w)
    
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "/back"
    t.child_frame_id = "/neck"
    
    t.transform.translation.x = (l2+l3)
    t.transform.translation.y= 0.0
    t.transform.translation.z = 0.0
    
    t.transform.rotation.x = q.x/norm
    t.transform.rotation.y = q.y/norm
    t.transform.rotation.z = q.z/norm
    t.transform.rotation.w = q.w/norm
    br.sendTransform(t)
    neck = Pose()
    neck.orientation = data.orientation
    neck.position.x = t.transform.translation.x
    neck.position.y = t.transform.translation.y
    neck.position.z = t.transform.translation.z
    pub56.publish(neck)
    lock.release()
#IMU57 
def callback57(data):
    global R_back


    lock.acquire()



    R_back = quaternion_rotation_matrix(data.orientation.w,data.orientation.x,data.orientation.y, data.orientation.z)
    R_yaw= [[1,0,0],[0,1,0],[0,0,1]]
    R_back=matmul(R_yaw,R_back);

    #R = [[1,0,0],[0,1,0],[0,0,1]]
    #R_inv = inv(R)
    R_trazeno = R_back
    #print data.orientation
    qn=matrix_rotation_quaternion(R_trazeno)
    q.w = qn[3]
    q.x = qn[0]
    q.y = qn[1]
    q.z = qn[2]
    norm = sqrt(q.x*q.x + q.y*q.y + q.z*q.z +q.w*q.w)
    
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()
    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "/base_link_body"
    t.child_frame_id = "/back"
    
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = l1
    
    t.transform.rotation.x = q.x/norm
    t.transform.rotation.y = q.y/norm
    t.transform.rotation.z = q.z/norm
    t.transform.rotation.w = q.w/norm

    #print t.transform.rotation
    #print "\n" 
    
    br.sendTransform(t)
    back = Pose()
    back.orientation = data.orientation
    back.position.x = t.transform.translation.x
    back.position.y = t.transform.translation.y
    back.position.z = t.transform.translation.z
    pub57.publish(back)
    lock.release()

if __name__=='__main__':
    rospy.init_node('imu_listener', anonymous=True)
    shoulder_right  = rospy.get_param('~shoulder_right','/imu7')
    wrist_left	=	rospy.get_param('~wrist_left', '/imu5')
    elbow_right = rospy.get_param('~elbow_right','/imu3')
    elbow_left	=	rospy.get_param('~elbow_left' , '/imu4')
    shoulder_left   = rospy.get_param('~shoulder_left','/imu2')
    wrist_right	=	rospy.get_param('~wrist_right', '/imu1')
    neck  = rospy.get_param('~neck','/imu0')
    back = rospy.get_param('~back','/imu6')
    quaternion_rotation_matrix(0,0,1,1)
    #subscribes to topic, type IMU   
    sub50  = rospy.Subscriber(shoulder_right, Imu, callback50)
    sub51  = rospy.Subscriber(wrist_left, Imu, callback51)
    sub52  = rospy.Subscriber(elbow_right, Imu, callback52)
    sub53  = rospy.Subscriber(elbow_left, Imu, callback53)
    sub54  = rospy.Subscriber(shoulder_left, Imu, callback54)
    sub55  = rospy.Subscriber(wrist_right, Imu, callback55)
    sub56  = rospy.Subscriber(neck, Imu, callback56)
    sub57  = rospy.Subscriber(back, Imu, callback57)
    #publishing msgs
    pub50  = rospy.Publisher('shoulder_right_pose', Pose, queue_size=10)
    pub51  = rospy.Publisher('wrist_left_pose', Pose, queue_size=10)
    pub52  = rospy.Publisher('elbow_right_pose', Pose, queue_size=10)
    pub53  = rospy.Publisher('elbow_left_pose', Pose, queue_size=10)
    pub54  = rospy.Publisher('shoulder_left_pose', Pose, queue_size=10)
    pub55  = rospy.Publisher('wrist_right_pose', Pose, queue_size=10)
    pub56  = rospy.Publisher('neck_pose', Pose, queue_size=10)
    pub57  = rospy.Publisher('back_pose', Pose, queue_size=10)
	
    rospy.spin()
