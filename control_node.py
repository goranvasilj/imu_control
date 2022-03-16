#!/usr/bin/env python

import rospy
import tf2_ros

from geometry_msgs.msg import Pose, Point, Vector3, PoseStamped
import geometry_msgs.msg
from std_msgs.msg import Float64
from math import atan2,asin, pi, cos, sin,sqrt,acos
from std_msgs.msg import String
import math
from sensor_msgs.msg import JointState

global current_pose
global pose_ref, pose_ref_left, pose_stamped, pose_ref_back, pose_ref_neck, pose_ref_shoulder_left, pose_ref_elbow_left, p_elbow_left
global joint1,joint2,joint3,joint4,joint5
global start_time
global joint_state 
global left_wrist, right_wrist, left_elbow, right_elbow, neck_tf
global status

pose_stamped = PoseStamped()
current_pose = Pose()
pose_ref = pose_ref_left = pose_ref_neck = pose_ref_back = pose_ref_elbow_left = pose_ref_shoulder_left = Pose()
joint1 = joint2 = joint3 = joint4 = joint5 =  Float64()
joint_state = JointState()

def callback1(data):
    global pose_ref
    pose_ref = data
def callback2(data):
    #data from uav
    global pose_stamped
    pose_stamped = data
def callback3(data):
    global pose_ref_left
    pose_ref_left = data   
def callback4(data):
    global pose_ref_neck
    pose_ref_neck = data
def callback5(data):
    global pose_ref_back
    pose_ref_back = data
def callback6(data):
    global pose_ref_elbow_left
    pose_ref_elbow_left = data
def callback7(data):
    global pose_ref_shoulder_left
    pose_ref_shoulder_left = data
def callback8(data):
    global joint_state
    joint_state = data
   
def callback_voice(data):
    global status
    print data
    if (data.data == "freeze "):
       if (status == "frozen"):
          status = "driving"
       else:
          status = "frozen"
    if (data.data == "slow "):
       status="slow"
    if (data.data == "stop "):
       print "land"
       state = "frozen"
       current_pose.position.x = pose_stamped.pose.position.x 
       current_pose.position.y = pose_stamped.pose.position.y
       current_pose.position.z = 0
       current_pose.orientation.x = 0
       current_pose.orientation.y = 0
       current_pose.orientation.z = sin(yaw_command/2)
       current_pose.orientation.w = cos(yaw_command/2)    
       pub_pose.publish(current_pose)   
    if (data.data == "begin "):
       state = "frozen"
       current_pose.position.x = pose_stamped.pose.position.x 
       current_pose.position.y = pose_stamped.pose.position.y
       current_pose.position.z = 1
       current_pose.orientation.x = 0
       current_pose.orientation.y = 0
       current_pose.orientation.z = sin(yaw_command/2)
       current_pose.orientation.w = cos(yaw_command/2)    
       pub_pose.publish(current_pose)   

#publisher
pub_pose = rospy.Publisher('/uav/pose_ref',Pose,queue_size=10)
pub1 = rospy.Publisher('/uav/joint1_position_controller/command',Float64,queue_size=10)
pub2 = rospy.Publisher('/uav/joint2_position_controller/command',Float64,queue_size=10)
pub3 = rospy.Publisher('/uav/joint3_position_controller/command',Float64,queue_size=10)
pub4 = rospy.Publisher('/uav/joint4_position_controller/command',Float64,queue_size=10)
pub5 = rospy.Publisher('/uav/joint5_position_controller/command',Float64,queue_size=10)
#susbcriber
sub1 = rospy.Subscriber('wrist_right_pose',Pose,callback1)
sub2 = rospy.Subscriber('/uav/pose',PoseStamped,callback2)
sub3 = rospy.Subscriber('wrist_left_pose',Pose,callback3)
sub4 = rospy.Subscriber('/neck_pose',Pose,callback4)
sub5 = rospy.Subscriber('/back_pose',Pose,callback5)
sub6 = rospy.Subscriber('elbow_left_pose',Pose,callback6)
sub7 = rospy.Subscriber('shoulder_left_pose',Pose,callback7)
sub8 = rospy.Subscriber('/uav/joint_states',JointState,callback8)
sub9 = rospy.Subscriber('/voice_recognition',String,callback_voice)

global tfBuffer
global listener
global yaw_command

dead_zone=0.2
def calculate_based_on_dead_zone(value,zone):
   if (value>0):
      if (value < zone):
         value = 0
      else:
         value=value-zone
   else:
      if (value > -zone):
         value = 0
      else:
         value=value+zone
   return value

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
def up(scale):
    global right_wrist
    current_pose.position.x = pose_stamped.pose.position.x 
    current_pose.position.y = pose_stamped.pose.position.y

   # print right_wrist.transform.translation.x
    if(right_wrist.transform.translation.x < 0.0):
                if (right_wrist.transform.translation.x>-dead_zone):
                   right_wrist.transform.translation.x=0;
                else:
                   right_wrist.transform.translation.x=right_wrist.transform.translation.x+dead_zone;
    else:
                if (right_wrist.transform.translation.x<dead_zone):
                   right_wrist.transform.translation.x=0;
                else:
                   right_wrist.transform.translation.x=right_wrist.transform.translation.x-dead_zone;

    current_pose.position.z = pose_stamped.pose.position.z+right_wrist.transform.translation.x/scale
    #print right_wrist.transform.translation.x
    yaw=2*math.acos(pose_stamped.pose.orientation.w);
#    current_pose.orientation.x = 0
#    current_pose.orientation.y = 0
#    current_pose.orientation.z = sin(yaw)
#    current_pose.orientation.w = cos(yaw)
    
    
def right(scale):
    global right_wrist, yaw_command
    if(right_wrist.transform.translation.z < 0.0):
                if (right_wrist.transform.translation.z>-dead_zone):
                   right_wrist.transform.translation.z=0;
                else:
                   right_wrist.transform.translation.z=right_wrist.transform.translation.z+dead_zone;
    else:
                if (right_wrist.transform.translation.z<dead_zone):
                   right_wrist.transform.translation.z=0;
                else:
                   right_wrist.transform.translation.z=right_wrist.transform.translation.z-dead_zone;
    yaw=2*math.acos(pose_stamped.pose.orientation.w);
    yaw_command=yaw_command+right_wrist.transform.translation.z/30/scale
#    current_pose.position.x = current_pose.position.x+cos(yaw)*right_wrist.transform.translation.z
#    current_pose.position.y = current_pose.position.y+sin(yaw)*right_wrist.transform.translation.z
#    yaw=yaw+right_wrist.transform.translation.z/100
    print "yaw"
    print yaw_command

    current_pose.orientation.x = 0
    current_pose.orientation.y = 0
    current_pose.orientation.z = sin(yaw_command/2)
    current_pose.orientation.w = cos(yaw_command/2)



def forward(scale):
    global neck_tf, yaw_command



    x=math.sqrt(neck_tf.transform.translation.x*neck_tf.transform.translation.x+neck_tf.transform.translation.y*neck_tf.transform.translation.y)
    #print neck_tf
    print x
    x=calculate_based_on_dead_zone(x, dead_zone/1.5)
    print x
    current_pose.position.x = current_pose.position.x + cos(yaw_command) * x*2/scale
    current_pose.position.y = current_pose.position.y + sin(yaw_command) * x*2/scale
    current_pose.position.z = current_pose.position.z
 #   yaw=2*math.acos(current_pose.orientation.w);
 #   current_pose.orientation.x = 0
 #   current_pose.orientation.y = 0
 #   current_pose.orientation.z = sin(yaw)
 #   current_pose.orientation.w = cos(yaw)

def move_robot_arm():

       joint1 = atan2(-left_elbow.transform.translation.z, left_elbow.transform.translation.x)
       joint2 = atan2(left_elbow.transform.translation.y, sqrt(left_elbow.transform.translation.x**2 + left_elbow.transform.translation.z**2))
       joint3 =0.0#atan2(sqrt(((cos(y_elbow_left)*sin(p_elbow_left))**2) + ((sin(y_elbow_left)*sin(p_elbow_left))**2)),cos(p_elbow_left))
       joint4 = 0.0#atan2(cos(y_elbow_left)*sin(p_elbow_left),sin(y_elbow_left)*sin(p_elbow_left)) 
       joint5 = 0.0

       pub1.publish(joint1)
       pub2.publish(joint2)
       pub3.publish(joint3)
       pub4.publish(joint4)
       pub5.publish(joint5)



       
def move_uav():
       global r_left, p_elbow_left, y_elbow_left, right_wrist, left_wrist, right_elbow, left_elbow, neck_tf
       global tfBuffer
       global status
       r_left = r_elbow_left= y_elbow_left=0.0
       quat_right = pose_ref.orientation
       quat_left = pose_ref_left.orientation
       quat_back = pose_ref_back.orientation
       quat_elbow_left = pose_ref_elbow_left.orientation
       right_wrist = geometry_msgs.msg.TransformStamped()
       left_wrist = geometry_msgs.msg.TransformStamped()
       right_elbow = geometry_msgs.msg.TransformStamped()
       left_elbow = geometry_msgs.msg.TransformStamped()
       neck_tf = geometry_msgs.msg.TransformStamped()

       try:
         neck_tf = tfBuffer.lookup_transform('base_link_body','neck',  rospy.Time(0),rospy.Duration(0.2))
       except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
         rospy.logerr("No transform 0.") 
         return
       try:
         left_wrist = tfBuffer.lookup_transform('neck','wrist_left',  rospy.Time(0),rospy.Duration(0.2))
       except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
         rospy.logerr("No transform 1.")
         return
       try:
         left_elbow = tfBuffer.lookup_transform('neck','elbow_left', rospy.Time(0),rospy.Duration(0.2))
       except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
         rospy.logerr("No transform 2")
         return
       try:
         right_wrist = tfBuffer.lookup_transform('neck','wrist_right', rospy.Time(0),rospy.Duration(0.2))
       except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
         rospy.logerr("No transform 3")
         return
       try:
         right_elbow = tfBuffer.lookup_transform('neck','wrist_right', rospy.Time(0),rospy.Duration(0.2))

       except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
         rospy.logerr("No transform 4")
         return
       print right_wrist
       print status
       (r_right,p_right,y_right)= ang(quat_right.x,quat_right.y,quat_right.z,quat_right.w)
       (r_left,p_left,y_left)= ang(quat_left.x,quat_left.y,quat_left.z,quat_left.w)
       (r_back,p_back,y_back)= ang(quat_back.x,quat_back.y,quat_back.z,quat_back.w)
       (r_elbow_left,p_elbow_left,y_elbow_left)= ang(quat_elbow_left.x,quat_elbow_left.y,quat_elbow_left.z,quat_elbow_left.w)
       print r_right*180/pi, p_right*180/pi,y_right*180/pi, 'desna', pose_ref.position.z
       print r_left*180/pi,p_left*180/pi,y_left*180/pi, pose_ref_left.position.z
       #print r_back*180/pi, p_back*180/pi,y_back*180/pi
       scale=1
       if (status == "driving"):
	  up(1)
          right(1)
          forward(1)
          pub_pose.publish(current_pose)
       if (status == "slow"):
	  up(3)
          right(3)
          forward(3)
          pub_pose.publish(current_pose)
       move_robot_arm()
    
def control():
       global status
       status = "frozen" 
       rospy.init_node('controller', anonymous=True)
       rate = rospy.Rate(1) #hz
       global start_time, tfBuffer, listener,yaw_command
       yaw_command=0;
       tfBuffer = tf2_ros.Buffer()
       listener = tf2_ros.TransformListener(tfBuffer)

       start_time = rospy.get_rostime()
       while not rospy.is_shutdown():
          move_uav()
          rospy.sleep(0.1)
                   
if __name__ == '__main__':
     try:
         control()
     except rospy.ROSInterruptException:
         pass
