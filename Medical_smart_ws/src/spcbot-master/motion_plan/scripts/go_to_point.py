#! /usr/bin/env python3

# import ros stuff
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from tf import transformations
from std_srvs.srv import *

import math

active_ = False

# robot state variables
position_ = Point()
yaw_ = 0
# machine state
state_ = 0
# goal
desired_position_ = Point()
desired_position_.x = rospy.get_param('des_pos_x')
desired_position_.y = rospy.get_param('des_pos_y')
desired_position_.z = 0
# parameters
yaw_precision_ = math.pi / 90 # +/- 2 degree allowed                               # Gia tri chinh xac 
dist_precision_ = 0.3

# publishers
pub = None

# service callbacks
def go_to_point_switch(req):                                                        # Chua hieu ve service cua ROS
    global active_
    active_ = req.data
    res = SetBoolResponse()
    res.success = True
    res.message = 'Done!'
    return res

# callbacks
def clbk_odom(msg):                                                                 # Odometry dung de xac dinh huong, vi tri va cung cap van toc ..v..v 
    global position_	
    global yaw_
    
    # position
    position_ = msg.pose.pose.position
    
    # yaw
    quaternion = (                                                                   # Quaternion ?????  gia tri chuyen doi cac thanh phan 2D 3D cua value yaw            Chua hieu ve Odometry
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
    euler = transformations.euler_from_quaternion(quaternion)
    yaw_ = euler[2]

def change_state(state):
    global state_                                                                     # Bien toan cau State_  va bien state de cap nhat thay doi
    state_ = state
    print ('State changed to [%s]' % state_)

def normalize_angle(angle):                                                          # Ham math.fabs ???  tri tuyet doi               
    if(math.fabs(angle) > math.pi):							# Lay goc be hon 180
        angle = angle - (2 * math.pi * angle) / (math.fabs(angle))
    return angle

def fix_yaw(des_pos):                                                                  # Neu huong cua robot toi target rat lon thi tru di 0.7 radian roi chuyen sang trang thai 1 : Go ahead
    global yaw_, pub, yaw_precision_, state_
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = normalize_angle(desired_yaw - yaw_)
    #print("fix")
    rospy.loginfo(err_yaw)
    #rospy.loginfo(yaw_)
    
    twist_msg = Twist()
    if math.fabs(err_yaw) > yaw_precision_:
        twist_msg.angular.z = 0.4 if err_yaw > 0 else -0.4
    
    pub.publish(twist_msg)
    
    # state change conditions
    if math.fabs(err_yaw) <= yaw_precision_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(1)

def go_straight_ahead(des_pos):							              # Tinh khoang cach can toi va cho robot di thang va cong tru 0.2 truc yaw 
												      # Tiep tuc di thang toi khi den target neu goc yaw bi lechj chuyen sang trang thai 0 (fix_yaw)
												      # Neu khaong cach < yaw_predict  chuyen trang thai 2 (Ket thuc : Done)
    global yaw_, pub, yaw_precision_, state_                                            
    desired_yaw = math.atan2(des_pos.y - position_.y, des_pos.x - position_.x)
    err_yaw = desired_yaw - yaw_
    err_pos = math.sqrt(pow(des_pos.y - position_.y, 2) + pow(des_pos.x - position_.x, 2))
    #print ('Position error test: [%s]' % err_pos)
    #print ('yaw error initial: [%s]' % err_yaw)
    #print ('Yaw ', yaw_)
    if err_pos > dist_precision_:
        twist_msg = Twist()
        twist_msg.linear.x = 0.3
        #twist_msg.angular.z = 0.02 if err_yaw > 0 else -0.02                            # thay doi 0.2 > 0.1
        #print ('Sai so goc yaw %s',err_yaw)
        pub.publish(twist_msg)
    else:
        print ('Position error: [%s]' % err_pos)
        change_state(2)
    
    #print(state_)
    # state change conditions
    if math.fabs(err_yaw) > yaw_precision_:
        print ('Yaw error: [%s]' % err_yaw)
        change_state(0)

def done():											 # Robot dung yen
    twist_msg = Twist()           
    twist_msg.linear.x = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)

def main():
    global pub, active_
    
    rospy.init_node('go_to_point')
    
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)
    
    srv = rospy.Service('go_to_point_switch', SetBool, go_to_point_switch)                  # Chua hieu ve service lam
   
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():								# Chuyen trang thai State va kiem tra active
        if not active_:
            
            continue
        else:
            if state_ == 0:
                fix_yaw(desired_position_)
            elif state_ == 1:
                go_straight_ahead(desired_position_)
            elif state_ == 2:
                done()
            else:
                rospy.logerr('Unknown state!')
        
        rate.sleep()

if __name__ == '__main__':
    main()
