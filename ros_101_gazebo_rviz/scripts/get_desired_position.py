#!/usr/bin/env python
import rospy
import tf
import math
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

global robot_x
global robot_y
global robot_yaw

target_x_found = False
target_y_found = False
target_yaw_found = False
drive_yaw_arrived = False
arrived_at_coordinates = False
linear_vel = 0.0
angular_vel = 0.0
target_x = 0.0
target_y = 0.0
target_yaw = 0.0
robot_x = 0.0
robot_y = 0.0
robot_yaw = 0.0
drive_yaw = 0.0
dX = 0.0
dY = 0.0


# Initialize the node 
rospy.init_node('goal_subscriber')

# Instantiate listener object
listener = tf.TransformListener()

# Create publisher 
move_to_target_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

def callback(data):
    # Retrieve the position of the goal from the PoseStamped message
    global target_x
    global target_y
    
    global target_yaw
    global target_x_found
    global target_y_found
    global target_yaw_found

    target_x = data.pose.position.x
    target_y = data.pose.position.y
    orientation = data.pose.orientation
    # Convert the orentation values from quaternion to Euler values
    (_,_,target_yaw) = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    target_x_found = True
    target_y_found = True
    target_yaw_found = True

    #print(f"X: {target_x:.2f}\tY: {target_y:.2f}\tYaw: {target_yaw:.2f}")

def rotate_to_desired_orientation():
        print(f"rotate to final position!")
        
        twist = Twist()

        shortest_angle = math.atan2(math.sin(target_yaw - robot_yaw), math.cos(target_yaw - robot_yaw))
                # If target and is within 2 deg make angular velocity 0
        if abs(shortest_angle) <= 0.007:
            angular_vel = 0
        else:
            angular_vel = (1/0.57) * shortest_angle             #1/3.14 is proportional gain for angular velocity (the greater the differencethe faster it rotates)
        
        twist.linear.x = 0.0
        twist.angular.z = angular_vel
        move_to_target_pub.publish(twist)


def move_to_destination():
    global dX
    global dY
    global linear_vel
    global angular_vel
    global drive_yaw
    global arrived_at_coordinates
    global drive_yaw_arrived

    twist = Twist()

    if target_yaw_found and target_x_found and target_y_found:
        # Compute current angle to rotate
        shortest_angle = math.atan2(math.sin(drive_yaw - robot_yaw), math.cos(drive_yaw - robot_yaw))
        
        # Compute current distance to travel
        dX = target_x - robot_x
        dY = target_y - robot_y
        delta_distance = math.sqrt(dX**2 + dY**2)  
        drive_yaw = math.atan2(dY,dX) 

        # Set linear and angular velocity
        if delta_distance <= 0.1:
            linear_vel = 0
            arrived_at_coordinates = True
            print(f"coordinates arrived!")
        else:
            # Assign a linear velocity only when angle between current orientation and target is <= 15deg   
            arrived_at_coordinates = False
            if abs(shortest_angle) <= 0.262:
                linear_vel = 4.2* delta_distance                 #4.2 is proportional gain for linear velocity (the greater the difference the faster it moves)
            else:
                linear_vel = 0
                
        # If target and is within 2 deg make angular velocity 0
        if abs(shortest_angle) <= 0.2:
            angular_vel = 0
            drive_yaw_arrived = True
            print(f"drive yaw accomplished")
        else:
            angular_vel = (1/3.14) * shortest_angle             #1/3.14 is proportional gain for angular velocity (the greater the differencethe faster it rotates)
            drive_yaw_arrived = False

        if arrived_at_coordinates:
            rotate_to_desired_orientation()
        else:  

            twist.linear.x = linear_vel
            twist.angular.z = angular_vel
            move_to_target_pub.publish(twist)

# Subscribe to the move base simple/goal topic
rospy.Subscriber('move_base_simple/goal', PoseStamped, callback)

while not rospy.is_shutdown():

    try:
        (trans, rot) = listener.lookupTransform("/odom", "/base_footprint", rospy.Time(0))
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue
    robot_x = trans[0]
    robot_y = trans[1]
    (x, y, z, w) = rot
    (_,_,robot_yaw) = euler_from_quaternion([x, y, z, w])

    print(f"X: {robot_x:.2f}\tY: {robot_y:.2f}\t Yaw: {robot_yaw:.2f}")
    move_to_destination()
    

rospy.spin()