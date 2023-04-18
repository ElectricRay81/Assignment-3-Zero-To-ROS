#!/usr/bin/env python
import rospy
import tf
import math
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion

# Initialise the node
rospy.init_node("Move_to_goal")
pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

# Global variables
goal = None

# Instantiate the tf listener
listener = tf.TransformListener()

def move_robot():
    if goal is None:
        return
    
    (trans, rot) = listener.lookupTransform('/odom', '/base_footprint', rospy.Time(0))
    robot_x_pos = trans[0]
    robot_y_pos = trans[1]
    robot_theta = euler_from_quaternion(rot)[2] # This pulls the yaw angle

    # Calculate the angular difference between the robot's current heading and the goal
    angle = math.atan2(goal.pose.position.y - robot_y_pos, goal.pose.position.x - robot_x_pos)
    print(f"Angle: {angle:.2f}")

    # Convert the goal to Euler values 
    orientation_quat = (goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w)
    _, _, yaw = euler_from_quaternion(orientation_quat)

    # compute the shortest angle to rotate to
    shortest_angle = math.atan2(math.sin(yaw- robot_theta), math.cos(yaw- robot_theta))

    # Create twist object
    twist = Twist()

    # Compute difference between current pos and goal
    dX = robot_x_pos - goal.pose.position.x 
    dY = robot_y_pos - goal.pose.position.y 
    distance_to_goal = math.sqrt(dX**2 + dY**2)

    if distance_to_goal < 0.1:              # If the target distance is within the limits stop driving and rotate to final orientation         
        if abs(shortest_angle) <= 0.035:
            twist.angular.z = 0
            pub.publish(twist)
            print(f"Final position reached")            
        else:
            twist.angular.z = 0.5 * shortest_angle
            pub.publish(twist)
            print(f"Rotate to final orientation")
    else:
        if abs(angle - robot_theta) > 0.2:
            # We need to rotate           
            twist.angular.z = 0.25 if angle > robot_theta else -0.25
            # We never assign the linear x, so it won't move forward. Only rotate
            pub.publish(twist)
            print(f"Rotate to goal")
            
        else:
            # We need to move forward
            twist.linear.x = 0.6 * distance_to_goal
            # We never assign the angular z, so it won't rotate. Only move forward
            pub.publish(twist)
            print(f"Drive to goal")
            print(distance_to_goal)

def goal_callback(msg):
    global goal
    goal = msg
    print(f"goal received")

# Set up our goal subscriber
rospy.Subscriber('move_base_simple/goal', PoseStamped, goal_callback)

rate = rospy.Rate(1.0)
while not rospy.is_shutdown():
    move_robot()
    rate.sleep()


