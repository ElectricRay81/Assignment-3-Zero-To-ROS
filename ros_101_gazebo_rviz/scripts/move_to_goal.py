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

    # Create twist object
    twist = Twist()

    # Compute difference between current pos and goal
    dX = robot_x_pos - goal.pose.position.x 
    dY = robot_y_pos - goal.pose.position.y 
    distance_to_goal = math.sqrt(dX**2 + dY**2)

    if distance_to_goal < 0.1:             
        twist.linear.x = 0
        pub.publish(twist)
        print(f"Goal reached")

        shortest_angle = math.atan2(math.sin(goal.pose.orientation.z - robot_theta), math.cos(goal.pose.orientation.z - robot_theta))
        print(f"goal X: {goal.pose.position.x:.2f}")
        print(f"goal Y: {goal.pose.position.y:.2f}")
        print(f"goal orientation Z: {goal.pose.orientation.z:.2f}")
        print(f"Shortest Angle: {shortest_angle:.2f}")

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
            twist.angular.z = 0.5 if angle > robot_theta else -0.5
            # We never assign the linear x, so it won't move forward. Only rotate
            pub.publish(twist)
            print(f"Rotate to goal")
            
        else:
            # We need to move forward
            twist.linear.x = 0.8 * distance_to_goal
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


