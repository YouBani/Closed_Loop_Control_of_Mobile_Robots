#!/usr/bin/env python

from math import pi, sqrt, atan2, cos, sin
import numpy as np

import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D


class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        # reset odometry to zero
        self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)
        for i in range(10):
            self.reset_pub.publish(Empty())
            self.rate.sleep()

        # subscribe to odometry
        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')

    def run(self):
        self.vel = Twist()
        count = 0
        theta_error = 0
        distance_error = 0.15
        while not rospy.is_shutdown():
            theta_desired = count * (pi / 2)
            last_theta_error = theta_error
            if count < 2:
                theta_error = theta_desired - self.pose.theta
                theta_derivative = theta_error - last_theta_error
                Kp = 3
                Kd = 0.4
            if abs(theta_error) < 0.05:  # Keep moving forward
                self.vel.linear.x = 0.5
            else:
                self.vel.linear.x = 0  # Stop and corrects the error
            if count == 0 and abs(self.pose.x - 4) < distance_error:
                count += 1
                self.vel.linear.x = 0  # Stop before making a turn
                print("Side 1 is finished!")
            elif count == 1 and abs(self.pose.y - 4) < distance_error:
                count += 1
                self.vel.linear.x = 0
                print("Side 2 is finished!")
            elif count == 2:  # The orientation of the robot ranges from -pi to pi on 2D plane When the robot turns
                if self.pose.theta < 0:  # in CCW direction , the value of theta will jump from pi to −pi.
                    theta_error = -pi - self.pose.theta  # The desired theta is adjusted
                else:
                    theta_error = theta_desired - self.pose.theta
                theta_derivative = theta_error - last_theta_error
                if abs(self.pose.x - 0) < distance_error:
                    count += 1
                    self.vel.linear.x = 0
                    self.vel.angular.z = 5
                    self.vel_pub.publish(self.vel)
                    print("Side 3 is Finished!")
                    self.rate.sleep()
            elif count == 3:
                theta_desired = -pi / 2
                theta_error = theta_desired - self.pose.theta
                theta_derivative = theta_error - last_theta_error
                if abs(theta_error) < 0.05:
                    self.vel.linear.x = 0.5
                else:
                    self.vel.linear.x = 0
                if abs(self.pose.y - 0) < distance_error:
                    self.vel.linear.x = 0
                    self.vel_pub.publish(self.vel)
                    self.rate.sleep()
                    print('Side 4  is finished and the origin is reached!')
                    count += 1
            self.vel.angular.z = Kp * theta_error + Kd * theta_derivative
            self.vel_pub.publish(self.vel)
            if count == 4:
                break
            self.rate.sleep()

    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z,
                       msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
            rospy.loginfo("odom: x=" + str(self.pose.x) + ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))


if __name__ == '__main__':
    whatever = Turtlebot()
