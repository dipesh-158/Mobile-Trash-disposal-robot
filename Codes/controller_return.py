#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion

class MyRobotPIDController:
    def __init__(self):
        self.node_name = 'MyRobotPIDController'
        rospy.init_node(self.node_name, anonymous=True)

        # initial Parameters
        self.goal = np.array([10, 10])  # Goal position
        self.zeta = 0.5  # Scaling factor for attractive force
        self.eta = 10.0  # Scaling factor for repulsive force
        self.d = 3.0  # Threshold distance for changing attractive potential
        self.rho_0 = 3.0  # Influence range of repulsive potential
        self.goal_threshold = 0.5  # How close the robot needs to be to be considered at the goal

        # State Variables
        self.position = np.zeros(2)  # Robot's current position from odometry
        self.yaw = 0  # Robot's current orientation from IMU
        self.nearest_obstacle_distance = float('inf')  # Initialize with a high number
        self.starting_position = None  # To store the starting position
        self.reached_goal = False  # Flag to indicate goal has been reached

        # Communication
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/imu', Imu, self.imu_callback)
        rospy.Subscriber('/laser/scan', LaserScan, self.laser_callback)

    def rho(self, q1, q2):
        return np.linalg.norm(q1 - q2)

    def gradient_att(self, q, q_final):
        dist = self.rho(q, q_final)
        if dist <= self.d:
            return -self.zeta * (q - q_final)
        else:
            return -self.d * self.zeta * (q - q_final) / dist

    def gradient_rep(self, q, nearest_obstacle_position):
        dist = self.rho(q, nearest_obstacle_position)
        if dist <= self.rho_0 and dist != 0:  # Added check to avoid division by zero
            return self.eta * (1.0 / dist - 1.0 / self.rho_0) * (1.0 / dist**2) * ((q - nearest_obstacle_position) / np.linalg.norm(q - nearest_obstacle_position))
        else:
            return np.zeros_like(q)

    def odom_callback(self, msg):
        if self.starting_position is None:
            self.starting_position = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y])
        
        position = msg.pose.pose.position
        self.position = np.array([position.x, position.y])

    def imu_callback(self, msg):
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.yaw = euler_from_quaternion(orientation_list)[2]

    def laser_callback(self, msg):
        ranges = np.array(msg.ranges)
        self.nearest_obstacle_distance = np.min(ranges)
        min_index = np.argmin(ranges)
        nearest_obstacle_angle = min_index * msg.angle_increment + msg.angle_min

        # Computes the position of the nearest obstacle in the robot's coordinate frame
        nearest_obstacle_position = self.position + (
            self.nearest_obstacle_distance * np.array([
                np.cos(nearest_obstacle_angle + self.yaw),
                np.sin(nearest_obstacle_angle + self.yaw)
            ])
        )

        # Calculates attractive and repulsive forces
        F_att = self.gradient_att(self.position, self.goal)
        F_rep = self.gradient_rep(self.position, nearest_obstacle_position)

        # Combine the forces and navigate
        self.navigate(F_att + F_rep)

    def navigate(self, force):
        # Computes the desired angle from the force vector
        desired_angle = np.arctan2(force[1], force[0])
        angular_velocity = 0.5 * self.shortest_angular_distance(self.yaw, desired_angle)
        linear_velocity = min(np.linalg.norm(force), 1)  # Limit max speed

        # If close to the goal and haven't set the goal to start position
        if not self.reached_goal and self.rho(self.position, self.goal) < self.goal_threshold:
            self.goal = self.starting_position
            self.reached_goal = True

        # Publish the velocity command
        self.publish_cmd(linear_velocity, angular_velocity)

    def publish_cmd(self, linear, angular):
        twist = Twist()
        twist.linear.x = linear
        twist.angular.z = angular
        self.vel_pub.publish(twist)

    def shortest_angular_distance(self, from_angle, to_angle):
        # Function to compute the shortest distance between two angles
        return (to_angle - from_angle + np.pi) % (2 * np.pi) - np.pi

    def run(self):
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        controller = MyRobotPIDController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
