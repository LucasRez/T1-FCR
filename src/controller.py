#!/usr/bin/env python

import math
import rospy
import tf
import grafo
import dijkstra

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

class Pioneer:
    def __init__(self):
        rospy.init_node('pioneer_controller', anonymous=True)
        rospy.loginfo("Initializing...")
        self.MOV_SPEED = 0.5
        self.TURN_SPEED = 0.5
        self.MOVING = 0
        self.AVOIDING = 1
        self.MAX_RANGE = 30.0
        self.AVOID_DISTANCE = 0.7
        self.DETECT_DISTANCE = 0.9
        self.LASER_MID = 67
        self.LASER_FRONT_OFFSET_MOVING = 20
        self.LASER_FRONT_OFFSET_AVOIDING = 25
        self.XY_GOAL_TOLERANCE = 0.5
        self.YAW_GOAL_TOLERANCE = 0.05

        self.map = grafo.load_map()

        self.obstacle_left = False
        self.obstacle_right = False
        self.obstacle_front = False
        self.current_status = self.MOVING
        self.laser_offset = self.LASER_FRONT_OFFSET_MOVING
        self.avoiding_left = False
        self.avoiding_right = False
        self.en_route = False

        self.mov_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.laser_subscriber = rospy.Subscriber('hokuyo_scan', LaserScan, self.laser_callback)
        self.goal_subscriber = rospy.Subscriber('goal', Odometry, self.goal_callback)
        self.pose_subscriber = rospy.Subscriber('pose', Odometry, self.pose_callback)
        self.rate = rospy.Rate(10)

        self.obstacles_front = []
        self.obstacles_left = []
        self.obstacles_right = []
        self.laser_ranges = []
        self.goals = []
        self.current_route = []
        self.position_x = 0.0
        self.position_y = 0.0
        self.yaw = 0.0
        self.goal_distance = 0
        self.goal_angle = 0
        
        rospy.loginfo("Ready to recieve")
        
    def move_forward(self):
        vel_msg = Twist()
        vel_msg.linear.x = self.MOV_SPEED
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.mov_publisher.publish(vel_msg)

    def stop(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = 0
        self.mov_publisher.publish(vel_msg)

    def spin_left(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = self.TURN_SPEED
        self.mov_publisher.publish(vel_msg)

    def spin_right(self):
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0

        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = -self.TURN_SPEED
        self.mov_publisher.publish(vel_msg)

    def spin_to_yaw(self, yaw):
        direction = 0
        if (math.degrees(yaw - self.yaw) + 360) % 360 <= 180:
            direction = 0 # LEFT = 0
        else:
            direction = 1 # RIGHT = 1
        while math.sqrt(math.pow(self.yaw - yaw, 2)) >= self.YAW_GOAL_TOLERANCE:
            if direction == 0:
                self.spin_left()
            else:
                self.spin_right()
        
        self.stop()

    def goal_callback(self, data):
        goal_x = round(data.pose.pose.position.x, 4)
        goal_y = round(data.pose.pose.position.y, 4)
        dest = grafo.qual_nodo(self.map, goal_x, goal_y)
        if dest == None:
            rospy.loginfo("Destination unreachable, discarding goal")
            return
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        goal_yaw = round(euler[2], 4)
        self.goals.append((goal_x, goal_y, goal_yaw))
        rospy.loginfo("New goal recieved:" + self.goals.__str__())

    def pose_callback(self, data):
        self.position_x = round(data.pose.pose.position.x, 4)
        self.position_y = round(data.pose.pose.position.y, 4)
        quaternion = (
            data.pose.pose.orientation.x,
            data.pose.pose.orientation.y,
            data.pose.pose.orientation.z,
            data.pose.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quaternion)
        self.yaw = round(euler[2], 4)

    def laser_callback(self, data):
        self.laser_ranges = list(data.ranges)[::8]
        self.obstacles_front = [(
            round(x, 4), self.laser_ranges.index(x)*2 - self.LASER_MID)
            for x in 
            self.laser_ranges[self.LASER_MID - self.laser_offset:self.LASER_MID + self.laser_offset]
            if x < self.AVOID_DISTANCE]
        self.obstacles_right = [(
            round(x, 4), self.laser_ranges.index(x)*2 - self.LASER_MID)
            for x in 
            self.laser_ranges[0:self.LASER_MID - self.laser_offset]
            if x < self.DETECT_DISTANCE]
        self.obstacles_left = [(
            round(x, 4), self.laser_ranges.index(x)*2 - self.LASER_MID)
            for x in 
            self.laser_ranges[self.LASER_MID + self.laser_offset:]
            if x < self.DETECT_DISTANCE]

    def calculate_obstacle_position(self):
        (min_dist, angle) = min(self.obstacles_front, key = lambda t: t[0])
        angle = self.yaw + math.radians(angle)
        obst_pos_x = round(self.position_x + (min_dist * math.cos(angle)), 4)
        obst_pos_y = round(self.position_y + (min_dist * math.sin(angle)), 4)
    
    def distance_to_goal(self):
        delta_x = self.current_route[0][0] - self.position_x
        delta_y = self.current_route[0][1] - self.position_y
        self.goal_distance = math.sqrt(math.pow(delta_x, 2) + math.pow(delta_y, 2))
        self.goal_angle = math.atan2(delta_y, delta_x)

    def move_to_goal(self):
        self.distance_to_goal()
        while self.goal_distance > self.XY_GOAL_TOLERANCE:
            self.find_obstacles()
            if self.obstacle_front:
                self.stop()
                self.current_status = self.AVOIDING
                self.laser_offset = self.LASER_FRONT_OFFSET_AVOIDING
                return
            if math.sqrt(math.pow(self.goal_angle - self.yaw, 2)) >= self.YAW_GOAL_TOLERANCE:
                self.spin_to_yaw(self.goal_angle)
            self.move_forward()
            self.distance_to_goal()
        self.stop()
        if len(self.current_route) == 1:
            self.spin_to_yaw(self.current_route[0][2])
        rospy.loginfo("Arrived at: " + self.current_route[0].__str__())
        self.current_route.pop(0)
        if len(self.current_route) == 0:
            self.en_route = False
            rospy.loginfo("Arrived at goal: " + self.goals[0].__str__())
            self.goals.pop(0)

    def avoid_obstacle(self):
        if self.obstacle_front:
            if len(self.obstacles_left) >= len(self.obstacles_right):
                self.avoiding_left = True
                self.avoiding_right = False
                self.spin_right()
            else:
                self.avoiding_left = False
                self.avoiding_right = True
                self.spin_left()
        else:
            while self.obstacle_left and self.avoiding_left and not self.obstacle_front:
                self.move_forward()
                self.find_obstacles()
            while self.obstacle_right and self.avoiding_right and not self.obstacle_front:
                self.move_forward()
                self.find_obstacles()
            self.avoiding_left = False
            self.avoiding_right = False
            self.current_status = self.MOVING
            self.laser_offset = self.LASER_FRONT_OFFSET_MOVING
                

    def find_obstacles(self):
        if len(self.obstacles_front):
            self.obstacle_front = True
        else:
            self.obstacle_front = False

        if len(self.obstacles_right):
            self.obstacle_right = True
        else:
            self.obstacle_right = False

        if len(self.obstacles_left):
            self.obstacle_left = True
        else:
            self.obstacle_left = False

    def calculate_route(self):
        origin = grafo.qual_nodo(self.map, self.position_x, self.position_y)
        dest = grafo.qual_nodo(self.map, self.goals[0][0], self.goals[0][1])
        if dest == None:
            rospy.loginfo("Destination unreachable, discarding goal")
            self.goals.pop(0)
            return
        path = dijkstra.dijkstra(self.map, origin.id, dest.id)
        for p in path:
            self.current_route.append((p.x, p.y, self.yaw))
        self.current_route.append(self.goals[0])
        self.en_route = True
        rospy.loginfo("Calculated path: " + self.current_route.__str__())
        

    def mover(self):
        while not rospy.is_shutdown():
            self.find_obstacles()
            if len(self.goals):
                if self.current_status == self.MOVING:
                    if not self.en_route:
                        self.calculate_route()
                    else: 
                        rospy.loginfo("moving to: " + self.current_route[0].__str__())
                        self.move_to_goal()
                elif self.current_status == self.AVOIDING:
                    rospy.loginfo("avoiding obstacle")
                    self.avoid_obstacle()
                else:
                    pass
            else:
                self.stop()

            self.rate.sleep()

if __name__ == '__main__':
    try:
        p = Pioneer()
        p.mover()
    except rospy.ROSInterruptException:
        pass