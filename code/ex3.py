#!/usr/bin/env python

# imports
from math import sin, cos, atan2, pi

import rospy
from geometry_msgs.msg import Pose2D
from math import sqrt, sin, cos, atan2
from geometry_msgs.msg import Pose2D, Twist
from sensor_msgs.msg import Range
from ex2 import Ex2
import numpy as np
from copy import copy 
import tf
from nav_msgs.msg import Odometry

#exercise class
class Ex3(Ex2):

    # init func
    def __init__(self,name):
        # super class ex2(inherit ex2 methods)
        super(Ex3,self).__init__(name)
        #odometry pose subscriber to retrieve robot pose from world
        self.pose_subscribe = rospy.Subscriber('/%s/odom' % self.name, Odometry, self.update_pose)
        # to allow ctrlc
        rospy.on_shutdown(self.stop_moving)
    #proximity sensor listener update func
    def update_proximity(self, data, sensor):
        self.proximity_distances[sensor] = data.range
    # odometry listener update func
    def update_pose(self,data):
        self.pose=self.quaternion_transform(data.pose.pose)
    # eucledian distance between 2 poses
    def euclidean_distance(self, new_pose, estimated_pose):
        return sqrt(pow((new_pose.x - estimated_pose.x), 2) +
                    pow((new_pose.y - estimated_pose.y), 2))
    # method to transform a 2d pose in a 3d pose using distance from origin
    def quaternion_transform(self, pose):
        quaternion = (pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)
        euler = tf.transformations.euler_from_quaternion(quaternion)
        euler_pose = Pose2D(pose.position.x, pose.position.y,euler[2])
        return euler_pose
    #move from perpendicular to opposite perpendicular(180 rotation) by using the back sensor
    def turn_opposite(self):
        # rotate 180 degrees 
        for _ in range(1,180):
            self.vel_msg.linear.x = 0.0
            # one degree
            self.vel_msg.angular.z = 0.355
            self.velocity_publisher.publish(self.vel_msg)
            self.rate.sleep()
            #if the back sensors see a wall stop turning arbitrarly and start to turn by checking the back sensors allignment
            if self.proximity_distances["rear_left"]<=0.11 or self.proximity_distances["rear_right"]<=0.11:
                while not rospy.is_shutdown():
                    target_diff = 0
                    # check back sensors difference to see when perpendicular to wall
                    diff = self.proximity_distances["rear_left"] - self.proximity_distances["rear_right"]
                    error = target_diff - diff
                    #if difference is still big enough to turn then check to which side to turn and prepare the message
                    self.vel_msg.linear.x = 0.0
                    if error>=0.006:
                        self.vel_msg.angular.z = -0.1
                    elif error<=-0.006:
                        self.vel_msg.angular.z = 0.1
                    else:
                        self.stop_moving()
                        return
                    #send prepared message
                    self.velocity_publisher.publish(self.vel_msg)
                    self.rate.sleep()
        # stop movement
        self.stop_moving()
    #logic to move 2 meters away from the wall
    def move2meters(self,distance):
        #get how mutch distance is still needed
        distanceleft=2.0-distance
        #get target pose of where to move
        target_pose = copy(self.pose)
        target_pose.x += distanceleft * cos(self.pose.theta)
        target_pose.y += distanceleft * sin(self.pose.theta)
        #go to pose until error distance is under a treshold
        while not rospy.is_shutdown():
            error = self.euclidean_distance(target_pose,self.pose)
            if error<0.4:
                self.stop_moving()
                break
                #send forward message
            self.vel_msg.linear.x = 0.3
            self.vel_msg.angular.z = 0.0
            self.velocity_publisher.publish(self.vel_msg)
            self.rate.sleep()
        #stop moving
        self.stop_moving()
if __name__ == '__main__':
    controller = Ex3("Thymio1")
    try:
        #call logic of exercise 2
        Ex2.go(controller)
        #stop movement before rotating 180
        controller.stop_moving()
        #rotate 180
        controller.turn_opposite()
        #calculate distance to wall using back sensors
        distanceToWall=controller.proximity_distances["rear_left"]
        #send 2 meters movement command
        controller.move2meters(distanceToWall)
    except rospy.ROSInterruptException as e:
        pass
