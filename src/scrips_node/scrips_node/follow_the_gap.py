import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
import numpy as np
import math

import sys

class follow_the_gap(Node): 
    def __init__(self):
        super().__init__("follow_the_gap") 

        self.min_angle = 0
        self.max_angle = 240
        self.middle_angle = self.max_angle/2
        self.min_dist = 2.0
        max_gap=[]
        self.targetAngle=self.middle_angle
        # subscriptor obj
        # obj (msg_type,topic_name, callback_handler, buffer) 
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.scanner, 1)
        
        self.error_pub = self.create_publisher(Pose, '/error', 10)

        timer_period = 0.05 # in [s]
        self.timer = self.create_timer(timer_period, self.error)
    
    def scanner(self, data):
        distances = data.ranges.tolist()
        angles = range(self.min_angle,self.max_angle)
        max_gap = self.getMaxGap(distances)
        gapDistances, gapAngles = self.getGapInfo(distances,angles, max_gap)
        try:
            self.targetAngle = self.getTargetAngle(gapDistances, gapAngles)
        except:
            self.get_logger().info('No gap'+'\n\n\n')

    def error(self):
        error = Pose()
        error.position.x = (self.middle_angle - self.targetAngle)
        self.error_pub.publish(error)


    def getMaxGap(self, distances):
        currentGap = []
        max_gap = []
        for i, distance in enumerate(distances):
            if distance < self.min_dist:
                currentGap = []
                continue
            currentGap.append(i)
            if len(currentGap) <= len(max_gap):
                continue
            max_gap = currentGap
        return max_gap

    def getGapInfo(self, distances, angles,max_gap):
        gapDistances = [distances[i] for i in max_gap]
        gapAngles = [angles[i] for i in max_gap]
        return gapDistances, gapAngles
        
    def getTargetAngle(self, gapDistances, gapAngles):
        targetAngle = gapAngles[len(gapDistances)//2]
        return targetAngle

def main(args=None):
    rclpy.init(args=args)
    node = follow_the_gap() # Definicion del objeto "node"

    # ejecucion ciclica 
    rclpy.spin(node)
    # finalizacion
    rclpy.shutdown()

if __name__ == "__main__":
    main()
