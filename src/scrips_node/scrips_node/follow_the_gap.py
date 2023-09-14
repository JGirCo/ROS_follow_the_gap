import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Pose
import math

import sys

class follow_the_gap(Node): 
    def __init__(self):
        super().__init__("follow_the_gap") 

        self.min_angle = 0
        self.max_angle = 240
        self.middle_angle = self.max_angle/2
        self.min_dist = 2.5
        self.max_gap=[]
        self.targetAngle=self.middle_angle
        # subscriptor obj
        # obj (msg_type,topic_name, callback_handler, buffer) 
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.scanner, 1)
        
        self.error_pub = self.create_publisher(Pose, '/error', 10)

        timer_period = 0.5 # in [s]
        self.timer = self.create_timer(timer_period, self.error)
    
    def scanner(self, data):
        ranges = data.ranges
        currentGap = []
        self.max_gap = []
        for i, distance in enumerate(ranges):
            if distance < self.min_dist:
                currentGap = []
                continue
            currentGap.append(i)
            if len(currentGap) <= len(self.max_gap):
                continue
            self.max_gap = currentGap
        angles = range(self.min_angle,self.max_angle)
        gapRanges = [ranges[i] for i in self.max_gap]
        gapAngles = [angles[i] for i in self.max_gap]
        print(gapRanges)
        self.get_logger().info('min range:'+ str(min(gapRanges))+'\n\n\n')
        sortedGapRanges = sorted(gapRanges)
        medianGapValue = sortedGapRanges[len(sortedGapRanges)//2]
        self.targetAngle = gapAngles[gapRanges.index(medianGapValue)]

    def error(self):
        # Escrito así para poder hacer uso del PD de Carlos.
        # Se escribe en la posición x pero representa una diferencia de ángulos
        # Aún así, la salida del PD es una velocidad angular en z
        error = Pose()
        error.position.x = (self.middle_angle - self.targetAngle)
        #self.get_logger().info('Envio el error:'+ str(error.position.x)+'\n\n\n')

        self.error_pub.publish(error)

def main(args=None):
    rclpy.init(args=args)
    node = follow_the_gap() # Definicion del objeto "node"

    # ejecucion ciclica 
    rclpy.spin(node)
    # finalizacion
    rclpy.shutdown()

if __name__ == "__main__":
    main()
