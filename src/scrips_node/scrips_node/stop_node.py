import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class stop_node(Node): 
    def __init__(self):
        super().__init__("stop_node") 
        self.ranges = []

        # subscriptor obj
        # obj (msg_type,topic_name, callback_handler, buffer) 
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.scaner, 1)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_parar', 10)

        timer_period = 0.5 # in [s]
        self.timer = self.create_timer(timer_period, self.parar)

    def scaner(self, data):
        self.ranges = data.ranges
        #self.get_logger().info('El minimo rango es:'+ str(min(self.ranges))+'\n\n\n')
    
    def parar(self):
        vel = Twist()
        if(min(self.ranges) < 0.4):
            vel.linear.x = -0.3
            vel.linear.y = 0.0
            vel.angular.z = 0.0
            self.cmd_pub.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    node = stop_node() # Definicion del objeto "node"

    # ejecucion ciclica 
    rclpy.spin(node)
    # finalizacion
    rclpy.shutdown()

if __name__ == "__main__":
    main()