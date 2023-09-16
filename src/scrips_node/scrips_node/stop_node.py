import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan

class stop_node(Node): 
    def __init__(self):
        super().__init__("stop_node") 
        self.ranges = []
        self.stop = False

        # subscriptor obj
        # obj (msg_type,topic_name, callback_handler, buffer) 
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.scaner, 1)

        self.cmd_pub = self.create_publisher(Bool, '/stop', 10)

        timer_period = 0.1 # in [s]
        self.timer = self.create_timer(timer_period, self.parar)

    def scaner(self, data):
        distances = data.ranges
        self.ranges = distances[100:-100]
        if min(self.ranges) <= 0.4:
            stop = True
        else:
            stop = False
        self.stop = stop
        #self.get_logger().info('El minimo rango es:'+ str(min(self.ranges))+'\n\n\n')
    
    def parar(self):
        msg = Bool()
        msg.data = self.stop
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = stop_node() # Definicion del objeto "node"

    # ejecucion ciclica 
    rclpy.spin(node)
    # finalizacion
    rclpy.shutdown()

if __name__ == "__main__":
    main()
