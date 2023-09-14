import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
import math

class pidWF_node(Node): 
    def __init__(self):
        super().__init__("pidWF_node") 
        self.error_mio = 0
        self.kp = 0.01
        self.kd = 1.0

        # subscriptor obj
        # obj (msg_type,topic_name, callback_handler, buffer) 
        self.error_sub = self.create_subscription(Pose, '/error', self.error, 1)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        timer_period = 0.5 # in [s]
        self.timer = self.create_timer(timer_period, self.velocidad)

    def error(self, data):
        self.error_mio = data.position.x
    
    def velocidad(self): #el eje de giro z sale del piso
        vel = Twist()
        vel.linear.x = 0.3
        if(not math.isnan(self.error_mio)):
            vel.angular.z = -self.kp*self.error_mio
        #self.get_logger().info('Recibo el error:'+ str(self.error_mio)+'\n\n\n')
        self.get_logger().info('Vel angular:'+ str(vel.angular.z)+'\n\n\n')
        self.cmd_pub.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    node = pidWF_node() # Definicion del objeto "node"

    # ejecucion ciclica 
    rclpy.spin(node)
    # finalizacion
    rclpy.shutdown()

if __name__ == "__main__":
    main()
