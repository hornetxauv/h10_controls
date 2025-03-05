import rclpy
import time
from rclpy.node import Node
from msg_types.srv import MovementService  # Ensure correct package reference
from msg_types.msg import Movement  # Import Movement message
from threading import Thread
from rclpy.duration import Duration



class MovementServiceNode(Node):
    def __init__(self):
        super().__init__('movement_service_node')
        
        # Create a service
        self.srv = self.create_service(MovementService, 'foxglove_movement_service', self.handle_movement_request)
        
        # Create a publisher for wanted_movement topic
        self.goal_publisher = self.create_publisher(Movement, "/controls/wanted_goal_movement", 10)
        
        self.get_logger().info("Movement Service Node Ready")

    def handle_movement_request(self, request, response):
        self.get_logger().info(f"Received movement request. Publishing for {request.duration} seconds...")

        # Start a separate thread to publish continuously
        thread = Thread(target=self.publish_continuously, args=(request.movement, request.duration))
        thread.start()

        response.success = True
        return response

    def publish_continuously(self, movement_msg, duration):
        start_time = self.get_clock().now()
        rate = self.create_rate(2)  # 10 Hz publishing rate (adjustable)

        while (self.get_clock().now() - start_time) < Duration(seconds=duration):
            self.goal_publisher.publish(movement_msg)
            self.get_logger().info(f"Published: {movement_msg}")
            rate.sleep()  # Maintain 10 Hz publishing rate

        myFinalMessage = Movement(x=0.0, y=0.0, z=0.0, roll=0.0, pitch=0.0, yaw=0.0)
        self.goal_publisher.publish(myFinalMessage)
        self.get_logger().info(f"Published: {myFinalMessage}")

        self.get_logger().info("Finished publishing movement.")


def main(args=None):
    rclpy.init(args=args)
    node = MovementServiceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down node.")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
