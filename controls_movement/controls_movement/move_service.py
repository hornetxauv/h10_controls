import rclpy
import time
from rclpy.node import Node
from msg_types.srv import MovementService  # Ensure correct package reference
from msg_types.msg import Movement  # Import Movement message

class MovementServiceNode(Node):
    def __init__(self):
        super().__init__('movement_service_node')
        
        # Create a service
        self.srv = self.create_service(MovementService, 'foxglove_movement_service', self.handle_movement_request)
        
        # Create a publisher for wanted_movement topic
        self.publisher = self.create_publisher(Movement, 'wanted_movement', 10)
        
        self.get_logger().info("Movement Service Node Ready")

    def handle_movement_request(self, request, response):
        self.get_logger().info(f"Received movement request. Publishing after {request.duration} seconds...")
        
        # Wait for the specified duration
        time.sleep(request.duration)
        
        # Publish the movement message
        self.publisher.publish(request.movement)
        
        self.get_logger().info(f"Published to wanted_movement: {request.movement}")
        response.success = True
        return response

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
