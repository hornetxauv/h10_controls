import rclpy
from rclpy.node import Node
from msg_types.msg import IMU
import cv2
import numpy as np
from collections import deque

class IMUPlotter(Node):
    def __init__(self):
        super().__init__('imu_plotter')
        self.subscription = self.create_subscription(
            IMU,
            '/sensors/imu',
            self.imu_callback,
            10)
        
        self.roll_values = deque(maxlen=100)  # Limit history to 100 points
        self.pitch_values = deque(maxlen=100)
        self.time_values = np.linspace(-10, 0, 100)  # Display last 10 seconds (example)

        self.window_name = 'Roll and Pitch Plot'
        cv2.namedWindow(self.window_name)

    def imu_callback(self, msg):
        roll = msg.roll
        pitch = msg.pitch
        self.get_logger().info(f'Received roll: {roll:.2f}, pitch: {pitch:.2f}')
        
        self.roll_values.append(roll)
        self.pitch_values.append(pitch)
        
        self.plot_roll_pitch()

    def plot_roll_pitch(self):
        plot_height, plot_width = 400, 600
        plot_image = np.zeros((plot_height, plot_width, 3), dtype=np.uint8)

        max_angle = 90  # Assuming pitch and roll range is -90 to 90
        roll_scaled = [(r / max_angle) * (plot_height // 2) + plot_height // 2 for r in self.roll_values]
        pitch_scaled = [(p / max_angle) * (plot_height // 2) + plot_height // 2 for p in self.pitch_values]

        for i in range(1, len(roll_scaled)):
            cv2.line(plot_image, (i-1, int(roll_scaled[i-1])), (i, int(roll_scaled[i])), (255, 0, 0), 2)  # Roll in blue
            cv2.line(plot_image, (i-1, int(pitch_scaled[i-1])), (i, int(pitch_scaled[i])), (0, 255, 0), 2)  # Pitch in green

        # Draw time and angle axes
        for y in range(0, plot_height, 50):
            cv2.putText(plot_image, f'{max_angle - (y * 2 * max_angle) // plot_height}', 
                        (5, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        for x in range(0, plot_width, 60):
            time_label = f'{self.time_values[int(x * len(self.time_values) // plot_width)]:.1f}'
            cv2.putText(plot_image, time_label, (x, plot_height - 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        cv2.imshow(self.window_name, plot_image)
        cv2.waitKey(1)

    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    imu_plotter = IMUPlotter()

    try:
        rclpy.spin(imu_plotter)
    except KeyboardInterrupt:
        pass
    finally:
        imu_plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()