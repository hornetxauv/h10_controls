import rclpy
from rclpy.node import Node
from msg_types.msg import IMU
import cv2
import numpy as np
from collections import deque

class IMUPlotter(Node):
    def __init__(self):
        super().__init__('imu_plotter')
        
        self.dashboard_sections = {
            'IMU Data': {
                'topic_type': IMU,
                'topic_name': '/sensors/imu',
                'display_values': ['roll,pitch,yaw'],
                'color': (255, 255, 0),
                'x_pos': 20,
                'data': {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
            },
            'Depth Data': {
                'topic_type': None,
                'topic_name': '/depth',
                'display_values': ['current_depth,desired_depth'],
                'color': (0, 255, 255),
                'x_pos': 300,
                'data': {'current_depth': 0.0, 'desired_depth': 0.0}
            },
            'Thrust PWMs': {
                'topic_type': None,
                'topic_name': '/pwm',
                'display_values': ['thrust_pwms'],
                'color': (0, 255, 0),
                'x_pos': 580,
                'data': {'thrust_pwms': [0] * 7}
            }
        }
        
        self.init_subscriptions()
        
        self.plot_width = 600
        self.plot_points = 100
        self.roll_points = np.zeros(self.plot_points)
        self.pitch_points = np.zeros(self.plot_points)
        self.yaw_points = np.zeros(self.plot_points)
        self.x_points = np.linspace(0, self.plot_width, self.plot_points)
        self.time_values = np.linspace(-10, 0, self.plot_points)

        self.plot_window = 'Roll, Pitch and Yaw Plot'
        self.dashboard_window = 'Dashboard'
        cv2.namedWindow(self.plot_window, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.dashboard_window, cv2.WINDOW_NORMAL)
        cv2.moveWindow(self.plot_window, 0, 0)
        cv2.moveWindow(self.dashboard_window, 0, 500)

    def init_subscriptions(self):
        for section_name, config in self.dashboard_sections.items():
            if config['topic_type'] is not None:
                self.create_subscription(
                    config['topic_type'],
                    config['topic_name'],
                    lambda msg, s=section_name: self.topic_callback(msg, s),
                    10
                )

    def add_dashboard_section(self, name, topic_type, topic_name, display_values, color, x_pos):
        self.dashboard_sections[name] = {
            'topic_type': topic_type,
            'topic_name': topic_name,
            'display_values': display_values,
            'color': color,
            'x_pos': x_pos,
            'data': {value: 0.0 for value in display_values[0].split(',')}
        }
        
        if topic_type is not None:
            self.create_subscription(
                topic_type,
                topic_name,
                lambda msg, s=name: self.topic_callback(msg, s),
                10
            )

    def topic_callback(self, msg, section_name):
        section = self.dashboard_sections[section_name]
        
        for value in section['display_values'][0].split(','):
            if hasattr(msg, value):
                section['data'][value] = getattr(msg, value)
        
        if section_name == 'IMU Data':
            self.update_imu_plot(msg)
        
        self.update_dashboard()
        
    def update_imu_plot(self, msg):
        self.roll_points = np.roll(self.roll_points, -1)
        self.pitch_points = np.roll(self.pitch_points, -1)
        self.yaw_points = np.roll(self.yaw_points, -1)
        
        self.roll_points[-1] = msg.roll
        self.pitch_points[-1] = msg.pitch
        self.yaw_points[-1] = msg.yaw
        
        self.plot_roll_yaw_pitch()

    def plot_roll_yaw_pitch(self):
        plot_height = 400
        plot_image = np.zeros((plot_height, self.plot_width, 3), dtype=np.uint8)
        max_angle = 90

        grid_color = (50, 50, 50)
        for i in range(0, plot_height, 50):
            cv2.line(plot_image, (0, i), (self.plot_width, i), grid_color, 1)
        for i in range(0, self.plot_width, 50):
            cv2.line(plot_image, (i, 0), (i, plot_height), grid_color, 1)

        roll_scaled = np.int32((self.roll_points / max_angle) * (plot_height // 2) + plot_height // 2)
        pitch_scaled = np.int32((self.pitch_points / max_angle) * (plot_height // 2) + plot_height // 2)
        yaw_scaled = np.int32((self.yaw_points / max_angle) * (plot_height // 2) + plot_height // 2)

        for i in range(1, len(self.x_points)):
            cv2.line(plot_image, 
                    (int(self.x_points[i-1]), roll_scaled[i-1]),
                    (int(self.x_points[i]), roll_scaled[i]),
                    (255, 0, 0), 2)
            
            cv2.line(plot_image,
                    (int(self.x_points[i-1]), pitch_scaled[i-1]),
                    (int(self.x_points[i]), pitch_scaled[i]),
                    (0, 255, 0), 2)
            
            cv2.line(plot_image,
                    (int(self.x_points[i-1]), yaw_scaled[i-1]),
                    (int(self.x_points[i]), yaw_scaled[i]),
                    (0, 0, 255), 2)

        for y in range(0, plot_height, 50):
            angle = max_angle - (y * 2 * max_angle) // plot_height
            cv2.putText(plot_image, f'{angle}°',
                       (5, y + 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        num_time_labels = 10
        for i in range(num_time_labels):
            x = int(i * self.plot_width / (num_time_labels - 1))
            time_index = int(i * (self.plot_points - 1) / (num_time_labels - 1))
            time_label = f'{self.time_values[time_index]:.1f}s'
            cv2.putText(plot_image, time_label,
                       (x, plot_height - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        cv2.putText(plot_image, f'Roll: {self.roll_points[-1]:.2f}°',
                   (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 2)
        cv2.putText(plot_image, f'Pitch: {self.pitch_points[-1]:.2f}°',
                   (170, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        cv2.putText(plot_image, f'Yaw: {self.yaw_points[-1]:.2f}°',
                   (330, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)

        cv2.imshow(self.plot_window, plot_image)
        cv2.waitKey(1)

    def update_dashboard(self):
        dashboard_height, dashboard_width = 400, 800
        dashboard_image = np.zeros((dashboard_height, dashboard_width, 3), dtype=np.uint8)
        
        def add_text_block(text, position, color=(255, 255, 255)):
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 0.7
            thickness = 1
            padding = 5
            
            (text_width, text_height), _ = cv2.getTextSize(text, font, font_scale, thickness)
            
            start_y = position[1] - text_height - padding
            cv2.rectangle(dashboard_image, 
                         (position[0] - padding, start_y),
                         (position[0] + text_width + padding, position[1] + padding),
                         (50, 50, 50), -1)
            
            cv2.putText(dashboard_image, text, position, font, font_scale, color, thickness)
            return text_height + 2 * padding

        y_pos = 30
        cv2.putText(dashboard_image, "Dashboard", (dashboard_width//2 - 100, y_pos), 
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (255, 255, 255), 2)

        for section_name, config in self.dashboard_sections.items():
            y_pos = 80
            x_pos = config['x_pos']
            
            y_pos += add_text_block(f"{section_name}:", (x_pos, y_pos), config['color'])
            
            for value_name in config['display_values'][0].split(','):
                data = config['data'][value_name]
                if isinstance(data, list):
                    text = f"[{', '.join(str(x) for x in data)}]"
                else:
                    text = f"{value_name}: {data:.2f}"
                y_pos += add_text_block(text, (x_pos, y_pos))

        cv2.imshow(self.dashboard_window, dashboard_image)
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
