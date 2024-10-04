import rclpy
from rclpy.node import Node
import tf2_ros
from tf2_msgs.msg import TFMessage
import csv
import statistics

class TFListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener_node')

        # Create a TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscribe to the transform topic
        self.subscription = self.create_subscription(
            TFMessage,
            '/tf',  # Make sure this is the correct topic for TF messages
            self.tf_callback,
            10
        )

        # Initialize counters and lists for data tracking
        self.data = {
            'marker_0': {'times': [], 'last_time': None, 'csv_file': '../analysis/marker_0.csv', 'trans': [], 'rot': []},
            'marker_111': {'times': [], 'last_time': None, 'csv_file': '../analysis/marker_111.csv', 'trans': [], 'rot': []},
            'marker_222': {'times': [], 'last_time': None, 'csv_file': '../analysis/marker_222.csv', 'trans': [], 'rot': []}
        }
        self.counter = {'marker_0': 0, 'marker_111': 0, 'marker_222': 0, 'total': 0}
        self.moving_average_window = 10  # Set how many messages to average
        self.filter_buffer_size = 4

        # Ensure CSV files exist and are initialized
        for marker_id, data in self.data.items():
            with open(data['csv_file'], 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Transformation Number',
                                 'Time Between Messages', 'Moving Average Time Between Messages',
                                 'Translation_x', 'Filtered Translation_x',
                                 'Translation_y', 'Filtered Translation_y',
                                 'Translation_z', 'Filtered Translation_z',
                                 'Rotation_x', 'Filtered Rotation_x',
                                 'Rotation_y', 'Filtered Rotation_y',
                                 'Rotation_z', 'Filtered Rotation_z',
                                 'Rotation_w', 'Filtered Rotation_w'])

    def tf_callback(self, tf_msg: TFMessage):

        # Process each transform in the TFMessage
        for transform in tf_msg.transforms:
            try:
                current_time = transform.header.stamp.sec + transform.header.stamp.nanosec / 1e9
                child_frame_id = transform.child_frame_id

                # Process the transform for the relevant marker
                if child_frame_id in self.data:
                    self.process_marker(transform, current_time, child_frame_id)
                    self.counter[child_frame_id] += 1

                self.counter['total'] += 1

            except Exception as e:
                self.get_logger().error(f"Could not process transform: {e}")

    def process_marker(self, transform, current_time, marker_id):
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        # Get the time difference for the current marker
        last_time = self.data[marker_id]['last_time']
        if last_time is not None:
            time_diff = current_time - last_time  # Time difference in seconds
            self.data[marker_id]['times'].append(time_diff)

            # Keep only the last X messages
            if len(self.data[marker_id]['times']) > self.moving_average_window:
                self.data[marker_id]['times'].pop(0)

            moving_avg_time_between_messages = sum(self.data[marker_id]['times']) / len(self.data[marker_id]['times']) if self.data[marker_id][
                'times'] else 0

            # Keep rolling windows for translation and rotation
            self.update_window(self.data[marker_id]['trans'], [translation.x, translation.y, translation.z])
            self.update_window(self.data[marker_id]['rot'], [rotation.x, rotation.y, rotation.z, rotation.w])

            # Compute median filter for translations and rotations
            filtered_trans = [statistics.median(x) for x in zip(*self.data[marker_id]['trans'])]
            filtered_rot = [statistics.median(x) for x in zip(*self.data[marker_id]['rot'])]


            self.log_to_csv(marker_id, time_diff, moving_avg_time_between_messages,
                            translation, rotation, filtered_trans, filtered_rot)

        # Update the last time for the current marker
        self.data[marker_id]['last_time'] = current_time

    def update_window(self, window, new_data):
        """Helper function to maintain a rolling window of values."""
        window.append(new_data)
        if len(window) > self.filter_buffer_size:
            window.pop(0)

    def log_to_csv(self, marker_id, time_diff, moving_avg_time_between_messages,
                   translation, rotation, filtered_trans, filtered_rot):
        csv_file = self.data[marker_id]['csv_file']
        with open(csv_file, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([self.counter[marker_id], time_diff, moving_avg_time_between_messages,
                             translation.x, filtered_trans[0],
                             translation.y, filtered_trans[1],
                             translation.z, filtered_trans[2],
                             rotation.x, filtered_rot[0],
                             rotation.y, filtered_rot[1],
                             rotation.z, filtered_rot[2],
                             rotation.w, filtered_rot[3]])

def main(args=None):
    rclpy.init(args=args)
    node = TFListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
