import rclpy
from rclpy.node import Node
import tf2_ros
import csv
import statistics
import os

class TFListenerNode(Node):
    def __init__(self):
        super().__init__('tf_listener_node')

        # Create a TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Initialize counters and lists for data tracking
        self.data = {
            'marker_111_to_marker_222': {'times': [], 'last_time': None, 'csv_file': f'marker_111_to_marker_222.csv',
                                'trans': [], 'rot': [], 'last_transform': None},
            'base_link1_to_base_link2': {'times': [], 'last_time': None, 'csv_file': f'base_link1_to_base_link2.csv',
                                         'trans': [], 'rot': [], 'last_transform': None},
        }
        self.counter = {'marker_111_to_marker_222':0, 'base_link1_to_base_link2':0, 'total': 0}
        self.filter_buffer_size = 4

        # Ensure CSV files exist and are initialized
        for marker_id, data in self.data.items():
            with open(data['csv_file'], 'w', newline='') as file:
                writer = csv.writer(file)
                writer.writerow(['Time Stamp',
                                 'Time Between Messages', 'Moving Average Time Between Messages',
                                 'Translation_x', 'Filtered Translation_x',
                                 'Translation_y', 'Filtered Translation_y',
                                 'Translation_z', 'Filtered Translation_z',
                                 'Rotation_x', 'Filtered Rotation_x',
                                 'Rotation_y', 'Filtered Rotation_y',
                                 'Rotation_z', 'Filtered Rotation_z',
                                 'Rotation_w', 'Filtered Rotation_w'])

        # Set timers to periodically lookup and process transforms
        self.timer_marker111 = self.create_timer(0.01, lambda: self.lookup_and_process("marker_111"))
        # self.timer_marker222 = self.create_timer(0.01, lambda: self.lookup_and_process("marker_222"))
        self.timer_base_link1 = self.create_timer(0.01, lambda: self.lookup_and_process("base_link1"))
        # self.timer_base_link2 = self.create_timer(0.01, lambda: self.lookup_and_process("base_link2"))

    def update_window(self, window, new_data):
        """Helper function to maintain a rolling window of values."""
        window.append(new_data)
        if len(window) > self.filter_buffer_size:
            window.pop(0)

    def log_to_csv(self, marker_id, time_stamp, time_diff, moving_avg_time_between_messages,
                   translation, rotation, filtered_trans, filtered_rot):
        csv_file = self.data[marker_id]['csv_file']
        with open(csv_file, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([time_stamp, time_diff, moving_avg_time_between_messages,
                             translation.x, filtered_trans[0],
                             translation.y, filtered_trans[1],
                             translation.z, filtered_trans[2],
                             rotation.x, filtered_rot[0],
                             rotation.y, filtered_rot[1],
                             rotation.z, filtered_rot[2],
                             rotation.w, filtered_rot[3]])

    def lookup_and_process(self, name_of_marker):
        if name_of_marker == "marker_111" or name_of_marker == "marker_222":
            filename = f'{name_of_marker}_to_marker_222'
        elif name_of_marker == "base_link1" or name_of_marker == "base_link2":
            filename = f'{name_of_marker}_to_base_link2'

        try:
            # Lookup transform between marker_xxx and marker_0 or between base_link_xxx and world
            if name_of_marker == "marker_111"  or name_of_marker == "marker_222":
                trans = self.tf_buffer.lookup_transform('marker_111', 'marker_222', rclpy.time.Time())

            elif name_of_marker == "base_link1" or name_of_marker == "base_link2":
                trans = self.tf_buffer.lookup_transform('base_link1', 'base_link2', rclpy.time.Time())
            translation = trans.transform.translation
            rotation = trans.transform.rotation

            # Get the time difference for marker_xxx_to_0
            current_time = int(trans.header.stamp.sec) +  int(trans.header.stamp.nanosec) / 1e9
            last_time = self.data[filename]['last_time']

            if last_time is not None:
                time_diff =current_time - last_time
                self.data[filename]['times'].append(time_diff)

                # Check if the transform is unique (different from the last logged one)
                current_transform = [translation.x, translation.y, translation.z,
                                     rotation.x, rotation.y, rotation.z, rotation.w]
                last_transform = self.data[filename]['last_transform']

                if last_transform is None or current_transform != last_transform:
                    # self.get_logger().info(str(current_transform))

                    # Keep rolling windows for translation and rotation
                    self.update_window(self.data[filename]['trans'],
                                       [translation.x, translation.y, translation.z])
                    self.update_window(self.data[filename]['rot'],
                                       [rotation.x, rotation.y, rotation.z, rotation.w])

                    moving_avg_time_between_messages = sum(self.data[filename]['times']) / len(self.data[filename]['times'])

                    # Compute median filter for translations and rotations
                    filtered_trans = [statistics.median(x) for x in zip(*self.data[filename]['trans'])]
                    filtered_rot = [statistics.median(x) for x in zip(*self.data[filename]['rot'])]

                    # Log to CSV if unique
                    self.log_to_csv(filename, f'{trans.header.stamp.sec}.{trans.header.stamp.nanosec}', time_diff,  moving_avg_time_between_messages,
                                    translation, rotation, filtered_trans, filtered_rot)
                    self.counter[filename] += 1
                    self.counter['total'] += 1
                    # Update the last logged transform
                    self.data[filename]['last_transform'] = current_transform

            # Update the last time for marker_xxx_to_0
            self.data[filename]['last_time'] = current_time

        except Exception as e:
            self.get_logger().warn(f"Could not get transform {filename}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = TFListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()