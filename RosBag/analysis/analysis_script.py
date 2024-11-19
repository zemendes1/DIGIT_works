import pandas as pd
import matplotlib.pyplot as plt
import os

csv_list = ['Time Stamp',
             'Time Between Messages', 'Moving Average Time Between Messages',
             'Translation_x', 'Filtered Translation_x',
             'Translation_y', 'Filtered Translation_y',
             'Translation_z', 'Filtered Translation_z',
             'Rotation_x', 'Filtered Rotation_x',
             'Rotation_y', 'Filtered Rotation_y',
             'Rotation_z', 'Filtered Rotation_z',
             'Rotation_w', 'Filtered Rotation_w']

# Function to read data from a CSV file
def read_csv_data(csv_file):
    df = pd.read_csv(csv_file)
    if df.empty:
        print(f"No data found in {csv_file}")
        return None

    return df[csv_list]


# Function to plot a line graph
def plot_line(ax, x, y, label, xlabel=None, ylabel=None, title=None, linestyle='-', marker='o', color=None, grid=True):
    ax.plot(x, y, label=label, linestyle=linestyle, marker=marker, color=color)
    ax.set_xlabel(xlabel if xlabel else 'Time Stamp')
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    ax.grid(grid)


# Function to plot marker metadata
def plot_marker(marker_id, data, ax_value, ax_filtered, value, moving_avg, label, category_names, ros_bag_path='',
                size_of_buffer=4, plot_values=True, plot_filtered=True):
    y_min, y_max = float('inf'), float('-inf')

    if data is None:
        return

    if plot_values:
        plot_line(ax_value, data['Time Stamp'], y=value,
                  label = f'Marker {marker_id}', ylabel='',
                  title=f'{label}', color='r')

    if plot_filtered:
        plot_line(ax_filtered, data['Time Stamp'], moving_avg,
              f'Marker {marker_id}', ylabel='',
              title=f'{category_names[1]} {label} (Buffer Size={size_of_buffer})', linestyle='--', color='g')

    if min(value) != 0 and max(value) != 0:
        y_min = min(y_min, min(value))
        y_max = max(y_max, max(value))
        ax_value.set_ylim(y_min-abs(y_min*0.1), y_max+abs(y_max*0.1))
        ax_filtered.set_ylim(y_min-abs(y_min*0.1), y_max+abs(y_max*0.1))

    plt.tight_layout()
    os.makedirs(f'{ros_bag_path}/out/{marker_id}', exist_ok=True)

    plt.savefig(f'{ros_bag_path}/out/{marker_id}/{label}.png')
    plt.show()


# Main function to read the CSVs and generate the plots
def main():
    ros_bag_path = 'bag_241024_drone/'
    markers = {'marker_111_to_marker_0': 'marker_111_to_marker_0.csv', 'marker_222_to_marker_0': 'marker_222_to_marker_0.csv', 'marker_111_to_marker_222': 'marker_111_to_marker_222.csv'}
    # markers = {'base_link1_to_world': 'base_link1_to_world.csv', 'base_link2_to_world': 'base_link2_to_world.csv', 'base_link1_to_base_link2': 'base_link1_to_base_link2.csv'}

    categories = 2

    for marker_id, csv_file in markers.items():
        data = read_csv_data(ros_bag_path+csv_file)

        if data is None:
            continue

        # Define the triplet of columns for value, moving average, and standard deviation
        labels = ['Translation X (m)', 'Translation Y (m)', 'Translation Z (m)',
                      'Rotation X', 'Rotation Y', 'Rotation Z', 'Rotation W']

        fig, (ax_value, ax_moving_avg) = plt.subplots(categories, 1, figsize=(10, 5))
        plot_marker(marker_id, data, ax_value, ax_moving_avg, data[csv_list[1]], data[csv_list[2]],"Time Between Messages (s)",
                    ['Time Between Messages (s)', 'Moving Average'], ros_bag_path, 10)

        for i in range(3, len(csv_list), categories):  # Step by 2 to access sets of value, filtered_value
            value = data[csv_list[i]]  # Value column
            filtered_value = data[csv_list[i + 1]]  # Filtered column
            label = labels[(i - 2) // categories]

            fig, (ax_value, ax_filtered) = plt.subplots(categories, 1, figsize=(10, 5))
            plot_marker(marker_id, data, ax_value, ax_filtered, value, filtered_value, label,
                        ['Time Between Messages (s)', 'Median Filtered'], ros_bag_path)



if __name__ == '__main__':
    main()
