import pandas as pd
import matplotlib.pyplot as plt
import os

csv_list = ['Transformation Number',
             'Time Between Messages', 'Moving Average Time Between Messages',
             'Translation_x', 'Moving Average Translation_x', 'Std Translation_x',
             'Translation_y', 'Moving Average Translation_y', 'Std Translation_y',
             'Translation_z', 'Moving Average Translation_z', 'Std Translation_z',
             'Rotation_x', 'Moving Average Rotation_x', 'Std Rotation_x',
             'Rotation_y', 'Moving Average Rotation_y', 'Std Rotation_y',
             'Rotation_z', 'Moving Average Rotation_z', 'Std Rotation_z',
             'Rotation_w','Moving Average Rotation_w', 'Std Rotation_w']

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
    ax.set_xlabel(xlabel if xlabel else 'Transformation Number')
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    ax.grid(grid)


# Function to plot marker metadata
def plot_marker(marker_id, data, ax_value, ax_moving_avg, ax_std_deviation, ax_z_score, value, moving_avg, std_deviation, z_score, label,
                plot_values=True, plot_moving_avg=True, plot_std_deviation=True, plot_z_score=True):
    if data is None:
        return

    if plot_values:
        plot_line(ax_value, data['Transformation Number'], y=value,
                  label = f'Marker {marker_id}', ylabel='',
                  title=f'{label}', color='r')

    if plot_moving_avg:
        plot_line(ax_moving_avg, data['Transformation Number'], moving_avg,
              f'Marker {marker_id}', ylabel='',
              title=f'Moving Average {label} (Window Size=10)', linestyle='--', color='g')

    if plot_std_deviation:
        plot_line(ax_std_deviation, data['Transformation Number'], std_deviation,
              f'Marker {marker_id}', ylabel='',
              title=f'Moving Standard Deviation {label} (Window Size=10)', linestyle='--', color='orange')

    if plot_z_score:
        plot_line(ax_z_score, data['Transformation Number'], z_score,
              f'Marker {marker_id}', ylabel='',
              title=f'Moving Z Score {label} (Window Size=10)', linestyle='--', color='orange')

    plt.tight_layout()
    os.makedirs(f'out/{marker_id}', exist_ok=True)

    plt.savefig(f'out/{marker_id}/{label}.png')
    plt.show()


# Main function to read the CSVs and generate the plots
def main():
    markers = {'marker_0': 'marker_0.csv', 'marker_111': 'marker_111.csv', 'marker_222': 'marker_222.csv'}

    for marker_id, csv_file in markers.items():
        data = read_csv_data(csv_file)

        if data is None:
            continue

        # Define the triplet of columns for value, moving average, and standard deviation
            # Define the triplet of columns for value, moving average, and standard deviation
        labels = ['Translation X (m)', 'Translation Y (m)', 'Translation Z (m)',
                      'Rotation X', 'Rotation Y', 'Rotation Z', 'Rotation W']

        fig, (ax_value, ax_moving_avg) = plt.subplots(2, 1, figsize=(10, 5))
        plot_marker(marker_id, data, ax_value, ax_moving_avg, None, None, data[csv_list[1]], data[csv_list[2]], None,
                    None, "Time Between Messages (s)", True, True, False, False)

        for i in range(3, len(csv_list), 4):  # Step by 4 to access sets of value, moving avg, std dev, z_score
            value = data[csv_list[i]]  # Value column
            moving_avg = data[csv_list[i + 1]]  # Moving Average column
            std = data[csv_list[i + 2]]  # Standard Deviation column
            z_score = data[csv_list[i + 3]]  # Standard Deviation column
            label = labels[(i - 1) // 4]

            fig, (ax_value, ax_moving_avg, ax_std_deviation, ax_z_score) = plt.subplots(4, 1, figsize=(10, 10))
            plot_marker(marker_id, data, ax_value, ax_moving_avg, ax_std_deviation, ax_z_score, value, moving_avg, std, z_score, label)



if __name__ == '__main__':
    main()
