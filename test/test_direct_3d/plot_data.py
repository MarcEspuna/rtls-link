# plot_data.py
import pandas as pd
import matplotlib.pyplot as plt

def plot_data(file_path):
    data = pd.read_csv(file_path)
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(data['x'], data['y'], data['z'])
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.show()

if __name__ == "__main__":
    import sys
    plot_data(sys.argv[1])
