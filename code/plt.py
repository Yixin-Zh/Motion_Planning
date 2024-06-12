import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

def main():
    center = np.array([0, 0, 0])
    length = 2
    d = length / 2
    all_nodes = np.array([[i, j, k] for i in [-d, 0, d] for j in [-d, 0, d] for k in [-d, 0, d]])

    fig = plt.figure("Cube")
    ax = fig.add_subplot(111, projection='3d')
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    ax.set_xlim(-d, d)
    ax.set_ylim(-d, d)
    ax.set_zlim(-d, d)

    # Plot all nodes
    for node in all_nodes:
        if np.array_equal(node, center):
            ax.scatter(node[0], node[1], node[2], color='r', s=100)
            # ax.text(node[0], node[1], node[2], 'current node', color='red', horizontalalignment='center')
        else:
            ax.scatter(node[0], node[1], node[2], color='b', s=50)
            # ax.text(node[0], node[1], node[2], 'neighbor', color='blue', horizontalalignment='center')
            ax.plot([center[0], node[0]], [center[1], node[1]], [center[2], node[2]], color='gray')

    plt.show()

if __name__ == "__main__":
    main()
