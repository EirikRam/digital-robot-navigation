"""Visualization helpers for navigation results."""

import matplotlib.pyplot as plt


def visualize_path(environment, path):
    if not path:
        print("No valid path found to the goal.")
        return

    print("Path found:", path)

    fig, ax = plt.subplots()
    ax.imshow(environment.grid, cmap="gray")

    path_coords = [(state[1][0], state[1][1]) for state in path if state[0] is not None]
    path_x, path_y = zip(*path_coords)

    ax.plot(path_y, path_x, color="red", marker="o", linestyle="-")
    ax.plot(path_y[0], path_x[0], "bs", markersize=12)
    ax.plot(path_y[-1], path_x[-1], "gs", markersize=12)

    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_title("Robot Navigation Path")
    plt.show()
