import math
import numpy as np

def get_center(pts_x, pts_y):
    RESOLUTION = 50
    LOCALITY = 2.0

    dx = max(pts_x) - min(pts_x)
    dy = max(pts_y) - min(pts_y)

    delta = min(dx, dy) / RESOLUTION
    nx = int(dx / delta)
    ny = int(dy / delta)
    radius = (1 / LOCALITY) * min(dx, dy)

    grid_x = np.linspace(min(pts_x), max(pts_x), num=nx)
    grid_y = np.linspace(min(pts_y), max(pts_y), num=ny)

    x, y = np.meshgrid(grid_x, grid_y)


    def gauss(x1, x2, y1, y2):
        """
        Apply a Gaussian kernel estimation (2-sigma) to distance between points.

        Effectively, this applies a Gaussian kernel with a fixed radius to one
        of the points and evaluates it at the value of the euclidean distance
        between the two points (x1, y1) and (x2, y2).
        The Gaussian is transformed to roughly (!) yield 1.0 for distance 0 and
        have the 2-sigma located at radius distance.
        """
        return (
            (1.0 / (2.0 * math.pi))
            * math.exp(
                -1 * (3.0 * math.sqrt((x1 - x2)**2 + (y1 - y2)**2) / radius))**2
            / 0.4)


    def _kde(x, y):
        """
        Estimate the kernel density at a given position.

        Simply sums up all the Gaussian kernel values towards all points
        (pts_x, pts_y) from position (x, y).
        """
        return sum([
            gauss(x, px, y, py)
            # math.sqrt((x - px)**2 + (y - py)**2)
            for px, py in zip(pts_x, pts_y)
        ])


    kde = np.vectorize(_kde)  # Let numpy care for applying our kde to a vector
    z = kde(x, y)

    xi, yi = np.where(z == np.amax(z))
    max_x = grid_x[xi][0]
    max_y = grid_y[yi][0]
    print(f"{max_x:.4f}, {max_y:.4f}")

    return (max_x, max_y)