import numpy as np
from shapely.geometry import Polygon


def polygonInterscetion2D(subj: np.ndarray, clip: np.ndarray):
    if len(subj.tolist()) or len(clip.tolist()) < 3:
        print(subj, clip)
        return 0, 0
    p1 = Polygon(subj)
    p2 = Polygon(clip)
    if not p1.intersects(p2):
        print("no intersection found")
        return 0, 0
    intersection = p1.intersection(p2)
    return intersection.area, list(intersection.exterior.coords)


if __name__ == "__main__":
    a = np.array([[0, 0], [0, 1], [1, 1], [1, 0]])
    b = np.array([[0.5, -1], [0.5, 0.5]])
    c = b.tolist()
    print(c, len(c))
