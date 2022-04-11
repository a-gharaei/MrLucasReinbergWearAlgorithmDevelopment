import meshcut
import numpy as np
import pyclipper
import trimesh
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
from shapely.geometry import Polygon

from geometrics import Vector
from grain import Grain3D

ex = np.array([1, 0], dtype=float)  # Unit vector in x-direction
ey = np.array([0, 1], dtype=float)  # Unit vector in y-direction


def plane_side(point: list, coefficents: np.ndarray) -> float:
    """Returns result of the scalar equation of plane ax + by + cz - d

    Args:
        point (list): [x, y, z]
        coefficents (np.ndarray): [a, b, c, d]

    Returns:
        float: ax + by + cz - d
    """
    return (
        coefficents[0] * point[0]
        + coefficents[1] * point[1]
        + coefficents[2] * point[2]
        - coefficents[3]
    )


def polygonIntersection(subj: np.ndarray, clip: np.ndarray):
    """Given 2 intersecting polygons, it returns the intersection polygon and the area of the intersection polygon

    Args:
        subj (np.ndarray): Array of points of the polygon which is to be clipped
        clip (np.ndarray): Array of points of the polygon which the subject is clipped with

    Returns:
        list: list of points of the intersection polygon,
        float: area of the intersection polygon
    """
    SCALING_FACTOR = 1000
    pc = pyclipper.Pyclipper()
    pc.AddPath(
        pyclipper.scale_to_clipper(subj, SCALING_FACTOR), pyclipper.PT_SUBJECT, True
    )
    pc.AddPath(
        pyclipper.scale_to_clipper(clip, SCALING_FACTOR), pyclipper.PT_CLIP, True
    )
    solution = pyclipper.scale_from_clipper(
        pc.Execute(
            pyclipper.CT_INTERSECTION, pyclipper.PFT_EVENODD, pyclipper.PFT_EVENODD
        ),
        SCALING_FACTOR,
    )
    pgon = Polygon(solution[0])
    return solution[0], pgon.area


def cuttingRectangle(grain):
    """Returns a rectangle to be intersected with the grain to get the contact length depending on the penetration depth

    Args:
        grain (Grain2D): grain that has to be cut

    Returns:
        np.ndarray: edges of the rectangle
    """
    origin = grain.cutting_edge + grain.penetration_depth * ey
    left_upper = origin - 10 * ex
    right_upper = origin + 10 * ex
    right_lower = right_upper - 2 * grain.penetration_depth * ey
    left_lower = left_upper - 2 * grain.penetration_depth * ey
    return np.array([left_upper, right_upper, right_lower, left_lower])


def cuttingRectangle3D(grain, p_depth):
    """Returns a rectangle to be intersected with the grain to further calculate the cutting area

    Args:
        grain (Grain3D): Grain to be intersected
        p_depth (float): Penetration depth of the grain into the workpiece

    Returns:
        np.ndarray: edges of the rectangle
    """
    origin = np.array([0, (grain.cutting_edge[2] + p_depth)])
    left_upper = origin - 10 * ex
    right_upper = origin + 10 * ex
    right_lower = right_upper - 2 * p_depth * ey
    left_lower = left_upper - 2 * p_depth * ey
    return np.array([left_upper, right_upper, right_lower, left_lower])


def get_A_ortho(grain: Grain3D, D: float) -> float:
    """Calculates the area that is orthogonal to the cutting area

    Args:
        grain (Grain3D): Instance of Grain3D
        D (float): Penetration depth of the grain into the workpiece

    Returns:
        float: Area of the grain at the workpiece surface
    """
    z_value = grain.cutting_edge[2] + D
    plane_orig = (0, 0, z_value)
    plane_norm = (0, 0, 1)
    plane = meshcut.Plane(plane_orig, plane_norm)
    mesh = meshcut.TriangleMesh(grain.vertices, grain.faces)
    P = meshcut.cross_section_mesh(mesh, plane)
    A_ortho = Polygon(P[0])
    return A_ortho.area


def get_A_cut(grain: Grain3D, P_DEPTH: float, cutting_direction: Vector) -> float:
    """Calculates the penetrated area of the grain, perpendicular to the cutting direction

    Args:
        grain (Grain3D): Instance of Grain3D
        P_DEPTH (float): Penetration depth of the grain into the workpiece
        cutting_direction (Vector): Moving direction of the grain

    Returns:
        float: Cutting area
    """
    cutting_direction.project_onto("xy")
    area = list(
        trimesh.path.polygons.projected(
            grain.mesh, cutting_direction.value
        ).exterior.coords
    )
    cutting_rectangle = cuttingRectangle3D(grain, P_DEPTH)
    intersection, A_cut = polygonIntersection(area, cutting_rectangle)
    return A_cut


def cross_section(grain: Grain3D, norm: np.ndarray, orig: np.ndarray) -> np.ndarray:
    """Gets a cross section of a 3D mesh at a given plane

    Args:
        grain (Grain3D): Instance of Grain3D
        norm (np.ndarray): Normal vector to the plane that cuts the mesh
        orig (np.ndarray): Origin of the plane that cuts the mesh

    Returns:
        np.ndarray: Array of points of the resulting cutting polygon
    """
    plane = meshcut.Plane(orig, norm)
    mesh = meshcut.TriangleMesh(grain.vertices, grain.faces)
    P = meshcut.cross_section_mesh(mesh, plane)
    return P[0]


def plot_trimesh(mesh: trimesh.base.Trimesh):
    """Plots a mesh of type Trimesh in a 3D coordinate system

    Args:
        mesh (trimesh.base.Trimesh): Mesh to be plotted
    """
    figure = plt.figure()
    axes = figure.add_subplot(projection="3d")
    axes.add_collection3d(
        mplot3d.art3d.Poly3DCollection(mesh.triangles, alpha=0.7, edgecolors="k")
    )
    scale = mesh.vertices.flatten()
    axes.auto_scale_xyz(scale, scale, scale)
    axes.set_xlabel("X")
    axes.set_ylabel("Y")
    axes.set_zlabel("Z")
    plt.show()
