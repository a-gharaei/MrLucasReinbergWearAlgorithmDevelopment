import imp
from logging import raiseExceptions
from msilib.schema import Error

import meshcut
import numpy as np
import pyclipper
import trimesh
from matplotlib import pyplot as plt
from mpl_toolkits import mplot3d
from scipy.spatial import ConvexHull
from shapely.geometry import LineString, Polygon

from FractureWear.geometrics import Vector
from FractureWear.grain import Grain3D

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


def polygonInterscetion2D(subj: list, clip: list):
    if len(subj) < 3:
        print(f"length of subj:{len(subj)}")
        print("subj invalid")
        return 0, 0
    if len(clip) < 3:
        print(f"length of subj:{len(clip)}")
        print("clip invalid")
        return 0, 0
    p1 = Polygon(subj)
    p2 = Polygon(clip)
    if not p1.intersects(p2):
        print("no intersection found")
        return 0, 0
    intersection = p1.intersection(p2)
    return list(intersection.exterior.coords), intersection.area


def polygonIntersection(subj: np.ndarray, clip: np.ndarray):
    """Given 2 intersecting polygons, it returns the intersection polygon and the area of the intersection polygon

    Args:
        subj (np.ndarray): Array of points of the polygon which is to be clipped
        clip (np.ndarray): Array of points of the polygon which the subject is clipped with

    Returns:
        list: list of points of the intersection polygon,
        float: area of the intersection polygon
    """
    SCALING_FACTOR = 100000
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
    if not solution:
        return 0, 0
    pgon = Polygon(solution[0])
    return solution[0], pgon.area


def line_polygon_intersection(polygon: list, line: list):
    if len(polygon) < 3:
        print(f"This aint a polygon:{polygon}")
        return 0
    if len(line) < 2:
        print(f"this aint a line:{line}")
        return 0
    p1 = Polygon(polygon)
    l1 = LineString(line)
    if not l1.intersects(p1):
        return 0
    intersection = l1.intersection(p1)
    return list(intersection.coords)


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
    return [
        left_upper.tolist(),
        right_upper.tolist(),
        right_lower.tolist(),
        left_lower.tolist(),
    ]


def cuttingRectangle3D(grain, p_depth):
    """Returns a rectangle to be intersected with the grain to further calculate the cutting area

    Args:
        grain (Grain3D): Grain to be intersected
        p_depth (float): Penetration depth of the grain into the workpiece

    Returns:
        list: edges of the rectangle
    """
    origin = [0, grain.cutting_edge[2] + p_depth]
    left_upper = origin - 10 * ex
    right_upper = origin + 10 * ex
    right_lower = right_upper - 5 * p_depth * ey
    left_lower = left_upper - 5 * p_depth * ey
    return [
        left_upper.tolist(),
        right_upper.tolist(),
        right_lower.tolist(),
        left_lower.tolist(),
    ]


def get_A_ortho(grain: Grain3D, penetration_depth: float) -> float:
    """Calculates the area that is orthogonal to the cutting area

    Args:
        grain (Grain3D): Instance of Grain3D
        D (float): Penetration depth of the grain into the workpiece

    Returns:
        float: Area of the grain at the workpiece surface
    """
    if ((grain.maxz - grain.minz)) / 2 < penetration_depth:
        z_value = grain.cutting_edge[2] + (grain.maxz - grain.minz) / 2
    else:
        z_value = grain.cutting_edge[2] + penetration_depth
    plane_orig = (0, 0, z_value)
    plane_norm = (0, 0, 1)
    plane = meshcut.Plane(plane_orig, plane_norm)
    mesh = meshcut.TriangleMesh(grain.vertices, grain.faces)
    P = meshcut.cross_section_mesh(mesh, plane)
    if not P:
        return 0, 0
    A_ortho = Polygon(P[0])
    return A_ortho, A_ortho.area


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
    projection_along_cutting_direction = project_mesh_onto_yz(grain.mesh)
    cutting_rectangle = cuttingRectangle3D(grain, P_DEPTH)
    intersection, A_cut = polygonInterscetion2D(
        projection_along_cutting_direction, cutting_rectangle
    )
    if A_cut == 0:
        return 0, 0
    return intersection, A_cut


def cross_section(
    original_mesh: trimesh.base.Trimesh, orig: np.ndarray, norm: np.ndarray
) -> np.ndarray:
    """Gets a cross section of a 3D mesh at a given plane

    Args:
        grain (Grain3D): Instance of Grain3D
        norm (np.ndarray): Normal vector to the plane that cuts the mesh
        orig (np.ndarray): Origin of the plane that cuts the mesh

    Returns:
        np.ndarray: Array of points of the resulting cutting polygon
    """
    plane = meshcut.Plane(orig, norm)
    mesh = meshcut.TriangleMesh(original_mesh.vertices, original_mesh.faces)
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
    axes.set_xlabel("X [mm]")
    axes.set_ylabel("Y [mm]")
    axes.set_zlabel("Z [mm]")
    plt.show()


def project_mesh_onto_yz(mesh: trimesh.base.Trimesh):
    """projects mesh onto y-z plane

    Args:
        mesh (trimesh.base.Trimesh): _description_

    Returns:
        mesh (trimesh.base.Trimesh): _description_
    """
    projected_vertices = []
    for vertice in mesh.vertices:
        new_vertice = [vertice[1], vertice[2]]
        projected_vertices.append(new_vertice)
    hull = ConvexHull(projected_vertices)
    return [projected_vertices[idx] for idx in hull.vertices.tolist()]
