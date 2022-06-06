import random

import matplotlib.pyplot as plt
import numpy as np
import trimesh
from scipy.spatial import ConvexHull
from shapely.geometry import LineString

import geometrics as geom
import helpers as hlp
from geometrics import Vector
from grain import Grain2D, Grain3D

ex = np.array([1, 0], dtype=float)  # Unit vector in x-direction
ey = np.array([0, 1], dtype=float)  # Unit vector in y-direction


def get_contact_length(grain: Grain2D) -> float:
    """Calculates grain-chip contact length

    Args:
        grain (Grain2D): Instance of Grain2D

    Returns:
        float: Returns contact grain-chip contact length
    """

    intersection, area = hlp.polygonIntersection(
        grain.vertices[-1], hlp.cuttingRectangle(grain)
    )  # Intersection between grain and workpiece
    return np.linalg.norm(
        geom.get_direction_vector(grain.cutting_edge, intersection[1])
    )


def align_grain(grain: Grain3D, cutting_direction: Vector) -> trimesh.base.Trimesh:
    """Rotates the grain around the z-axes so that the moving direction points at the negativ x-direction.
    # * Has to be done in order to simplify the crack calculation and cutting of the mesh

    Args:
        grain (Grain3D): Instance of Grain3D which has to be align
        cutting_direction (Vector): Moving direction of the grain

    Returns:
        trimesh.base.Trimesh: align mesh of type Trimesh
    """
    cutting_direction.project_onto("xy")
    rotation_angle = np.pi - Vector(1, 0, 0).angle_xy(cutting_direction)
    rotated_mesh = geom.mesh_rotation(grain.mesh, rotation_angle, "z")
    return rotated_mesh, rotation_angle


def project_grain_on_xz(mesh: trimesh.base.Trimesh) -> np.ndarray:
    """Projects grain on the xz plane by setting the y-values equal to zero

    Args:
        mesh (trimesh.base.Trimesh): Mesh to be projected

    Returns:
        np.ndarray: Polygon of the projection
    """
    area = list(
        trimesh.path.polygons.projected(mesh, np.array([0, 1, 0])).exterior.coords
    )
    area = np.array([np.array(x) for x in area], dtype=object)
    for ind, vertice in enumerate(area):
        temp_x = vertice[0]
        area[ind][0] = vertice[1]
        area[ind][1] = temp_x
    return area


def make_crack(grain: Grain2D) -> np.ndarray:
    """Calculates crack initiation point and direction of the crack.

    Args:
        grain (Grain2D): Instance of Grain2D which has to be fractured

    Returns:
        np.ndarray: crack_initiaion_point (crack_origin),
        np.ndarray: normal vector to the crack direction
    """
    contact_length = get_contact_length(grain)
    # initiation_point = LineString(grain.vertices[0][:]).interpolate(
    #     random.uniform(2, 3) * contact_length
    # )
    initiation_point = LineString(grain.vertices[0][:]).interpolate(
        2.75 * contact_length
    )
    initiation_point = list(initiation_point.coords)
    initiation_point = np.array([initiation_point[0][0], initiation_point[0][1]])
    wp_heigth = grain.cutting_edge[1] + grain.penetration_depth
    wp_x = [1, -1]
    wp_y = [wp_heigth, wp_heigth]

    x1, y1 = hlp.polygon_to_plot(grain.vertices[-1])
    fig, ax = plt.subplots()
    ax.set_xlabel("X [$mm$]")
    ax.set_ylabel("Z [$mm$]")
    plt.plot(x1, y1)
    plt.plot(wp_x, wp_y, color="k", linestyle="dotted")
    plt.plot(initiation_point[0], initiation_point[1], marker="o", color="g")
    ax.set_aspect("equal")
    plt.tight_layout()
    plt.show()

    crack = initiation_point + contact_length * ex
    # theta = random.uniform(
    #     np.pi / 10, np.pi / 3
    # )  # TODO: theta depending on force ratio
    theta = np.pi / 5
    crack_point = geom.point_rotation(crack, theta, initiation_point)
    crack_direction = geom.get_direction_vector(initiation_point, crack_point)
    crack_normal = geom.unit_vector(
        np.array([-crack_direction[1], 0, crack_direction[0]])
    )
    crack_origin = np.array([initiation_point[0], 0, initiation_point[1]])
    return crack_origin, crack_normal


def get_new_vertices(
    old_mesh: trimesh.base.Trimesh,
    additional_vertices: np.ndarray,
    plane_origin: np.ndarray,
    plane_normal: np.ndarray,
) -> np.ndarray:
    """Finds the nre vertices of a cracked plane. Searches all vertices that lie 'over' the cutting plane and appends the cutting polygon

    Args:
        old_mesh (trimesh.base.Trimesh): Unfractured mesh
        additional_vertices (np.ndarray): cutting polygon
        plane_origin (np.ndarray): cutting plane origin
        plane_normal (np.ndarray): cutting plane normal vector

    Returns:
        np.ndarray: array of vertices of the fractured grain
    """
    old_vertices = old_mesh.vertices.tolist()
    additional_vertices = additional_vertices.tolist()
    new_vertices = []
    coefficients = geom.get_plane_coefficents(plane_origin, plane_normal)
    ref_point = np.array([old_mesh.bounds[0][0], 0, old_mesh.bounds[1][2]])
    if hlp.plane_side(ref_point, coefficients) > 0:
        new_vertices.extend(
            point for point in old_vertices if hlp.plane_side(point, coefficients) > 0
        )

    else:
        new_vertices.extend(
            point for point in old_vertices if hlp.plane_side(point, coefficients) < 0
        )

    new_vertices.extend(iter(additional_vertices))
    return np.array([np.array(x) for x in new_vertices], dtype=object), coefficients


def generate_mesh(vertices: np.ndarray) -> trimesh.base.Trimesh:
    """Generates a mesh from a set of vertices
    # ! works only for convex objects

    Args:
        vertices (np.ndarray): vertices of the new mesh

    Returns:
        trimesh.base.Trimesh: Mesh of type Trimesh
    """
    hull = ConvexHull(vertices)
    indices = hull.simplices
    return trimesh.Trimesh(vertices=vertices, faces=indices)
