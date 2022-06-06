import numpy as np
import numpy.linalg as la
import trimesh


class Vector:
    def __init__(self, x: float, y: float, z: float) -> None:
        """Initializes the vector

        Args:
            x (float): x value
            y (float): y value
            z (float): z value
        """
        self.value = np.array([x, y, z])  # Vector as np.ndarray
        self.unit = self.value / la.norm(self.value)  # Normalized vector

    def x(self) -> float:
        """x value of the vector

        Returns:
            float: x value
        """
        return self.value[0]

    def y(self) -> float:
        """y value of the vector

        Returns:
            float: y value
        """
        return self.value[1]

    def z(self) -> float:
        """z value of the vector

        Returns:
            float: z value
        """
        return self.value[2]

    def dot(self, other) -> float:
        return (
            self.value[0] * other.value[0]
            + self.value[1] * other.value[1]
            + self.value[2] * other.value[2]
        )

    def abs(self) -> float:
        """Absolut value of this vector

        Returns:
            float: Absolut value
        """
        return la.norm(self.value)

    def angle_xy(self, other) -> float:
        """Calculates angle between this and another vector, both projected on the x-y plane

        Args:
            other (Vector): Other vector

        Returns:
            float: angle between the two vectors on the x-y plane
        """
        dot = self.value[0] * other.value[0] + self.value[1] * other.value[1]
        det = self.value[0] * other.value[1] - self.value[1] * other.value[0]
        return np.arctan2(det, dot)

    def project_onto(self, plane: str):
        """Projects itself onto a given plane

        Args:
            plane (str): plane the vector is going to be projected on. Can be 'xy', 'xz' or 'yz'
        """
        if all(x in plane for x in ("x", "y")):
            self.value[2] = 0
        if all(x in plane for x in ("x", "z")):
            self.value[1] = 0
        if all(x in plane for x in ("y", "z")):
            self.value[0] = 0


def Rx(theta: float) -> np.ndarray:
    """Rotation matrix around x-axis by given angle theta

    Args:
        theta (float): angle of rotation in radian

    Returns:
        np.ndarray: rotation matrix of shape (3,3)
    """
    return np.array(
        [
            [1, 0, 0],
            [0, np.cos(theta), -np.sin(theta)],
            [0, np.sin(theta), np.cos(theta)],
        ]
    )


def Ry(theta: float) -> np.ndarray:
    """Rotation matrix around y-axis by given angle theta

    Args:
        theta (float): angle of rotation in radian

    Returns:
        np.ndarray: rotation matrix of shape (3,3)
    """
    return np.array(
        [
            [np.cos(theta), 0, np.sin(theta)],
            [0, 1, 0],
            [-np.sin(theta), 0, np.cos(theta)],
        ]
    )


def Rz(theta: float) -> np.ndarray:
    """Rotation matrix around z-axis by given angle theta

    Args:
        theta (float): angle of rotation in radian

    Returns:
        np.ndarray: rotation matrix of shape (3,3)
    """
    return np.array(
        [
            [np.cos(theta), -np.sin(theta), 0],
            [np.sin(theta), np.cos(theta), 0],
            [0, 0, 1],
        ]
    )


def get_rotation_matrix(theta: float, axis_of_rotation: str) -> np.ndarray:
    """Gets rotation matrix according to the axis

    Args:
        theta (float): angle of rotation in radian
        axis_of_rotation (str): axis of rotation as string. Can be 'x', 'y' or 'z'

    Returns:
        np.ndarray: rotation matrix of shape (3,3)
    """
    rot_matrixes = {"x": Rx(theta), "y": Ry(theta), "z": Rz(theta)}
    return rot_matrixes[axis_of_rotation]


def mesh_rotation(
    mesh: trimesh.base.Trimesh, theta: float, axis_of_rotation: str
) -> trimesh.base.Trimesh:
    """Rotates mesh by angle theta around axis_of_rotation

    Args:
        mesh (trimesh.base.Trimesh): mesh of type Trimesh
        theta (float): angle of rotation in radian
        axis_of_rotation (str): axis of rotation as string. Can be 'x', 'y' or 'z'

    Returns:
        trimesh.base.Trimesh: rotated mesh
    """
    rotated_mesh = mesh.copy()
    rotation_matrix = get_rotation_matrix(theta, axis_of_rotation)
    for index, vertice in enumerate(rotated_mesh.vertices):
        rotated_mesh.vertices[index] = rotation_matrix.dot(vertice)
    return rotated_mesh


def point_rotation(
    point: np.ndarray, theta: float, origin=np.array([0, 0, 0]), axis_of_rotation="z"
) -> np.ndarray:
    """Rotates a point around a given origin around the axis of rotation by the angle theta

    Args:
        point (np.ndarray): point to be rotated
        theta (float): angle of rotation
        origin (center_of_rotation, optional): point around other point is rotated. Defaults to (0, 0, 0).

    Returns:
        np.ndarray: rotated point
    """
    rotation_matrix = get_rotation_matrix(theta, axis_of_rotation)
    if point.shape[0] == 2 and origin.shape[0] == 2:
        point = np.array([point[0], point[1], 0])
        origin = np.array([origin[0], origin[1], 0])
        return np.delete(rotation_matrix.dot(point - origin) + origin, 2)
    return rotation_matrix.dot(point - origin) + origin


def get_direction_vector(start: np.ndarray, end: np.ndarray) -> np.ndarray:
    """Vector from start point to end point

    Args:
        start (np.ndarray): start point of the vector
        end (np.ndarray): end point of the vector

    Returns:
        np.ndarray: direction vector from start to end point
    """
    return np.subtract(end, start)


def unit_vector(vector: np.ndarray) -> np.ndarray:
    """Normalized vector of any given vector

    Args:
        vector (np.ndarray): Vector to be normalized

    Raises:
        Exception: Vector has no length : vector = (0, 0, 0)

    Returns:
        np.ndarray: normalized vector
    """
    if np.linalg.norm(vector) == 0:
        raise Exception("Vector has no length!")
    return vector / np.linalg.norm(vector)


def perpendicular_vector(vector: np.ndarray) -> np.ndarray:
    """Calculates vector that is perpendicular to given vector
    Only works for 2D vectors

    Args:
        vector (np.ndarray): initial vector

    Returns:
        np.ndarray: vector perpendicular to initial vector
    """
    return np.ndarray([[-vector[1], vector[0]]])


def get_plane_coefficents(
    point_on_plane: np.ndarray, plane_norm: np.ndarray
) -> np.ndarray:
    """Calculates coefficients a, b, c, d of the scalar equation of plane: a*x + b*y + c+z = d, given a point on a plane with the corresponding normal vector

    Args:
        point_on_plane (np.ndarray): Any point that lies on the plane
        plane_norm (np.ndarray): Normal vector to the plane

    Returns:
        np.ndarray: [a, b, c, d]
    """
    a, b, c = plane_norm[0], plane_norm[1], plane_norm[2]
    d = a * point_on_plane[0] + b * point_on_plane[1] + c * point_on_plane[2]
    return np.array([a, b, c, d])
