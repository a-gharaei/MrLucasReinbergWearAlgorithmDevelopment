import trimesh
from SimulationToolbox.Geometry.geometric_objects import Mesh
from SimulationToolbox.Geometry.geometry import Vector
from SimulationToolbox.PhysicalObjects import grain


def from_mesh_to_trimesh(mesh: Mesh):
    vertices = []
    for vector in mesh.vertices:
        vertice = [vector.x(), vector.y(), vector.z()]
        vertices.append(vertice)
    return trimesh.Trimesh(vertices, mesh.triangle_indices)


def from_trimesh_to_mesh(trimesh_mesh: trimesh.base.Trimesh, pose):
    vertices = [
        Vector(vertice[0], vertice[1], vertice[2]) for vertice in trimesh_mesh.vertices
    ]
    triangles = trimesh_mesh.faces.tolist()
    return Mesh(vertices, triangles, pose)
