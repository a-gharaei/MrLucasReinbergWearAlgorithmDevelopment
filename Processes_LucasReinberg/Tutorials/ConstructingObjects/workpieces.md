# How to construct a Workpiece

A workpiece is modelled as one to many `FlatManifold`'s (slices) in a list, and a `Pose`. Two additional properties called 'distance_to_points' and 'single_point_distance' is also required to proper functioning of proximity calculations and material removal estimation. These callables represents the workpiece boundary, i.e. for a given vector they calculate if the vector is inside the workpiece or not. This is used in the Material Removal class. 

```python
class Workpiece:
    distance_to_points: Callable[[List[Vector]], List[float]]
    single_point_distance: Callable[[List[Vector]], List[float]]
    slices: List[FlatManifold]
    pose: Pose
```

The code of this tutorial can be found here: [construction_objects.py](./../../Tutorials/ConstructingObjects/construction_objects.py)
## Workpiece from box

```python
# workpiece from box
pose = Pose.from_rotation_axis_angle(Vector.e_z(),np.pi/8)
box = Box.from_sizes(size_x=5, size_y=5, size_z=5, pose=pose)

workpiece = Workpiece.from_box(box, spatial_resolution=1)

plot_workpiece(workpiece, WorkpiecePlotConfig.default(workpiece))
```

![drawing](../../Resources/Images/Tutorials/ConstructingObjects/workpiece_from_box.png)

> **Note:** A workpiece once defined can not be moved after. If one wants to have a Workpiece in a certain pose it has to be done while creating it. If needed, this functionality can be implemented in the future.

## Workpiece from a cylinder with star-shaped slices

A workpiece can be constructed with `Workpiece.from_cylinder` factory method. The slicing is done in a star-shaped manner, making it suitable for specific use cases such as core drilling.

```python
# workpiece from cylinder
cylinder = Cylinder(radius=5.0,
                    z_min=-5.0,
                    z_max=5.0,
                    pose=Pose.identity())

workpiece = Workpiece.from_cylinder(cylinder, 
                                    spatial_resolution=0.1)

plot_workpiece(workpiece, WorkpiecePlotConfig.default(workpiece))
```

![drawing](../../Resources/Images/Tutorials/ConstructingObjects/workpiece_from_cylinder.png)

## Workpiece from extruded volume

A workpiece can be constructed by extruding a profile along the orthogonal direction with `Workpiece.from_extruded_volume` class method. The Extruded Volume and Spatial Resolution (distance between slices) are the required inputs. The extrusion profile should be a FlatManifold. The slicing is done along vertical planes parallel to the x-axis of the volume.

```python
position = Vector(0, 0, 0)
pose = Pose.from_axis_angle_translation(
    Vector.e_z(), np.pi/4.0, position)
x_points = np.array([1, 1, 1.4, 1.5, 1.4, 1, 1, 2, 2])
z_points = np.array([1, 1.3, 1.35, 1.5, 1.65, 1.7, 2, 2, 1])
extruded_volume = ExtrudedVolume(
    FlatManifold.from_x_z_arrays(x_points, z_points, pose), extrusion_length=1.5)

workpiece = Workpiece.from_extruded_volume(
    extruded_volume, spatial_resolution=0.5)
plot_workpiece(workpiece, WorkpiecePlotConfig.default(workpiece))
```

![drawing](../../Resources/Images/Tutorials/ConstructingObjects/workpiece_from_extruded_volume.png)

## Manually define a Workpiece

FlatManifolds are the representation of bounded 2D areas embedded in 3D space, consisting of a Area2D and a Pose. The Poses of a FlatMainfold in workpiece construction are expected to live in workpiece Frame. Here, a list of FlatManifolds (representing workpiece slices):

```python
# manually define a workpiece
list_manifold = []
for i in range(10):
    position = Vector(0.5, 0.5 + i*0.1, 0)
    pose = Pose.from_rotation_first_then_translation(
        Vector.e_z(), i*np.pi/20.0, position)
    z_points = 5*(-np.array([1, 1, 1.4, 1.5, 1.4, 1, 1, 2, 2])+2)
    x_points = 5*(np.array([1, 1.3, 1.35, 1.5, 1.65, 1.7, 2, 2, 1])-1)
    list_manifold.append(
        FlatManifold.from_x_z_arrays(x_points, z_points, pose))

# construct workpiece
workpiece = Workpiece(list_manifold, Pose.identity())

# plot workpiece
plot_workpiece(workpiece, WorkpiecePlotConfig.default(workpiece))
```

![drawing](../../Resources/Images/Tutorials/ConstructingObjects/workpiece_from_init.png)
