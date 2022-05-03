# Application Notes on iBrus

This note is a document to guide iBrus users to use the tool box in a error-free way. Especially, the aspects to pay attention to while developing with iBrus, based on the technical design of the toolbox.

## Objects and variables are linked with `=` sign, not copied.

When creating a variable in python, the `=` sign is merely passing a reference to an object. Which means the new variable will be linked with the old object and any change in the object will result in the change of the new variable

```python
new_variable = old_object
```

In the following example, if the value for the pose is changed, then both `Mesh_1` and `Mesh_2` will change:

```python
new_pose = Pose.identity()
mesh_1 = Mesh([triangle_1], new_pose)
mesh_2 = Mesh([triangle_1], new_pose)
```

More importantly, because there is `return` object or value for the majority of the methods and the modification is in place, this will result in the following situation:

```python
tool_moved = tool.move(Pose.identity())
```

The `move` is executed in place, so `tool` itself is moved to a new place. While `tool_moved` is now linked with `tool`, they are both representing the tool after moved.

> Notice: This means the tool before move is not accessible anymore, because it's moved.

So when users want to create an independent object that will not be affected if input variable changed, `copy.deepcopy()` pattern should be applied.

```python
new_tool = copy.deepcopy(tool.move(Pose.identity()))
```

This will secure the decoupling of objects, but it induce a minor performance decrease.
