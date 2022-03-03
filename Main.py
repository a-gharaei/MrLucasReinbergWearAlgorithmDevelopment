#you may start writing your here here
import stl # this is numpy-stl package

stl_grain = stl.mesh.Mesh.from_file('./Grains/CubeStandard.stl')
# stl_grain.vectors *= 0.8660258
volume, cog, inertia = stl_grain.get_mass_properties()
print("volume", volume)
calculated_info = {}
calculated_info.update({'grain_volume': float(volume)})
print("cog", cog)
print("inertia", inertia)
print("minx", stl_grain.x.min())
print("maxx", stl_grain.x.min())
print("miny", stl_grain.y.min())
print("maxy", stl_grain.y.min())
print("minz", stl_grain.z.min())
print("maxz", stl_grain.z.max())
print(stl_grain.vectors)
print(stl_grain.normals)