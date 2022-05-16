import stl

working_dir = './RailGrinding/Resources/Grains'
saving_dir = working_dir + '/GrainsSchleifsymposium'
calculated_info = {}
stl_grain = stl.mesh.Mesh.from_file(working_dir + './CuboctahedronStandard.stl')


volume, cog, inertia = stl_grain.get_mass_properties()
print("volume", volume)
calculated_info.update({'grain_volume': float(volume)})
print("cog", cog)
print("inertia", inertia)
print("minx", stl_grain.x.min())
print("maxx", stl_grain.x.max())
print("miny", stl_grain.y.min())
print("maxy", stl_grain.y.max())
print("minz", stl_grain.z.min())
print("maxz", stl_grain.z.max())

stl_grain.vectors *= 0.8967986





stl_grain.save(saving_dir + '/Cuboctahedron_1.7v.stl')

working_dir = './RailGrinding/Resources/Grains'
saving_dir = working_dir + '/GrainsSchleifsymposium'
calculated_info = {}
stl_grain = stl.mesh.Mesh.from_file(saving_dir + '/Cuboctahedron_1.7v.stl')
volume, cog, inertia = stl_grain.get_mass_properties()
print("volume", volume)
calculated_info.update({'grain_volume': float(volume)})
print("cog", cog)
print("inertia", inertia)
print("minx", stl_grain.x.min())
print("maxx", stl_grain.x.max())
print("miny", stl_grain.y.min())
print("maxy", stl_grain.y.max())
print("minz", stl_grain.z.min())
print("maxz", stl_grain.z.max())
