import os
import shutil
import sys
import stl
import yaml
from matplotlib import pyplot as plt
import time
import numpy as np
import pandas as pd

from SimulationToolbox.Geometry.geometry import Pose
from SimulationToolbox.PhysicalObjects.grain import Grain
from SimulationToolbox.PhysicalObjects.tool import Tool
from RailGrinding.AnalyzeSimulationResults import historical_data


def get_wheel_info(grain, tool, inputs, saving_path):
    volumes = []
    minZs = []
    for i in range(len(tool.grains)):
        mesh_gc = tool.grains[i].get_mesh().to_stl_mesh()
        volume, cog, inertia = mesh_gc.get_mass_properties()
        volumes.append(volume)
        minZs.append(mesh_gc.z.min())

    if inputs['save_wheel']:
        # (mu, sigma) = norm.fit(np.array(minZs))
        plt.hist(np.array(minZs), bins=30)
        plt.ylabel('number of grains')
        plt.xlabel('minZs')
        plt.legend(loc="upper left")
        plt.title("Histogram")
        plt.grid(True)

        plt.savefig(os.path.join(saving_path,
                                 'histogram of distances from neutral plane to grains leading edge[mm].jpg'),
                    dpi=300,
                    bbox_inches='tight')

        plt.hist(np.array(volumes), bins=10)  # density=False would make counts
        plt.ylabel('number of grains')
        plt.xlabel('volume')
        plt.savefig(os.path.join(saving_path, 'histogram of grains volume.jpg'), dpi=300, bbox_inches='tight')

        # plot the tool and grain
        # grain_plot_config = GrainPlotConfig.default()
        # tool_plot_config = ToolPlotConfig.default(tool)
        # plot_grain(grain, grain_plot_config)
        # configuration.do_plot=True
        # plot_tool(tool, tool_plot_config)
        # TODO: configurations has to be enhanced to provide user options whether to save or show plots

    average_radius = (inputs['wheel_inner_radius[mm]'] +
                      inputs['wheel_outer_radius[mm]']) * 0.5 * 0.001  # [m]

    info_dict = {'average_radius[m]': average_radius,
                 'average_grains_volume[mm3]': float(abs(sum(volumes) / len(volumes))),
                 'max_grains_central_penetration[mm]': float(abs(min(minZs))),
                 'min_grains_central_penetration[mm]': float(abs(max(minZs))),
                 'average_grains_central_penetration[mm]': float(abs(sum(minZs) / len(minZs)))}
    return info_dict


class WheelProperties:
    grain_type: str
    number_of_grains: float
    random_seed_number: float
    wheel_inner_radius: float
    wheel_outer_radius: float
    average_distance_neutral_plane_to_grains_leading_edge: float
    average_grains_volume: float
    min_distance_neutral_plane_to_grains_leading_edge: float
    wheel_avg_radius: float

    def __init__(self,
                 grain_type=None,
                 number_of_grains=None,
                 random_seed_number=None,
                 wheel_inner_radius=None,
                 wheel_outer_radius=None,
                 average_grains_central_penetration=None,
                 min_grains_central_penetration=None,
                 max_grains_central_penetration=None,
                 average_grains_volume=None,
                 average_radius=None):
        self.grain_type = str(grain_type)
        self.number_of_grains = number_of_grains
        self.random_seed_number = random_seed_number
        self.wheel_inner_radius = wheel_inner_radius
        self.wheel_outer_radius = wheel_outer_radius
        self.average_grains_central_penetration = average_grains_central_penetration
        self.min_grains_central_penetration = min_grains_central_penetration
        self.max_grains_central_penetration = max_grains_central_penetration
        self.average_grains_volume = average_grains_volume
        self.average_radius = average_radius

    @classmethod
    def load_from_yaml_disk(cls, path):
        return None

    def get_as_dictionary(self):
        return dict(filter(lambda item: item[1] is not None, vars(self).items()))

    def get_as_dataframe(self):
        return pd.DataFrame([self.get_as_dictionary()])

    def save_to_disk_as_csv(self, path):
        self.get_as_dataframe().to_csv(path + '/wheel properties.csv')


def create_wheel(inputs):
    if inputs['save_wheel']:
        saving_path = r'./RailGrinding/Resources/GrindingWheels/wheel#' + str(int(inputs['template_wheel_number']))
        if not os.path.exists(saving_path):
            os.makedirs(saving_path)

    else:
        saving_path = None

    calculated_info = {}

    stl_grain = stl.mesh.Mesh.from_file(inputs['grain_stl_path'])
    # stl_grain.vectors *= 0.8660258
    volume, cog, inertia = stl_grain.get_mass_properties()
    print("volume", volume)
    calculated_info.update({'grain_volume': float(volume)})
    print("cog", cog)
    print("inertia", inertia)
    print("minx", stl_grain.x.min())
    print("maxx", stl_grain.x.min())
    print("miny", stl_grain.y.min())
    print("maxy", stl_grain.y.min())
    print("minz", stl_grain.z.min())
    print("maxz", stl_grain.z.max())

    ibrus_grain = Grain.from_stl(inputs['grain_stl_path'], Pose.identity(), Pose.identity())

    # Generate tool that is using this grain shape
    wheel_creation_time0 = time.time()
    grinding_wheel = Tool.from_radius(basic_grain=ibrus_grain,
                                      inner_radius=inputs['wheel_inner_radius[mm]'],
                                      outer_radius=inputs['wheel_outer_radius[mm]'],
                                      number_of_grains=inputs['number_of_grains[-]'],
                                      tool_pose=Pose.identity(),
                                      random_seed_number=inputs['random_seed_number[-]'])
    wheel_creation_time1 = time.time()
    print("Wheel creation time : ", wheel_creation_time1 - wheel_creation_time0)

    calculated_info.update(get_wheel_info(ibrus_grain, grinding_wheel, inputs, saving_path=saving_path))

    wheel_properties = WheelProperties(wheel_inner_radius=inputs['wheel_inner_radius[mm]'],
                                       wheel_outer_radius=inputs['wheel_outer_radius[mm]'],
                                       number_of_grains=inputs['number_of_grains[-]'],
                                       random_seed_number=inputs['random_seed_number[-]'],
                                       average_grains_central_penetration=calculated_info['average_grains_volume[mm3]'],
                                       min_grains_central_penetration=calculated_info[
                                           'min_grains_central_penetration[mm]'],
                                       max_grains_central_penetration=calculated_info[
                                           'max_grains_central_penetration[mm]'],
                                       average_grains_volume=calculated_info['average_grains_volume[mm3]'],
                                       average_radius=calculated_info['average_radius[m]'])

    if inputs['save_wheel']:
        if not isinstance(int(inputs['template_wheel_number']), int):
            raise ValueError('wheel number is an identifier code and should be an int!')

        file_name = 'wheel#' + str(int(inputs['template_wheel_number']))
        historical_data.save_to_disk_as_pkl({file_name: grinding_wheel},
                                            path=os.path.join(saving_path))
        d = {'input_parameters': inputs, 'calculated_info': calculated_info}
        with open(saving_path + '/parameters.yml', 'w') as yaml_file:
            yaml.dump(d, yaml_file, default_flow_style=False)

        wheel_properties.save_to_disk_as_csv(path=saving_path)

        print("The grinding wheel was successfully saved in ", saving_path)

    elif inputs['save_wheel'] is False:
        print('The wheel was not saved in ../RailGrinding/Resources/GrindingWheels!')

    else:
        raise Exception("Wrong save_wheel input! Correct inputs are 'False' or a two-digit integer number")

    return grinding_wheel, wheel_properties


if __name__ == '__main__':
    working_dir = './RailGrinding/Resources/GrindingWheels'
    input_df = pd.read_csv(working_dir+'/input_parameters_wheel.csv')

    if os.path.exists(working_dir+'/CreatedWheels'):
        shutil.rmtree(working_dir+'/CreatedWheels')
    os.makedirs(working_dir+'/CreatedWheels')

    for exp in range(input_df.shape[0]):

        output_path = working_dir+'/CreatedWheels/wheel#' + str(int(input_df['template_wheel_number'][exp]))
        os.makedirs(output_path)
        if bool(input_df['txt_log_file'][exp]):
            terminal_log_file = open(output_path + '/Log', 'w')
            sys.stdout = terminal_log_file

        tool_creation_time0 = time.time()
        tool, tool_properties = create_wheel({'grain_stl_path': r'./RailGrinding/Resources/Grains/Cuboctahedron.stl',
                                              'wheel_inner_radius[mm]': input_df['inner_radius[mm]'][exp].item(),
                                              'wheel_outer_radius[mm]': input_df['outer_radius[mm]'][exp].item(),
                                              'number_of_grains[-]': int(
                                                  input_df['number_of_grains[-]'][exp].item()),
                                              'random_seed_number[-]': int(
                                                  input_df['random_seed_number[-]'][exp].item()),
                                              'save_wheel': bool(input_df['save_as_template'][exp]),
                                              'template_wheel_number': input_df['template_wheel_number'][
                                                  exp].item()})  # a none-zero integer number)
        tool_properties_df = tool_properties.get_as_dataframe()
        sim_params = {'tool_df': tool_properties_df}

        historical_data.save_to_disk_as_pkl({str('wheel#' + str(int(input_df['template_wheel_number'][exp]))): tool},
                                            path=output_path)
        tool_properties_df.to_csv(output_path + '/wheel_properties.csv')
        tool_creation_time1 = time.time()
        print("tool_creation_time: ", tool_creation_time1 - tool_creation_time0)
        # box = Box.from_sizes(size_x=1,
        #                      size_y=1,
        #                      size_z=1,
        #                      pose=Pose.from_translation(Vector(0, -1/2, 0)))
        # wp = Workpiece.from_box(box, spatial_resolution=1)
        #
        # feed_rate = 1  # mm/sec
        # # mm (height_of_workpiece + grain_z_radius - ap penetration )
        # start_point = Vector(0, 0, 0)
        # # ap = 7.814.32 micro meter
        # rotation_axis = Vector.e_z(). \
        #     transform(Transform.from_axis_angle(Vector.e_y(), 0))
        # rotational_speed = 1 * 2 * np.pi
        # feed = Vector(feed_rate, 0, 0)  # mm/sec
        # end_time = 0.001  # sec
        # time_step_size = 1 / 360 *  1 * 2 * np.pi # 60 [rev/s] --> 1/(360* 60) = 4.63e-5
        # kin_creation_time0 = time.time()
        # kin = Kinematics.from_feedrate_and_rotation_speed(start_point, rotation_axis,
        #                                                   rotational_speed, feed,
        #                                                   end_time, time_step_size)
        #
        # process = Process(tool, wp, kin)
        # configuration.do_plot=True
        # plot_process(process, plot_config=ProcessPlotConfig.default(process), dpi=100)

        if bool(input_df['txt_log_file'][exp]):
            terminal_log_file.close()
