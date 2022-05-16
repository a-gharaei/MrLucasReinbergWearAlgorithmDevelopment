"""This scripts relates to the simulation of abrasive rail grinding"""
import os
import time
import sys

from pandas import DataFrame

from RailGrinding.AnalyzeSimulationResults import historical_data
from RailGrinding.PhysicalModels.force_controlled_kinematics import ProportionalController, \
    ProportionalControllerResultList
from RailGrinding.PhysicalModels.kienzle_force_model import *
from RailGrinding.PhysicalModels.takazawa_temperature_model import *
from RailGrinding.PhysicalModels.Usui_wear_model import *
from RailGrinding.PhysicalModels.controller import *
from RailGrinding.Resources.GrindingWheels.grinding_wheel_factory import create_wheel

# importing required modules from Simulation Toolbox
from SimulationToolbox.Visualization.visualization import *
from SimulationToolbox.Simulation.material_removal import *
from SimulationToolbox.PhysicalObjects.tool import *
from SimulationToolbox.PhysicalObjects.grain import *
from SimulationToolbox.Simulation.kinematics import Kinematics


def process_simulation(input_df: DataFrame, output_path: str):
    sim_params = {}
    # Create experiment folders and run simulation
    for exp in range(input_df.shape[0]):
        os.makedirs(output_path + '/Exp' + str(exp) + '/PKLFiles')
        if bool(input_df['txt_log_file'][exp]):
            terminal_log_file = open(
                output_path + '/Exp' + str(exp) + '/Multi_Simulation_Run_Log', 'w')
            sys.stdout = terminal_log_file

        # ----------------------------------Definition of tool---------------------------------- #
        tool_creation_time0 = time.time()
        if bool(input_df['load_tool_from_disk'][exp]):

            # load_path = r'./RailGrinding/Resources/GrindingWheels'+'/wheel#'+str(input_df['tool_template_number'][exp])#
            # correct folder tool = Tool.load_from_disk(load_path+'/wheel#'+str(input_df['tool_template_number'][
            # exp])+'.pkl')
            loading_directory = r'./RailGrinding/Resources/GrindingWheels/wheel#' + str(
                int(input_df['wheel_number_to_load'][exp]))
            tool = Tool.load_from_disk(
                loading_directory + '/wheel#' + str(int(input_df['wheel_number_to_load'][exp])) + '.pkl')
            tool_properties_df = pd.read_csv(loading_directory + '/wheel properties.csv')
            sim_params = {'tool_df': tool_properties_df}

        else:
            tool, tool_properties = create_wheel(
                {'grain_stl_path': r'./RailGrinding/Resources/Grains/Cuboctahedron.stl',
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

        historical_data.save_to_disk_as_pkl({'first_state_tool': tool},
                                            path=output_path + '/Exp' + str(exp) + '/PKLFiles')
        tool_properties_df.to_csv(output_path + '/Exp' + str(exp) + '/PKLFiles' + '/wheel_properties.csv')

        tool_creation_time1 = time.time()
        print("tool_creation_time: ", tool_creation_time1 - tool_creation_time0)

        # ----------------------------------Definition of workpiece---------------------------------- #
        wp_creation_time0 = time.time()

        if bool(input_df['load_workpiece_from_disk'][exp]):
            workpiece_load_directory = r'./RailGrinding/Resources/Workpieces/workpiece#' + \
                                       str(int(input_df['workpiece_number_to_load'][exp].item())) + \
                                       '/workpiece#' + str(int(input_df['workpiece_number_to_load'][exp].item())) + \
                                       '.pkl'

            wp = Workpiece.load_from_disk(workpiece_load_directory)
            historical_data.save_to_disk_as_pkl({'first_state_workpiece': wp},
                                                path=output_path + '/Exp' + str(exp) + '/PKLFiles')
        else:
            box = Box.from_sizes(size_x=input_df['wp_x[mm]'][exp],
                                 size_y=input_df['wp_y[mm]'][exp],
                                 size_z=input_df['wp_z[mm]'][exp],
                                 pose=Pose.from_translation(Vector(0, -input_df['wp_y[mm]'][exp] / 2, 0)))
            wp = Workpiece.from_box(box, spatial_resolution=input_df['wp_spatial_resolution[mm]'][exp])
            historical_data.save_to_disk_as_pkl({'first_state_workpiece': wp},
                                                path=output_path + '/Exp' + str(exp) + '/PKLFiles')
            if input_df['save_workpiece_as_template_in_resources'][exp]:
                file_name = 'workpiece#' + \
                            str(int(input_df['template_workpiece_number'][exp].item()))
                saving_path = r'./RailGrinding/Resources/Workpieces/workpiece#' + str(
                    int(input_df['template_workpiece_number'][exp].item()))
                if not os.path.exists(saving_path):
                    os.mkdir(saving_path)
                historical_data.save_to_disk_as_pkl(
                    {file_name: wp}, path=os.path.join(saving_path))

        workpiece_material_propertise = WorkpieceMaterialProperties(
            energy_partition_coefficient=float(
                input_df['energy_partition_coefficient[-]'][exp]),
            thermal_conductivity=float(
                input_df['thermal_conductivity[W/mK]'][exp]),
            density=float(input_df['density[Kg/m3]'][exp]),
            specific_heat_capacity=float(
                input_df['specific_heat_capacity[J/KgK]'][exp]),
            melting_point=float(input_df['workpiece_melting_point[K]'][exp]))

        # plot_workpiece(wp, WorkpiecePlotConfig.default(wp))
        wp_creation_time1 = time.time()
        print("wp_creation_time, ", wp_creation_time1 - wp_creation_time0)
        grain0_minZs = []
        # ----------------------------------Definition of kinematics---------------------------------- #
        feed_rate = input_df['feed_rate[mm/s]'][exp]  # mm/sec
        # mm (height_of_workpiece + grain_z_radius - ap penetration )
        start_point = Vector(input_df['wheel_center_x_position[mm]'][exp],
                             input_df['wheel_center_y_position[mm]'][exp],
                             input_df['wheel_center_z_position[mm]'][exp])
                # tool_properties_df.iloc[0]['max_grains_central_penetration']
                # - input_df['grain_penetration_depth[mm]'][exp]))
        # ap = 7.814.32 micro meter
        rotation_axis = Vector.e_z(). \
            transform(Transform.from_axis_angle(Vector.e_y(),
                                                (np.pi / 180) * input_df['approach_angle[deg]'][exp])). \
            transform(Transform.from_axis_angle(Vector.e_x(),
                                                (np.pi / 180) * input_df['tilt_angle[deg]'][exp]))
        # "rad/sec"
        rotational_speed = input_df['rotational_speed[Hz]'][exp] * 2 * np.pi
        feed = Vector(feed_rate, 0, 0)  # mm/sec
        end_time = input_df['total_process_time[s]'][exp]  # sec
        time_step_size = 1 / (input_df['simulation_step_size[steps/rev]'][exp] * input_df['rotational_speed[Hz]'][
            exp])  # 60 [rev/s] --> 1/(360* 60) = 4.63e-5
        kin_creation_time0 = time.time()
        kin = Kinematics.from_feedrate_and_rotation_speed(start_point, rotation_axis,
                                                          rotational_speed, feed,
                                                          end_time, time_step_size)
        kin_creation_time1 = time.time()
        print("kin_creation_time: ", kin_creation_time1 - kin_creation_time0)

        # visualize Kinematics
        plot_kinematics(kin, KinematicsPlotConfig.default(),
                        library=PlotLibrary.plotly)
        # visualize process
        process_creation_time0 = time.time()
        process = Process(tool, wp, kin)
        # plot_process(process, ProcessPlotConfig.default(process))
        process_creation_time1 = time.time()
        print("process_creation_time: ",
              process_creation_time1 - process_creation_time0)
        # visualize grain trajectory
        # plot_grain_trajectory(process=process,
        #                          grain_index=0,
        #                          plot_config=ProcessPlotConfig.default(process))

        # ----------------------------------Simulation setup---------------------------------- #
        sim_preparation_time0 = time.time()
        max_grain_bounding_sphere_radius = tool.get_max_bounding_sphere_radius()
        # maximum_distance_travelled_by_grain = 2.246097539698602
        maximum_distance_travelled_by_grain = kin.get_max_travelled_distance_of_grains(
            tool, kin.ToolTrajectory)
        start_tool_pose = kin.ToolTrajectory.poses[0]

        collider_config = CollisionDetectionConfig.for_partial_tool_wp_contact_process(tool, wp, kin)

        mat_removal_config = MaterialRemovalConfig(bool(input_df['use_reduced_grain'][exp]),
                                                   bool(input_df['check_approach_angle'][exp]),
                                                   bool(input_df['wear_model'][exp]),
                                                   collider_config)
        matRemover = MaterialRemover(wp,
                                     tool,
                                     start_tool_pose,
                                     kin,
                                     mat_removal_config)
        # for release 1.2.0
        # matRemover = MaterialRemover(wp,
        #                              tool,
        #                              start_tool_pose,
        #                              maximum_distance_travelled_by_grain,
        #                              max_grain_bounding_sphere_radius,
        #                              kin,
        #                              bool(input_df['use_reduced_grain'][exp]),
        #                              bool(input_df['check_approach_angle'][exp]))

        spec_cutting_force = input_df['specific_cutting_force[N/mm2]'][exp]
        force_model_config = GrainForceModelConfiguration(specific_cutting_force=spec_cutting_force,
                                                          grinding_force_ratio=input_df['grinding_force_ratio[]'][exp])

        drive_sys_config = DriveSystemConfiguration.from_config_file(
            path=r"./RailGrinding/PhysicalModels/configs.yaml")
        if bool(input_df['controller_model'][exp]):
            cont = PidController(parameter_i=input_df['parameter_i'][exp].item(),
                                 parameter_p=input_df['parameter_p'][exp].item(
                                 ),
                                 parameter_t=input_df['parameter_t'][exp].item(
                                 ),
                                 nominal_current=input_df['controller_nominal_current[A]'][exp].item(
                                 ),
                                 controller_timing=int(
                                     input_df['controller_timing[s]'][exp].item()),
                                 change_in_z_position_by_controller=input_df['change_in_z_position_by_controller[mm]'][
                                     exp].item())  # every x steps

        if bool(input_df['force_controlled'][exp]):
            cont = ProportionalController(proportional_gain=float(
                input_df['proportional_gain[mm]'][exp]))  # every x steps

        grain_force_results = []
        tool_force_results = []

        # get_maximum_surface_grinding_temperature_results = []
        # temperature_gradient_coefficient_results = []

        sim_preparation_time1 = time.time()
        print("sim_preparation_time: ",
              sim_preparation_time1 - sim_preparation_time0)

        # ----------------------------------Simulation loop---------------------------------- #
        simulation_wear_results = SimulationWearModelResults([])
        simulation_controller_results = PidControllerResultList([])
        simulation_force_controlled_results = ProportionalControllerResultList([
        ])
        simulation_temperature_results = TemperatureResultList([])
        simulation_material_removal_results = MaterialRemovalResultsList([])

        sim_i = 0

        starttime = time.time()
        for current_tool_pose in kin.ToolTrajectory.poses:
            # print("z is : " , current_tool_pose.value.value[2][3])
            # remove material

            current_loop_time0 = time.time()
            mat_remover_result = matRemover.update(wp,
                                                   tool,
                                                   current_tool_pose,
                                                   mat_removal_config)
            # for release 1.2.0
            # mat_remover_result = matRemover.update(wp,
            #                                        tool,
            #                                        current_tool_pose,
            #                                        maximum_distance_travelled_by_grain,
            #                                        max_grain_bounding_sphere_radius,
            #                                        apply_wear=bool(input_df['wear_model'][exp]))
            if bool(input_df['save_material_removal_results'][exp]):
                simulation_material_removal_results.material_removal_result_list.append(
                    mat_remover_result)
            mesh_gc = tool.grains[0].get_mesh().to_stl_mesh()
            grain0_minZs.append(mesh_gc.z.min())

            # --------------- Force model application
            if bool(input_df['force_model'][exp]):
                grain_force_model_result = grain_force_model(
                    mat_remover_result, tool, force_model_config)
                tool_force_model_result = tool_force_model(
                    grain_force_model_result, tool)
                # print("Tool_force", tool_force_model_result.force.value)

                # capture results in a list
                if bool(input_df['save_force_results'][exp]):
                    tool_force_results.append(tool_force_model_result)
                    grain_force_results.append(grain_force_model_result)

            # --------------- Temperature model application
            if bool(input_df['temperature_model'][exp]):
                grain_temperature_model_result = grain_temperature_model(mat_remover_result,
                                                                         workpiece_material_propertise,
                                                                         spec_cutting_force,
                                                                         time_step_size)
                if bool(input_df['save_temperature_results'][exp]):
                    simulation_temperature_results.temperature_results_list.append(
                        grain_temperature_model_result)

            # --------------- usui attritious wear model application
            if bool(input_df['wear_model'][exp]):
                wear_model_result = apply_attritious_wear_model(
                    tool,
                    wear_magnitude_as_penetration_depth_percentage,
                    mat_remover_result,
                    grain_temperature_model_result,
                    input_df['ambient_temperature[K]'][exp],
                    workpiece_material_propertise,
                    input_df['arrhenius_constant[K]'][exp],
                    spec_cutting_force,  # [N/mm^2]
                    time_step_size,
                    input_df['wear_factor[-]'][exp])

                # for iBrus Core version 1.2.0
                # apply_attritious_wear(
                #     wear_model_result, tool, mat_remover_result)

                apply_attritious_wear(
                    wear_model_result, tool, mat_remover_result, input_df['use_reduced_grain'][exp])

                if bool(input_df['save_wear_results'][exp]):
                    simulation_wear_results.simulation_wear_model_results.append(
                        wear_model_result)

            # --------------- controller
            if bool(input_df['controller_model'][exp]):
                controller_result = cont.get_controller_results(rotational_speed,
                                                                tool_properties_df.iloc[0]['average_radius'],
                                                                # average radius in [m]
                                                                tool_force_results,
                                                                kin.ToolTrajectory,
                                                                sim_i,
                                                                int(input_df['controller_timing[s]'][exp].item(
                                                                )),
                                                                drive_sys_config)
                if bool(input_df['save_controller_results'][exp]):
                    simulation_controller_results.pid_controller_results_list.append(
                        controller_result)

            # --------------- forced controlled kinematics
            if bool(input_df['force_controlled'][exp]):
                controller_result = cont.get_proportional_controller_results(tool_force_results,
                                                                             kin.ToolTrajectory,
                                                                             sim_i,
                                                                             int(input_df[
                                                                                 'controller_timing[No.Steps]'][
                                                                                 exp].item(
                                                                             )),
                                                                             float(
                                                                                 input_df['reference_normal_force[N]'][
                                                                                     exp].item()),
                                                                             float(input_df['wear_compensation[mm]'][
                                                                                       exp].item()))
                if ((sim_i + 1) % int(input_df['controller_timing[No.Steps]'][exp].item())) == 0:
                    print("force_error", controller_result.force_error)
                    print("current_z_position",
                          controller_result.current_z_position)
                    print("next_wheel_z_position",
                          controller_result.next_wheel_z_position)

                if bool(input_df['saved_force_controlled_results'][exp]):
                    simulation_force_controlled_results.proportional_controller_results_list.append(
                        controller_result)

            current_loop_time1 = time.time()
            print(sim_i, ", ", current_loop_time1 - current_loop_time0)
            sim_i = sim_i + 1
        endtime = time.time()
        print("total_loop_time: ", endtime - starttime)

        # ----------------------------------Handling of historical data---------------------------------- #

        grain0df = pd.DataFrame({'grain0minZ': grain0_minZs})
        grain0df.to_csv(output_path + 'grain0.csv')

        result_time0 = time.time()
        if bool(input_df['save_force_results'][exp]):
            simulation_results_grain_forces = GrainForceModelResultListNumpyArrays.from_grain_force_model_result_list(
                grain_force_results)
            simulation_results_tool_forces = ToolForceModelResultListFloat.from_tool_force_model_result_list(
                tool_force_results)
            # for release 1.2.0
            # simulation_results_tool_global_forces = simulation_results_tool_forces.get_forces_in_global(
            #    kin.ToolTrajectory.poses)
            simulation_results_tool_global_forces = simulation_results_tool_forces.get_forces_in_global(
                kin.ToolTrajectory)

        result_time1 = time.time()
        print("transforming results to lists", result_time1 - result_time0)

        save_pkl_files_time0 = time.time()
        historical_data.save_to_disk_as_pkl({'last_state_workpiece': wp,
                                             'last_state_tool': tool,
                                             'last_state_kinematics': kin,
                                             'simulation_tool_forces': simulation_results_tool_forces,
                                             'simulation_grain_forces': simulation_results_grain_forces,
                                             'simulation_tool_forces_global_frame':
                                                 simulation_results_tool_global_forces,
                                             'simulation_controller_results': simulation_controller_results,
                                             'simulation_force_controlled_results': simulation_force_controlled_results,
                                             'simulation_temperature_results': simulation_temperature_results,
                                             'simulation_wear_results': simulation_wear_results,
                                             'simulation_material_removal_results':
                                                 simulation_material_removal_results},
                                            path=output_path + '/Exp' + str(exp) + '/PKLFiles')

        save_pkl_files_time1 = time.time()
        print("save_pkl_files_time", save_pkl_files_time1 - save_pkl_files_time0)

        if bool(input_df['txt_log_file'][exp]):
            terminal_log_file.close()
