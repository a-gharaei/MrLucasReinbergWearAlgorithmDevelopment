import sys

import pandas as pd
from pandas import ExcelWriter
from openpyxl.workbook import Workbook
from RailGrinding.AnalyzeSimulationResults.visualizing_results import *
from RailGrinding.PhysicalModels.Usui_wear_model import SimulationWearModelResults
from RailGrinding.PhysicalModels.controller import PidControllerResultList
from RailGrinding.PhysicalModels.force_controlled_kinematics import ProportionalControllerResultList
from RailGrinding.PhysicalModels.kienzle_force_model import MaterialRemovalResultsList
from RailGrinding.PhysicalModels.drive_system import DriveSystemConfiguration
from RailGrinding.PhysicalModels.drive_system import calculate_spindle_signals

from SimulationToolbox.Simulation.physical_models import ToolForceModelResultListFloat

from RailGrinding.PhysicalModels.takazawa_temperature_model import TemperatureResultList


def result_extraction(input_parameter_df,
                      result_directory,
                      visualization_config,
                      visualization=True,
                      extract_figures=False):
    input_dir = result_directory
    number_of_available_experiment_folders = len(next(os.walk(input_dir))[1])
    for exp in range(number_of_available_experiment_folders):

        if bool(input_parameter_df['txt_log_file'][exp]):
            terminal_log_file = open(
                input_dir + '/Exp' + str(exp) + '/Result_Extraction_Log', 'w')
            sys.stdout = terminal_log_file

        PKLs_path = input_dir + '/Exp' + str(exp) + '/PKLFiles'
        report_df = pd.DataFrame()
        df_collections = []

        if os.path.isfile(os.path.join(PKLs_path, 'wheel_properties.csv')):
            wheel_properties_df = pd.read_csv(os.path.join(PKLs_path, 'wheel_properties.csv'))

        # -------------------------------controller----------------------------
        if os.path.isfile(os.path.join(PKLs_path, 'simulation_controller_results.pkl')):
            controller_df = PidControllerResultList.load_from_disk(
                os.path.join(PKLs_path, 'simulation_controller_results.pkl')).get_in_dataframe()
            if not controller_df.empty:
                df_collections.append(controller_df)

        # -------------------------------force-controlled controller----------------------------
        if os.path.isfile(os.path.join(PKLs_path, 'simulation_force_controlled_results.pkl')):
            force_controlled_df = ProportionalControllerResultList.load_from_disk(
                os.path.join(PKLs_path, 'simulation_force_controlled_results.pkl')).get_in_dataframe()
            if not force_controlled_df.empty:
                df_collections.append(force_controlled_df)

        # -------------------------------forces----------------------------
        if os.path.isfile(os.path.join(PKLs_path, 'simulation_tool_forces_global_frame.pkl')):
            forces_df = ToolForceModelResultListFloat.load_from_disk(
                os.path.join(PKLs_path, 'simulation_tool_forces_global_frame.pkl')).get_in_dataframe()
            if not forces_df.empty:
                df_collections.append(forces_df)

        # -------------------------------temperature----------------------------
        if os.path.isfile(os.path.join(PKLs_path, 'simulation_temperature_results.pkl')):
            simulation_temperature_results = TemperatureResultList.load_from_disk(
                os.path.join(PKLs_path, 'simulation_temperature_results.pkl'))
            number_grains_with_temp_above_ambient = []
            number_grains_with_temp_above_melting_point = []
            sum_of_temperature_rise_of_grains_with_temp_above_ambient = []
            average_grains_temperature_rise = []
            first_grain_temperature_rise = []

            for temperature_model_result in simulation_temperature_results.temperature_results_list:

                grains_with_temperature_above_ambient = list(
                    filter(lambda x: x > 0, temperature_model_result.grains_temperature_rise))

                melting_point_exceeding_threshold = float(input_parameter_df['workpiece_melting_point[K]'][exp] -
                                                          input_parameter_df['ambient_temperature[K]'][exp])
                grains_with_temperature_above_melting_point = list(
                    filter(lambda x: x > melting_point_exceeding_threshold,
                           temperature_model_result.grains_temperature_rise))

                number_grains_with_temp_above_ambient.append(len(grains_with_temperature_above_ambient))
                number_grains_with_temp_above_melting_point.append(len(grains_with_temperature_above_melting_point))

                sum_of_temperature_rise_of_grains_with_temp_above_ambient.append(
                    sum(grains_with_temperature_above_ambient))

                if len(grains_with_temperature_above_ambient) > 0:
                    average_grains_temperature_rise.append(sum(grains_with_temperature_above_ambient) /
                                                           len(grains_with_temperature_above_ambient))
                else:
                    average_grains_temperature_rise.append(int(0))
                first_grain_temperature_rise.append(temperature_model_result.grains_temperature_rise[0])

            temperature_df = pd.DataFrame.from_dict(
                {'No. of grains with temperature above ambient': number_grains_with_temp_above_ambient,
                 'No. of grains with temperature above melting point': number_grains_with_temp_above_melting_point,
                 "Sum of all material-removing grains' temperature rise - K":
                     sum_of_temperature_rise_of_grains_with_temp_above_ambient,
                 'Average temperature rise of all material-removing grains - K': average_grains_temperature_rise,
                 "Temperature rise of grain #1 - K": first_grain_temperature_rise})

            if not temperature_df.empty:
                df_collections.append(temperature_df)

        # -------------------------------material_removal----------------------------
        if os.path.isfile(os.path.join(PKLs_path, 'simulation_material_removal_results.pkl')):
            material_removal_results = MaterialRemovalResultsList.load_from_disk(
                os.path.join(PKLs_path, 'simulation_material_removal_results.pkl'))

            removed_volumes_sum = []
            projected_area_sum = []
            number_of_grains_removing_material = []
            first_grain_penetration_depth = []

            for mat_removal_result in material_removal_results.material_removal_result_list:
                removed_volumes_sum.append(sum(mat_removal_result.removed_volumes))

                grains_projected_area_filtered = list(filter(lambda x: x > 0, mat_removal_result.projected_areas))
                projected_area_sum.append(sum(grains_projected_area_filtered))
                number_of_grains_removing_material.append(len(grains_projected_area_filtered))
                first_grain_penetration_depth.append(mat_removal_result.penetration_depths[0])
            material_removal_df = pd.DataFrame.from_dict({'Sum of removed volumes - mm3': removed_volumes_sum,
                                                          'Sum of projected cutting areas - mm2': projected_area_sum,
                                                          'No. of material-removing grains':
                                                              number_of_grains_removing_material,
                                                          'Penetration depth of grain #1 - mm': first_grain_penetration_depth})
            material_removal_df['Sum of projected cutting areas - micrometer^2'] = \
                1000 * 1000 * material_removal_df['Sum of projected cutting areas - mm2']
            material_removal_df['Cumulative removed volume - mm3'] = material_removal_df[
                'Sum of removed volumes - mm3'].cumsum()

            if not material_removal_df.empty:
                df_collections.append(material_removal_df)

        # -------------------------------spindle signals----------------------------
        if os.path.isfile(os.path.join(PKLs_path, 'simulation_tool_forces_global_frame.pkl')):
            tool_forces_y = list(forces_df["tool force Y"])

            drive_sys_config = DriveSystemConfiguration.from_config_file(
                path=r"./RailGrinding/PhysicalModels/configs.yaml")
            spindle_current_results = []
            spindle_mechanical_power_results = []
            spindle_electrical_power_results = []

            for tangential_force in tool_forces_y:
                spindle_signal_results = calculate_spindle_signals(tangential_force,
                                                                   input_parameter_df['rotational_speed[Hz]'][
                                                                       exp].item() * 2 * np.pi,
                                                                   wheel_properties_df['average_radius'].item(),  # [m]
                                                                   drive_sys_config)

                spindle_current_results.append(spindle_signal_results.spindle_current)
                spindle_mechanical_power_results.append(spindle_signal_results.mechanical_power_of_the_spindle)
                spindle_electrical_power_results.append(spindle_signal_results.electrical_power_of_the_spindle)

            spindle_df = pd.DataFrame.from_dict({'Spindle electrical current - A': spindle_current_results,
                                                 'Spindle mechanical power - W': spindle_mechanical_power_results,
                                                 'Spindle electrical power - W': spindle_electrical_power_results})
            if not spindle_df.empty:
                df_collections.append(spindle_df)

            # -------------------------------Wear----------------------------
            if os.path.isfile(os.path.join(PKLs_path, 'simulation_wear_results.pkl')):
                simulation_wear_results = SimulationWearModelResults.load_from_disk(
                    os.path.join(PKLs_path, 'simulation_wear_results.pkl'))

                first_grain_attritious_wear = []
                number_grains_subject_to_attritious_wear = []
                total_attritious_wear_in_mm = []
                average_attritious_wear_in_mm = []

                for wear_model_result in simulation_wear_results.simulation_wear_model_results:
                    filtered_wear_magnitude = list(filter(lambda x: x > 0, wear_model_result.wears_magnitude))
                    number_grains_subject_to_attritious_wear.append(len(filtered_wear_magnitude))
                    total_attritious_wear_in_mm.append(sum(filtered_wear_magnitude))
                    if len(filtered_wear_magnitude) != 0:
                        average_attritious_wear_in_mm.append(
                            sum(filtered_wear_magnitude) / len(filtered_wear_magnitude))
                    else:
                        average_attritious_wear_in_mm.append(int(0))

                    first_grain_attritious_wear.append(wear_model_result.wears_magnitude[0])

                attritious_wear_df = pd.DataFrame.from_dict(
                    {"No. of grains subject to attritious wear": number_grains_subject_to_attritious_wear,
                     "Sum of magnitude of grains' attritious 3D wear vectors - mm": total_attritious_wear_in_mm,
                     "Average of magnitude of grains' attritious 3D wear vectors - mm": average_attritious_wear_in_mm,
                     'Magnitude of wear vector of grain #1 - mm': first_grain_attritious_wear})

                attritious_wear_df["Average of magnitude of grains' attritious 3D wear vectors - micrometer"] = \
                    1000 * attritious_wear_df["Average of magnitude of grains' attritious 3D wear vectors - mm"]
                attritious_wear_df["Sum of magnitude of grains' attritious 3D wear vectors - micrometer"] = \
                    1000 * attritious_wear_df["Sum of magnitude of grains' attritious 3D wear vectors - mm"]
                attritious_wear_df[
                    "Cumulative average of magnitude of grains' attritious 3D wear vectors - micrometer"] = \
                    attritious_wear_df[
                        "Average of magnitude of grains' attritious 3D wear vectors - micrometer"].cumsum()

                if not attritious_wear_df.empty:
                    df_collections.append(attritious_wear_df)

        # -------------------------------combining dataframes----------------------------
        report_df = df_collections[0]
        for df in df_collections[1:]:
            report_df = report_df.join(df)

        # adding process time into dataframe
        time_step_size = float(1 / (input_parameter_df['simulation_step_size[steps/rev]'][exp] *
                                    input_parameter_df['rotational_speed[Hz]'][exp]))
        simulation_steps = [i for i in range(report_df.shape[0])]
        report_df['Simulation steps'] = simulation_steps
        report_df['Time - mSec'] = 1000 * time_step_size * report_df['Simulation steps']  # mSec

        if extract_figures == True:
            # Check whether the specified path exists or not
            isExist = os.path.exists(input_dir + '/Exp' + str(exp) + '/images/TIFF')
            if not isExist:
                # Create a new directory because it does not exist
                os.makedirs(input_dir + '/Exp' + str(exp) + '/images/TIFF')
                print("The new image directory is created!")

            time_step_size = float(1 / (input_parameter_df['simulation_step_size[steps/rev]'][exp] *
                                        input_parameter_df['rotational_speed[Hz]'][exp]))

            save_all_labels_in_TIFF_format(report_df,
                                           input_dir + '/Exp' + str(exp) + '/images/TIFF',
                                           time_step_size)

        # -------------------------------exporting to excel----------------------------
        with ExcelWriter(input_dir + '/Exp' + str(exp) + '/Report.xlsx') as writer:
            input_parameter_df.iloc[exp].to_excel(writer, sheet_name='input_paramteres')
            report_df.to_excel(writer, sheet_name='results')
        report_df.to_csv(input_dir + '/Exp' + str(exp) + '/Report.csv')

        # -------------------------------visualize dataframe----------------------------
        configuration.do_plot = visualization
        present(result_df=report_df,
                path=str(input_dir + '/Exp' + str(exp) + '/PKLFiles'),
                config=visualization_config)

        if bool(input_parameter_df['txt_log_file'][exp]):
            terminal_log_file.close()


if __name__ == "__main__":
    input_dir = r'./RailGrinding/MultipleRunSimulations/Results'
    number_of_available_experiment_folders = len(next(os.walk(input_dir))[1])
    input_parameter_df = pd.read_csv(r"./RailGrinding/MultipleRunSimulations/input_parameters.csv")

    visualization_config = {'visualize_workpiece': False,
                            'visualize_forces': False,
                            'visualize_process': False,
                            'visualize_forces_interactive': False,
                            'visualize_topography': False}
    result_extraction(input_parameter_df, input_dir, visualization_config, visualization=False, extract_figures=False)
