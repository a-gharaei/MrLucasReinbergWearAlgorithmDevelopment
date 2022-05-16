"""This scripts relates to the simulation of abrasive rail grinding"""
import importlib
import os
import shutil

import pandas as pd



def execute():
    working_dir = './RailGrinding/MultipleRunSimulations'
    script_name = 'main_script'
    output_path = working_dir+ '/Results'

    module = importlib.import_module(
        r'RailGrinding.MultipleRunSimulations.' + script_name)

    # Read input parameters from file
    if os.path.isfile( working_dir + '/input_all_parameters.csv'):
        df = pd.read_csv(working_dir + '/input_all_parameters.csv')
    elif os.path.isfile(working_dir+ '/input_parameters.csv'):
        df = pd.read_csv(working_dir+ '/input_parameters.csv')
    else:
        raise Exception('No valid input file is found.')

    # check_input_parameters_and_configurations(df)
    if os.path.exists(output_path):
        shutil.rmtree(output_path)

    module.process_simulation(df, output_path)


if __name__ == "__main__":
    execute()
