"""This scripts relates to the simulation of abrasive rail grinding"""
import importlib
import os
import shutil

import pandas as pd


def execute():  # sourcery skip: raise-specific-error
    working_dir = "MultipleRunSimulations"
    script_name = "main_script"
    output_path = f"{working_dir}/Results"

    module = importlib.import_module(f"{script_name}")

    # Read input parameters from file
    if os.path.isfile(f"{working_dir}/input_all_parameters.csv"):
        df = pd.read_csv(f"{working_dir}/input_all_parameters.csv")
    elif os.path.isfile(f"{working_dir}/input_parameters.csv"):
        df = pd.read_csv(f"{working_dir}/input_parameters.csv")
    else:
        raise Exception("No valid input file is found.")

    # check_input_parameters_and_configurations(df)
    if os.path.exists(output_path):
        shutil.rmtree(output_path)

    module.process_simulation(df, output_path)


if __name__ == "__main__":
    execute()
