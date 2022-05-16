import os
from logging import exception
import os
from collections import defaultdict
from typing import Dict, List

from pandas import ExcelWriter
import pandas as pd
import numpy as np


def save_to_disk_as_pkl(dict: Dict, path: str):
    # save to disk
    for key, value in dict.items():
        value.save_to_disk(os.path.join(path, key + '.pkl'))


def print_out_to_console(items: List):
    for item in items:
        print(item)


def export_to_excel(data, path: str, expected_length: int):
    # Check for identical length of each dictionary value
    for key in data:
        if len(data[key]) != expected_length:
            raise exception("length of " + key + " was not as should be expected!")

    df = pd.DataFrame(data)
    # Get the data
    with ExcelWriter(path) as writer:
        df.to_excel(writer, sheet_name='data')
