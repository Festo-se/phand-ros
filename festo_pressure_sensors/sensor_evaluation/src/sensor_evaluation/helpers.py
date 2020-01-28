import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def get_sample(file, folder):
    sample_name = file.replace(r'.csv', '')
    df = pd.read_csv(folder + file)[['A1', 'A2', 'A3', 'B1', 'B2', 'B3', 'C1', 'C2', 'C3']]

    return df, sample_name
