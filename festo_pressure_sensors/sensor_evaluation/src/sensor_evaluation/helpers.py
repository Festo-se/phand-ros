import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

def get_sample(file, folder, matrix_count=1):
    sample_name = file.replace(r'.csv', '')
    if matrix_count == 1:
    	df = pd.read_csv(folder + file)[['A1', 'A2', 'A3', 'B1', 'B2', 'B3', 'C1', 'C2', 'C3']]
    elif matrix_count == 2:
    	df = pd.read_csv(folder + file)[['A1','A2','A3','B1','B2','B3','C1','C2','C3',
    									'2-A1','2-A2','2-A3','2-B1','2-B2','2-B3','2-C1','2-C2','2-C3',]]


    return df, sample_name
    
