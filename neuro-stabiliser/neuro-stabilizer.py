# import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.gridspec import GridSpec
# import seaborn as sns
import torch
import torch.nn as nn
import pandas as pd
from tsfresh import extract_relevant_features, extract_features
from tsfresh.feature_selection.selection import select_features
from tsfresh.utilities.dataframe_functions import impute

import SurrGradSpike


nb_inputs  = 10
nb_hidden  = 4
nb_outputs = 2
time_step = 1e-3
nb_steps  = 150
batch_size = 50
dtype = torch.float
device = torch.device("cpu")

# Uncomment the line below to run on GPU
# device = torch.device("cuda:0")

## DATASET

# Prepare the data into temporal batches
orig_x_data = np.array([
    [0.00, 21.86, 0.00, 38.37, 54.41, 0.00, 0.00, 64.40, 15.23, 14.96, 15.93, 0.00, 0.00, 21.90, 0.00, 0.00, 58.73, 54.29, 38.76, 72.93, 83.48, 0.00, 36.81, 0.00, 0.00, 28.10, 0.00, 30.07, 0.00, 16.79, 63.25, 18.23, 98.63, 0.00, 0.00, 0.00, 50.75, 46.56, 14.35, 26.29, 53.62, 18.71, 0.00, 70.33, 0.00, 40.76, 20.66, 72.59, 0.00, 36.36, 16.82, 58.54, 55.73, 41.73, 62.95, 19.92, 61.53, 0.00, 56.70, 70.73, 0.00, 17.50, 23.56, 122.56, 132.25, 29.21, 24.36, 19.22, 14.25, 0.00, 48.98, 18.12, 32.90, 16.76, 0.00, 46.00, 26.48, 16.03, 19.04, 101.51, 38.34, 55.23, 26.35, 72.88, 33.49, 0.00, 22.59, 36.07, 0.00, 27.95, 0.00, 17.75, 23.65, 0.00, 0.00, 0.00, 31.35, 18.05, 0.00, 23.25, 0.00, 26.63, 15.74, 0.00, 92.10, 29.42, 48.98, 53.56, 0.00, 0.00, 16.23, 24.74, 66.92, 0.00, 58.98, 90.44, 30.33, 92.37, 23.19, 27.23, 35.49, 0.00, 76.02, 38.36, 27.59, 24.12, 0.00, 20.78, 0.00, 20.69, 0.00, 43.53, 0.00, 69.57, 42.76, 36.75, 48.72, 34.89, 0.00, 0.00, 18.87, 0.00, 18.81, 64.46, 92.58, 67.79, 70.54, 27.33, 20.46, 46.61, 100.23, 0.00, 49.60, 0.00, 0.00, 13.77, 132.62, 0.00, 0.00, 44.80, 48.19, 0.00, 87.06, 0.00, 48.33, 0.00, 27.50, 0.00, 18.49, 34.31, 66.36, 0.00, 0.00, 24.13, 0.00, 23.14, 62.68, 0.00, 22.83, 66.99, 0.00, 38.01, 15.16, 81.78, 40.93, 33.58, 0.00, 0.00, 0.00, 52.17, 26.08, 0.00, 17.73, 120.07, 0.00, 0.00, 0.00, 64.12, 17.26, 107.73, 40.37, 0.00, 44.46, 25.98, 0.00, 95.29, 62.24, 102.99, 16.93, 41.45, 23.95, 0.00, 24.14, 0.00, 0.00, 0.00, 37.52, 32.39, 0.00, 42.59, 0.00, 78.45, 39.24, 0.00, 30.21, 37.57, 86.00, 76.27, 0.00, 38.06, 0.00, 57.65, 28.88, 64.44, 30.96, 0.00, 99.31, 0.00, 0.00, 14.36, 0.00, 0.00, 30.45, 20.00, 38.85, 35.28, 49.61, 20.79, 0.00, 0.00, 0.00, 207.58, 0.00, 54.07, 26.19, 20.30, 0.00, 0.00, 47.41, 38.28, 44.65, 29.68, 33.62, 80.53, 0.00, 0.00, 71.43, 34.87, 41.32, 31.82, 34.78, 0.00, 38.41, 0.00, 54.25, 27.93, 49.09, 27.64, 16.20, 0.00, 54.46, 24.17, 35.58, 0.00, 62.93, 53.49, 0.00, 68.08, 17.20, 19.05, 0.00, 0.00, 30.69, 22.34, 19.56, 18.80, 0.00, 20.92, 23.85, 107.88, 39.42, 30.65, 18.69, 28.98, 49.91, 39.61, 28.59, 0.00, 105.21, 35.76, 52.24, 30.82, 0.00, 0.00, 33.33, 58.47, 31.92, 18.02, 0.00, 26.05, 14.54, 20.89, 16.89, 14.66, 0.00, 0.00, 53.59, 0.00, 0.00, 46.68, 0.00, 0.00, 25.65, 31.19, 68.23, 85.34, 21.84, 36.65, 58.88, 36.26, 23.07, 91.22, 0.00, 116.44, 0.00, 0.00, 64.87, 34.56, 0.00, 0.00, 0.00, 33.99, 0.00, 17.27, 0.00, 0.00, 0.00, 0.00, 0.00, 16.25, 95.86, 84.25, 34.14, 41.28, 46.02, 70.90, 0.00, 83.38, 0.00, 0.00, 46.75, 121.50, 0.00, 61.81, 0.00, 19.44, 62.90, 17.61, 0.00, 18.60, 22.31, 15.06, 26.86, 37.23, 0.00, 0.00, 0.00, 15.13, 49.81, 45.97, 49.43, 23.85, 18.98, 68.97, 0.00, 30.90, 18.42, 0.00, 19.81, 22.39, 35.49, 0.00, 0.00, 0.00, 85.74, 109.16, 57.83, 0.00, 16.35, 77.04, 15.23, 0.00, 0.00, 33.59, 0.00, 0.00, 49.03, 32.52, 62.68, 0.00, 47.52, 76.25, 0.00, 25.89, 0.00, 30.23, 38.26, 37.99, 0.00, 0.00, 23.48, 34.47, 20.27, 0.00, 0.00, 16.90, 39.83, 25.86, 25.78, 103.68, 24.90, 0.00, 30.24, 33.16, 0.00, 42.14, 37.25, 46.44, 69.01, 22.54, 39.99, 19.19, 27.85, 26.08, 0.00, 57.20, 22.19, 17.62, 27.30, 27.34, 32.65, 0.00, 28.20, 19.94, 0.00, 19.02, 27.73, 86.81, 61.72, 27.32, 0.00, 0.00, 13.78, 56.69, 13.97, 49.22, 50.52, 52.03, 0.00, 0.00, 62.65, 0.00, 32.65, 0.00, 52.35, 55.73, 63.12, 0.00, 32.15, 30.70, 0.00, 14.35, 0.00, 0.00, 0.00, 71.99, 32.41, 33.85, 0.00, 71.68],
    [0.00, 220.28, 260.86, 298.06, 248.48, 213.45, 257.19, 243.87, 206.55, 220.44, 238.68, 201.52, 260.96, 225.18, 291.68, 255.57, 259.17, 235.14, 218.34, 193.53, 228.17, 264.59, 286.83, 226.62, 234.07, 245.22, 308.74, 178.01, 219.45, 211.40, 247.14, 186.93, 291.87, 259.67, 193.91, 251.30, 328.18, 225.66, 230.60, 216.16, 219.77, 232.87, 235.03, 202.87, 281.17, 280.52, 238.57, 211.09, 270.64, 230.42, 250.39, 212.13, 273.83, 292.81, 247.89, 254.07, 216.52, 239.05, 274.76, 247.73, 229.79, 261.36, 222.09, 214.87, 241.25, 205.88, 343.79, 261.33, 216.91, 253.62, 287.59, 216.56, 250.02, 247.59, 250.61, 303.53, 215.57, 235.09, 246.91, 196.46, 230.88, 243.03, 252.67, 297.98, 299.24, 239.78, 328.06, 277.01, 226.54, 189.69, 249.89, 263.85, 254.77, 226.01, 247.60, 238.06, 254.67, 202.59, 225.50, 230.52, 257.72, 260.96, 223.33, 216.20, 217.44, 206.29, 232.68, 257.80, 226.07, 225.18, 236.79, 250.92, 258.96, 221.85, 244.80, 224.19, 236.92, 221.34, 183.90, 239.23, 228.49, 196.49, 252.59, 267.84, 222.51, 250.52, 265.66, 244.36, 205.38, 278.49, 252.17, 284.71, 221.03, 242.68, 273.14, 245.33, 255.77, 217.57, 261.20, 217.89, 222.96, 237.13, 236.00, 220.41, 238.29, 259.37, 236.79, 242.08, 270.95, 225.55, 231.42, 255.90, 261.62, 231.11, 305.58, 215.41, 285.95, 271.16, 207.49, 311.39, 239.68, 224.24, 263.88, 236.81, 253.82, 232.75, 255.55, 288.17, 221.73, 244.84, 299.64, 297.36, 329.54, 261.94, 247.25, 246.09, 230.20, 246.80, 232.63, 330.61, 219.79, 186.13, 202.68, 293.24, 280.80, 210.77, 217.26, 218.06, 253.03, 272.61, 258.56, 230.92, 225.54, 254.31, 205.38, 228.15, 262.67, 252.49, 254.51, 259.76, 206.15, 268.89, 293.79, 249.24, 261.67, 322.44, 228.65, 238.27, 245.36, 281.27, 218.52, 259.54, 206.74, 234.98, 228.09, 230.08, 206.06, 245.12, 252.12, 282.17, 247.14, 259.94, 235.68, 188.97, 274.70, 294.43, 219.20, 259.92, 228.59, 228.61, 280.39, 252.11, 265.02, 236.68, 233.70, 214.99, 251.08, 298.84, 209.54, 196.93, 272.71, 236.73, 207.11, 231.16, 266.35, 240.89, 243.33, 231.22, 326.77, 203.00, 202.08, 247.15, 227.61, 306.58, 227.57, 145.22, 268.91, 311.75, 219.50, 231.22, 231.89, 240.50, 267.27, 232.26, 232.63, 239.01, 216.93, 264.39, 236.18, 217.60, 224.07, 285.57, 228.22, 189.54, 233.19, 219.04, 253.73, 237.31, 246.21, 208.85, 264.88, 258.40, 262.67, 276.94, 305.68, 219.50, 241.08, 260.34, 234.78, 194.36, 239.36, 264.30, 229.58, 282.56, 212.85, 254.42, 188.78, 217.07, 226.21, 255.63, 215.45, 254.29, 217.50, 281.60, 282.41, 232.29, 255.03, 218.33, 270.42, 259.16, 229.91, 226.73, 220.64, 228.58, 272.65, 237.64, 269.84, 242.70, 212.66, 239.00, 269.08, 225.57, 301.53, 218.17, 252.04, 248.04, 304.02, 271.36, 316.39, 207.19, 276.85, 229.11, 238.64, 256.82, 247.20, 256.10, 227.89, 309.84, 249.95, 236.32, 221.58, 285.41, 233.04, 215.78, 235.20, 245.61, 215.83, 199.95, 210.17, 242.37, 222.64, 217.06, 318.13, 251.29, 253.77, 220.23, 195.65, 169.88, 270.48, 251.17, 236.98, 210.91, 215.94, 256.01, 255.51, 282.22, 247.71, 213.61, 240.51, 249.89, 218.87, 206.98, 215.29, 186.40, 234.49, 211.27, 263.02, 243.74, 248.44, 193.41, 263.41, 303.30, 204.30, 238.18, 296.50, 219.13, 238.90, 210.19, 257.73, 198.97, 264.82, 247.89, 241.01, 254.58, 247.56, 236.30, 228.11, 240.83, 286.86, 268.44, 237.72, 243.52, 219.42, 153.70, 253.96, 275.94, 210.91, 263.65, 195.70, 219.17, 222.84, 234.30, 236.55, 287.88, 199.06, 241.37, 313.80, 203.16, 284.09, 308.00, 201.31, 228.75, 354.12, 241.22, 239.14, 249.89, 260.98, 249.34, 248.59, 249.52, 226.21, 174.22, 236.91, 191.33, 233.09, 202.23, 284.93, 213.91, 258.55, 230.98, 236.23, 232.32, 232.20, 233.06, 237.76, 221.92, 246.33, 261.71, 228.19, 241.10, 259.70, 244.65, 272.20, 248.09, 229.90, 223.84, 198.54, 203.37, 224.36, 181.23, 218.78, 264.77, 255.99, 244.36, 316.67, 288.01, 187.29, 252.07, 219.56, 202.63, 288.13, 267.02, 193.81, 227.65, 226.32, 198.65, 236.58, 230.49, 266.98, 241.00, 202.58, 229.73, 213.39, 216.78, 263.56, 258.25, 242.40, 205.42, 247.65, 218.36, 255.86, 244.31, 229.20, 238.05, 268.61, 271.33, 287.63, 235.62, 244.68, 225.98], 
    [0.00, 106.27, 108.28, 103.03, 115.97, 108.60, 106.32, 108.41, 105.77, 105.78, 105.73, 111.46, 104.82, 105.18, 110.09, 105.42, 111.87, 106.07, 108.22, 108.10, 111.63, 106.47, 110.79, 110.62, 107.68, 110.15, 112.89, 109.65, 105.06, 108.37, 108.18, 104.21, 105.68, 104.96, 109.54, 110.80, 110.75, 106.94, 106.07, 106.29, 107.14, 108.70, 105.82, 106.32, 108.92, 107.31, 107.13, 104.47, 113.20, 107.42, 109.43, 109.38, 107.86, 111.11, 107.73, 103.70, 104.59, 105.10, 111.17, 109.03, 108.87, 108.37, 107.74, 106.57, 105.25, 110.30, 108.27, 109.71, 109.74, 104.51, 109.39, 109.37, 106.16, 109.81, 105.44, 110.60, 110.20, 106.25, 106.75, 106.30, 105.70, 104.58, 104.98, 108.64, 110.10, 106.59, 105.26, 109.15, 107.79, 107.30, 104.66, 112.12, 108.42, 110.17, 105.86, 105.19, 106.80, 105.37, 109.23, 107.42, 106.24, 106.80, 102.27, 105.09, 109.23, 111.49, 104.23, 104.71, 108.85, 107.54, 107.55, 109.27, 110.62, 107.74, 109.15, 109.29, 114.25, 105.39, 110.96, 107.35, 105.11, 104.29, 106.96, 105.30, 106.58, 111.61, 108.30, 108.46, 109.19, 114.07, 108.26, 108.76, 106.23, 109.37, 106.49, 110.30, 115.34, 109.13, 104.25, 105.86, 105.23, 108.20, 107.71, 104.55, 106.07, 107.85, 109.11, 106.83, 107.13, 106.11, 109.22, 108.27, 112.83, 105.96, 109.38, 112.06, 109.06, 105.97, 109.84, 105.95, 102.21, 105.08, 108.93, 108.63, 109.57, 108.18, 108.98, 107.70, 112.75, 111.47, 105.95, 106.87, 108.43, 109.58, 108.65, 107.24, 106.21, 104.60, 109.88, 104.99, 113.63, 108.60, 104.74, 111.18, 108.51, 108.50, 112.86, 107.62, 106.02, 109.67, 110.83, 109.61, 104.58, 105.34, 106.20, 109.30, 110.21, 104.10, 108.30, 109.82, 111.20, 108.27, 110.22, 107.20, 109.21, 103.68, 105.55, 108.83, 110.54, 107.02, 110.11, 106.98, 104.11, 107.27, 105.20, 112.43, 105.99, 113.58, 108.15, 110.77, 104.53, 109.16, 109.31, 111.99, 104.90, 104.53, 106.49, 104.74, 110.05, 107.54, 104.53, 103.70, 110.70, 113.76, 106.74, 108.31, 109.18, 105.21, 107.15, 108.39, 108.44, 105.84, 115.72, 108.15, 106.34, 105.62, 108.52, 109.99, 106.46, 109.87, 104.80, 107.39, 107.47, 104.23, 106.54, 114.31, 111.75, 109.66, 107.22, 106.44, 113.57, 113.76, 111.95, 106.62, 105.43, 103.92, 108.22, 111.31, 112.01, 105.39, 107.52, 108.62, 103.00, 104.91, 106.64, 117.43, 103.24, 106.51, 104.84, 108.65, 108.51, 103.53, 107.15, 107.92, 105.49, 110.64, 105.44, 104.82, 104.66, 107.67, 112.44, 105.27, 107.54, 104.71, 115.21, 103.24, 102.28, 117.58, 108.98, 109.06, 106.32, 106.58, 105.46, 112.22, 108.52, 108.45, 112.54, 114.37, 107.34, 107.49, 107.35, 110.71, 103.86, 111.33, 108.78, 102.72, 106.15, 110.39, 110.21, 107.32, 104.76, 106.56, 110.04, 112.44, 106.25, 107.34, 112.53, 109.16, 111.06, 103.87, 107.06, 111.36, 110.69, 103.93, 106.44, 106.83, 104.49, 110.40, 103.60, 104.90, 106.04, 107.73, 110.51, 106.69, 110.36, 103.46, 108.31, 110.67, 103.46, 110.34, 106.04, 108.02, 104.46, 107.65, 105.59, 105.75, 106.92, 106.73, 106.84, 105.62, 105.77, 111.78, 107.62, 103.93, 103.53, 104.94, 111.07, 106.29, 105.30, 107.37, 102.36, 105.40, 110.96, 107.69, 102.39, 107.69, 105.18, 105.93, 109.32, 109.43, 105.61, 103.04, 107.64, 104.36, 108.21, 111.28, 105.17, 109.21, 112.62, 107.78, 109.61, 107.68, 112.96, 110.10, 105.85, 106.77, 109.49, 108.40, 105.44, 106.12, 109.76, 108.61, 110.14, 104.93, 106.49, 104.86, 104.26, 107.07, 109.45, 104.58, 106.69, 109.69, 108.00, 104.37, 106.28, 105.65, 109.69, 107.23, 107.01, 106.09, 106.47, 108.50, 106.48, 106.15, 105.77, 106.63, 106.86, 109.71, 112.20, 107.67, 106.76, 111.08, 108.95, 109.45, 106.55, 107.31, 107.98, 107.03, 107.82, 105.93, 114.11, 105.38, 108.17, 106.53, 108.48, 109.22, 107.55, 107.57, 104.94, 105.78, 113.48, 107.05, 104.73, 109.36, 109.27, 110.88, 106.59, 106.94, 107.05, 110.07, 106.53, 113.36, 103.02, 108.84, 113.38, 102.98, 108.60, 104.59, 105.19, 105.85, 112.37, 111.59, 105.45, 108.00, 107.32, 106.51, 105.89, 109.23, 110.72, 108.35, 109.72, 110.12, 105.26, 106.50, 105.91, 111.47, 109.56, 106.78, 106.53, 108.89, 106.67, 102.21, 105.48, 106.98, 108.39, 104.70, 107.45, 108.59, 106.72, 109.78]
])

time_series_size = 10
orginal_recording_size = 500
number_of_batches = int(orginal_recording_size / time_series_size)

def pre_process_data(device, orig_x_data, time_series_size, number_of_batches):
    line_1 = np.split(orig_x_data[0], number_of_batches)
    line_2 = np.split(orig_x_data[1], number_of_batches)
    line_3 = np.split(orig_x_data[2], number_of_batches)

    x_data = torch.tensor(np.concatenate([line_1, line_2, line_3]).tolist(), device = device)

    # Generate labels
    b = []
    indexes = []
    for i in range(number_of_batches):
        b.append(0)
        indexes.append(i * 0.8 * time_series_size)

    for i in range(number_of_batches):
        b.append(1)
        indexes.append(i * 0.8 * time_series_size)

    for i in range(number_of_batches):
        b.append(2)
        indexes.append(i * 0.8 * time_series_size)

    y_data = torch.tensor(b, device = device)
    return x_data,y_data

x_data, y_data = pre_process_data(device, orig_x_data, time_series_size, number_of_batches)

# SPIKING MODEL SETUP

tau_mem = 10e-3
tau_syn = 5e-3
alpha   = float(np.exp(-time_step/tau_syn))
beta    = float(np.exp(-time_step/tau_mem))
weight_scale = 7*(1.0-beta) # this should give us some spikes to begin with

w1 = torch.empty((nb_inputs, nb_hidden),  device=device, dtype=dtype, requires_grad=True)
torch.nn.init.normal_(w1, mean=0.0, std=weight_scale/np.sqrt(nb_inputs))

w2 = torch.empty((nb_hidden, nb_outputs), device=device, dtype=dtype, requires_grad=True)
torch.nn.init.normal_(w2, mean=0.0, std=weight_scale/np.sqrt(nb_hidden))

print("init done")

def run_snn(inputs):
    h1 = torch.einsum("ab,bc->ac", (inputs, w1))
    syn = torch.zeros((batch_size, nb_hidden), device=device, dtype=dtype)
    mem = torch.zeros((batch_size, nb_hidden), device=device, dtype=dtype)

    mem_rec = []
    spk_rec = []

    # Compute hidden layer activity
    for t in range(nb_steps):
        mthr = mem-1.0
        out = spike_fn(mthr)
        rst = out.detach() # We do not want to backprop through the reset

        new_syn = alpha * syn + h1[:,t]
        new_mem = (beta * mem + syn) * (1.0 - rst)

        mem_rec.append(mem)
        spk_rec.append(out)
        
        mem = new_mem
        syn = new_syn

    mem_rec = torch.stack(mem_rec,dim=1)
    spk_rec = torch.stack(spk_rec,dim=1)

    # Readout layer
    h2= torch.einsum("abc,cd->abd", (spk_rec, w2))
    flt = torch.zeros((batch_size,nb_outputs), device=device, dtype=dtype)
    out = torch.zeros((batch_size,nb_outputs), device=device, dtype=dtype)
    out_rec = [out]
    for t in range(nb_steps):
        new_flt = alpha*flt +h2[:,t]
        new_out = beta*out +flt

        flt = new_flt
        out = new_out

        out_rec.append(out)

    out_rec = torch.stack(out_rec,dim=1)
    other_recs = [mem_rec, spk_rec]
    return out_rec, other_recs
    
# here we overwrite our naive spike function by the "SurrGradSpike" 
# nonlinearity which implements a surrogate gradient
spike_fn  = SurrGradSpike.SurrGradSpike.apply


def print_classification_accuracy():
    """ Dirty little helper function to compute classification accuracy. """
    output,_ = run_snn(x_data)
    m,_= torch.max(output,1) # max over time
    _,am=torch.max(m,1) # argmax over output units
    acc = np.mean((y_data==am).detach().cpu().numpy()) # compare to labels
    print("Accuracy %.3f"%acc)
    
print_classification_accuracy()


# The following lines will reinitialize the weights
torch.nn.init.normal_(w1, mean=0.0, std=weight_scale/np.sqrt(nb_inputs))
torch.nn.init.normal_(w2, mean=0.0, std=weight_scale/np.sqrt(nb_hidden))
print("init done")

params = [w1,w2]
optimizer = torch.optim.Adam(params, lr=2e-3, betas=(0.9,0.999))

log_softmax_fn = nn.LogSoftmax(dim=1)
loss_fn = nn.NLLLoss()

loss_hist = []
for e in range(1000):
    output,_ = run_snn(x_data)
    m,_=torch.max(output,1)
    log_p_y = log_softmax_fn(m)
    loss_val = loss_fn(log_p_y, y_data)

    optimizer.zero_grad()
    loss_val.backward()
    optimizer.step()
    loss_hist.append(loss_val.item())

# plt.figure(figsize=(3.3,2),dpi=150)
# plt.plot(loss_hist, label="Surrogate gradient")
# plt.xlabel("Epoch")
# plt.ylabel("Loss")
# plt.legend()
# sns.despine()


output,_ = run_snn(x_data)
m,_=torch.max(output,1)

# Compute training accuracy
_,am=torch.max(m,1)
acc = np.mean((y_data==am).detach().cpu().numpy())
print("Accuracy %f"%acc)



# out_rec,other_recs = run_snn(x_data)

# fig=plt.figure(dpi=100)
# plot_voltage_traces(out_rec)