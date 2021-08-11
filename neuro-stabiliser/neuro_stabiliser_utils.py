from matplotlib.gridspec import GridSpec
from matplotlib import pyplot as plt
import numpy as np
import torch

def plot_voltage_traces(mem, spk=None, dim=(3,5), spike_height=5):
    gs=GridSpec(*dim)
    if spk is not None:
        dat = 1.0*mem
        dat[spk>0.0] = spike_height
        dat = dat.detach().cpu().numpy()
    else:
        dat = mem.detach().cpu().numpy()
    for i in range(np.prod(dim)):
        if i==0: a0=ax=plt.subplot(gs[i])
        else: ax=plt.subplot(gs[i],sharey=a0)
        ax.plot(dat[i])
        ax.axis("off")

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