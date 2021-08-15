from matplotlib.gridspec import GridSpec
from matplotlib import pyplot as plt
import numpy as np
import torch


def plot_voltage_traces(mem, spk=None, dim=(3, 5), spike_height=5):
    gs = GridSpec(*dim)
    if spk is not None:
        dat = 1.0*mem
        dat[spk > 0.0] = spike_height
        dat = dat.detach().cpu().numpy()
    else:
        dat = mem.detach().cpu().numpy()
    for i in range(np.prod(dim)):
        if i == 0:
            a0 = ax = plt.subplot(gs[i])
        else:
            ax = plt.subplot(gs[i], sharey=a0)
        ax.plot(dat[i])
        ax.axis("off")


def pre_process_data(device, orig_x_data):
    dtype = torch.float
    device = torch.device("cpu")

    x_data = orig_x_data.reshape(15, 10, 10).tolist()

    # # Generate labels
    labels = np.concatenate([[0] * 5, [1] * 5, [2] * 5])

    x_data = torch.tensor(x_data, device=device, dtype=dtype, requires_grad=False)
    y_data = torch.tensor(labels, device=device, dtype=torch.long)
    return x_data, y_data

