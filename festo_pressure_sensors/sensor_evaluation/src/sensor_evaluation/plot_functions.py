import os
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Simple plotting functions:

def time_plot(df, sample_name):
    # Initialise figure
    fig, ax = plt.subplots(figsize=(16, 4))
    ax.plot(df)

    # Define colour space for lines
    colourmap = plt.cm.gist_ncar
    plt.gca().set_prop_cycle(plt.cycler('color', plt.cm.jet(np.linspace(0, 0.9, 9))))

    # Set up layout
    ax.set(xlabel='time steps',
           ylabel='signal (analog counts)',
           title='data sample: ' + sample_name)
    labels = ['A1', 'A2', 'A3', 'B1', 'B2', 'B3', 'C1', 'C2', 'C3']
    ax.legend(labels)
    ax.grid()

    #     fig.savefig("data/figures/"+sample_name+"_timeplot.png")
    plt.show()

    return

def time_plot2(df, sample_name,labels):
    # Initialise figure
    fig, ax = plt.subplots(figsize=(16, 4))
    ax.plot(df)

    # Define colour space for lines
    colourmap = plt.cm.gist_ncar
    plt.gca().set_prop_cycle(plt.cycler('color', plt.cm.jet(np.linspace(0, 0.9, 9))))

    # Set up layout
    ax.set(xlabel='time steps',
           ylabel='signal (analog counts)',
           title='data sample: ' + sample_name)
    ax.legend(labels)
    ax.grid()

    #     fig.savefig("data/figures/"+sample_name+"_timeplot.png")
    plt.show()

    return


def max_plot(data, sample_name):
    fig, ax = plt.subplots(figsize=(16, 4))
    labels = ['A1', 'A2', 'A3', 'B1', 'B2', 'B3', 'C1', 'C2', 'C3']
    data = data[:].max()

    plot = ax.bar(labels, data)

    # Set up layout
    ax.set(xlabel='sensor cell',
           ylabel='signal range (analog counts)',
           title='data sample: ' + sample_name)
    ax.grid()

    plt.show()
    return


def box_plot(data, sample_name, uplim=None, *kwargs):

    fig, ax = plt.subplots(figsize=(8,4))
    labels = ['A1','A2','A3','B1','B2','B3','C1','C2','C3']

    plot = ax.boxplot(data.T, *kwargs)

    # Set up layout
    ax.set(xlabel='sensor cell',
           ylabel='signal range (analog counts)',
           title='data sample: '+sample_name,
           xticklabels=labels)
    ax.set_ybound(lower=0, upper=uplim)
    ax.grid()

    plt.show()
    return


def max_map(data, sample_name):
    rows = ['A', 'B', 'C']
    columns = ['1', '2', '3']

    max_data = data[:].max().to_numpy().reshape((3, 3))

    # Plot the heatmap
    fig, ax = plt.subplots()
    im = ax.imshow(max_data, cmap="YlOrRd", vmin=0, vmax=1000)  # cmap="Wistia"

    # Create colorbar
    cbar_kw = {}
    cbarlabel = "Sensor max values (analog counts)"
    cbar = ax.figure.colorbar(im, ax=ax, **cbar_kw)
    cbar.ax.set_ylabel(cbarlabel, rotation=-90, va="bottom")

    # We want to show all ticks...
    ax.set_xticks(np.arange(len(columns)))
    ax.set_yticks(np.arange(len(rows)))
    # ... and label them with the respective list entries
    ax.set_xticklabels(columns)
    ax.set_yticklabels(rows)

    # Loop over data dimensions and create text annotations.
    for i in range(len(rows)):
        for j in range(len(columns)):
            text = ax.text(j, i, max_data[i, j],
                           ha="center", va="center", color="black")

    ax.set_title('data sample: ' + sample_name)
    fig.tight_layout()
    plt.show()

    return


def avg_map(data, sample_name):
    rows = ['A', 'B', 'C']
    columns = ['1', '2', '3']

    max_data = data[:].mean(axis=0).astype(int).to_numpy().reshape((3, 3))

    # Plot the heatmap
    fig, ax = plt.subplots()
    im = ax.imshow(max_data, cmap="YlOrRd", vmin=0, vmax=1000)  # cmap="Wistia"

    # Create colorbar
    cbar_kw = {}
    cbarlabel = "Sensor max values (analog counts)"
    cbar = ax.figure.colorbar(im, ax=ax, **cbar_kw)
    cbar.ax.set_ylabel(cbarlabel, rotation=-90, va="bottom")

    # We want to show all ticks...
    ax.set_xticks(np.arange(len(columns)))
    ax.set_yticks(np.arange(len(rows)))
    # ... and label them with the respective list entries
    ax.set_xticklabels(columns)
    ax.set_yticklabels(rows)

    # Loop over data dimensions and create text annotations.
    for i in range(len(rows)):
        for j in range(len(columns)):
            text = ax.text(j, i, max_data[i, j],
                           ha="center", va="center", color="black")

    ax.set_title('data sample: ' + sample_name)
    fig.tight_layout()
    plt.show()

    return

# Loop plotting functions:

def box_plot2(fig, axs, data, sample_name, n, uplim=None, *kwargs):
    labels = ['A1', 'A2', 'A3', 'B1', 'B2', 'B3', 'C1', 'C2', 'C3']
    x = int(n / 2)
    y = n % 2

    plot = axs[x, y].boxplot(data.T, *kwargs)
    #     plot = axs[x,y].boxplot(data.T, 0, '')     # don't show outliers

    # Set up layout
    axs[x, y].set(xlabel='sensor cell',
                  ylabel='signal range (analog counts)',
                  title='data sample: ' + sample_name,
                  xticklabels=labels)
    axs[x, y].set_ybound(lower=0, upper=uplim)
    axs[x, y].grid()

    fig.tight_layout()
    fig.set_figheight(12)
    fig.set_figwidth(16)

    return


def avg_map2(fig, axs, data, sample_name, n, uplim=1000):
    rows = ['A', 'B', 'C']
    columns = ['1', '2', '3']

    x = int(n / 3)
    y = n % 3

    avg_data = data[:].mean(axis=0).astype(int).to_numpy().reshape((3, 3))

    # Plot the heatmap
    im = axs[x, y].imshow(avg_data, cmap="YlOrRd", vmin=0, vmax=uplim)  # cmap="Wistia"

    # We want to show all ticks...
    axs[x, y].set_xticks(np.arange(len(columns)))
    axs[x, y].set_yticks(np.arange(len(rows)))
    # ... and label them with the respective list entries
    axs[x, y].set_xticklabels(columns)
    axs[x, y].set_yticklabels(rows)

    # Loop over data dimensions and create text annotations.
    for i in range(len(rows)):
        for j in range(len(columns)):
            text = axs[x, y].text(j, i, avg_data[i, j],
                                  ha="center", va="center", color="black")

    axs[x, y].set_title('data sample: ' + sample_name)
    fig.tight_layout()
    fig.set_figheight(10)
    fig.set_figwidth(10)

    return fig, axs


def max_map2(fig, axs, data, sample_name, n):
    rows = ['A', 'B', 'C']
    columns = ['1', '2', '3']

    x = int(n / 3)
    y = n % 3

    avg_data = data[:].max().to_numpy().reshape((3, 3))

    # Plot the heatmap
    im = axs[x, y].imshow(avg_data, cmap="YlOrRd", vmin=0, vmax=1000)  # cmap="Wistia"

    # We want to show all ticks...
    axs[x, y].set_xticks(np.arange(len(columns)))
    axs[x, y].set_yticks(np.arange(len(rows)))
    # ... and label them with the respective list entries
    axs[x, y].set_xticklabels(columns)
    axs[x, y].set_yticklabels(rows)

    # Create colorbar
    cbar_kw = {"fraction": 0.046, "pad": 0.04}
    cbarlabel = "Sensor max values (analog counts)"
    cbar = axs[x, y].figure.colorbar(im, ax=axs[x, y], **cbar_kw)
    cbar.ax.set_ylabel(cbarlabel, rotation=-90, va="bottom")

    # Loop over data dimensions and create text annotations.
    for i in range(len(rows)):
        for j in range(len(columns)):
            text = axs[x, y].text(j, i, avg_data[i, j],
                                  ha="center", va="center", color="black")

    axs[x, y].set_title('data sample: ' + sample_name)
    fig.tight_layout()
    fig.set_figheight(10)
    fig.set_figwidth(10)

    return fig, axs

def map_matrix(df, sample_name, what='max', remap='no'):

    rows = ['0','1','2','3','4','5','6','7','8','9','10','11']
    columns = ['0','1','2','3','4','5','6','7','8','9','10','11']
    
    if what == 'max':
        data = pd.DataFrame(df.max().values.reshape(12,12)).T
    elif what == 'min':
        data = pd.DataFrame(df.min().values.reshape(12,12)).T
        
    if remap == 'yes':
        data.iloc[[11],[11]] = 0
        rows = [2,1,0,3,8,9,4,5,6,10,11]
        columns = [3,4,5,6,7,8,9,10,11,0,1,2]
        data = data.loc[rows].loc[:,columns]

    # Plot the heatmap
    fig, ax = plt.subplots()
    im = ax.imshow(data, cmap="YlOrRd", vmin=0, vmax=3000)  # cmap="Wistia"

    # Create colorbar
    cbar_kw = {}
    cbarlabel = "Sensor max values (analog counts)"
    cbar = ax.figure.colorbar(im, ax=ax, **cbar_kw)
    cbar.ax.set_ylabel(cbarlabel, rotation=-90, va="bottom")

    # We want to show all ticks...
    ax.set_xticks(np.arange(len(columns)))
    ax.set_yticks(np.arange(len(rows)))
    # ... and label them with the respective list entries
    ax.set_xticklabels(columns)
    ax.set_yticklabels(rows)

    # Loop over data dimensions and create text annotations.
    for i in range(len(rows)):
        for j in range(len(columns)):
            try:
                text = ax.text(j, i, max_data[i, j],ha="center", va="center", color="black")
                pass
            except:
                pass

    ax.set_title('data sample: ' + sample_name+' '+what+' values')
    fig.tight_layout()
    plt.show()

def map_matrix2(fig, axs, df, sample_name, n, what='max', remap='no'):

    rows = ['0','1','2','3','4','5','6','7','8','9','10','11']
    columns = ['0','1','2','3','4','5','6','7','8','9','10','11']
    
    x = int(n / 3)
    y = n % 3
    
    if what == 'max':
        data = pd.DataFrame(df.max().values.reshape(12,12)).T
    elif what == 'min':
        data = pd.DataFrame(df.min().values.reshape(12,12)).T
        
    if remap == 'yes':
        data.iloc[[11],[11]] = 0
        rows = [2,1,0,3,8,9,4,5,6,10,11]
        columns = [3,4,5,6,7,8,9,10,11,0,1,2]
        data = data.loc[rows].loc[:,columns]

    # Plot the heatmap
    im = axs[x,y].imshow(data, cmap="YlOrRd", vmin=0, vmax=3000)  # cmap="Wistia"

    # Create colorbar
    cbar_kw = {}
    cbarlabel = "Sensor max values (analog counts)"
    cbar = axs[x,y].figure.colorbar(im, ax=axs[x,y], **cbar_kw)
    cbar.ax.set_ylabel(cbarlabel, rotation=-90, va="bottom")

    # We want to show all ticks...
    axs[x,y].set_xticks(np.arange(len(columns)))
    axs[x,y].set_yticks(np.arange(len(rows)))
    # ... and label them with the respective list entries
    axs[x,y].set_xticklabels(columns)
    axs[x,y].set_yticklabels(rows)

    # Loop over data dimensions and create text annotations.
    for i in range(len(rows)):
        for j in range(len(columns)):
            try:
                text = axs[x,y].text(j, i, max_data[i, j],ha="center", va="center", color="black")
                pass
            except:
                pass

    axs[x,y].set_title('data sample: ' + sample_name+' '+what+' values')
    
    return fig, axs



