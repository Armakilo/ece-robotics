# program that plots input csv tracking data

# next step -> make gui that lets you put a pointer on the graph


import pandas as pd
import matplotlib.pyplot as plt
import os
from matplotlib.widgets import Button, Slider, Cursor
import numpy as np
from matplotlib.ticker import FuncFormatter

import tkinter as tk
from tkinter import filedialog

from mpl_toolkits.axes_grid1 import make_axes_locatable

root = tk.Tk()
root.withdraw()

folder_path = os.getcwd()

file_path = filedialog.askopenfilename(initialdir = folder_path, filetypes=[('Data Files','*.csv')])

print(f"Reading CSV file: {file_path}")
df = pd.read_csv(file_path)
df['stateChange'] = df['state'].shift(1, fill_value=df['state'].iloc[0]) != df['state']



time = df['time_ms'].to_numpy()


t_i = time[0]

t_rel_sec = [(t-t_i)/1000 for t in time]

df['rel_time'] = t_rel_sec

# fig, (pos_ax, rot_ax) = plt.subplots(1,2)

# pos_fig, pos_ax = plt.subplots(figsize=(6, 5))
# rot_fig, rot_ax = plt.subplots(figsize=(6, 5))

pos_ax = df.plot(kind='scatter', x='x_pred_m', y='y_pred_m', label='_nolegend_', c=t_rel_sec, cmap='viridis', figsize=(14,9))
df.plot(kind = 'scatter', x='x_est_m', y='y_est_m', ax=pos_ax, label='_nolegend_', linestyle='--', c=t_rel_sec, cmap='plasma') # est. path


s1, s2 = pos_ax.collections[0], pos_ax.collections[1]

statelst = [] #for each dict item, have a list of tuples of positions made using append
statedict = {}



for i in range(len(df['stateChange'])):
    if(df['stateChange'][i] == True):
        curr_state = df['state'][i]
        if curr_state not in statedict:
            statedict[curr_state] = []
        statedict[curr_state].append(((df['x_est_m'][i]),df['y_est_m'][i]))

        
for key, value in statedict.items():
    x_val = [x[0] for x in value]
    y_val = [x[1] for x in value]
    pos_ax.scatter(x_val, y_val, marker='D',label = key)
#pos_ax.plot(df['x_est_m'][i], df['y_est_m'][i], marker='o',label=curr_state, markersize=10)
        
# when the state changes, it displays on the graph

fig = pos_ax.figure
divider = make_axes_locatable(pos_ax)
cax1 = divider.append_axes("right", size="3%", pad=0.4)
cax2 = divider.append_axes("right", size="3%", pad=0.6)
fig.colorbar(s1, cax=cax1, label='Predicted Position at Time (s)')
fig.colorbar(s2, cax=cax2, label='Estimated Position at Time (s)')

pos_ax.set_xlabel('X-Position')
pos_ax.set_ylabel('Y-Position')

pos_ax.legend()
#pos_ax.set_title('Predicted vs. Estimated Postion')
pos_ax.grid()

pos_fig = plt.gcf

plt.savefig('position.png', dpi=300)



rot_ax = df.plot(kind='line', x='time_ms', y='heading_gyro', label='Estimated Heading',color = 'red', linestyle='--', figsize=(11,7))
df.plot(kind = 'line', x='time_ms', y='heading_pred', label='Predicted Heading',ax = rot_ax, color = 'blue', linestyle='-.')

rot_ax.set_ylabel('Angle (degrees)')
#rot_ax.set_title('Predicted and Estimated Rotation vs. Time')

rot_ax.xaxis.set_major_formatter(
    FuncFormatter(lambda x, _: f'{x/1000:g}')
)
rot_ax.set_xlabel('Time (s)')

plt.savefig('angle.png', dpi = 300)

plt.show()