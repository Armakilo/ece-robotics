# program that plots input csv tracking data

# next step -> make gui that lets you put a pointer on the graph


import pandas as pd
import matplotlib.pyplot as plt
import os

import tkinter as tk
from tkinter import filedialog

root = tk.Tk()
root.withdraw()

folder_path = os.getcwd()

file_path = filedialog.askopenfilename(initialdir = folder_path, filetypes=[('Data Files','*.csv')])

print(f"Reading CSV file: {file_path}")
df = pd.read_csv(file_path)
df['stateChange'] = df['state'].shift(1, fill_value=df['state'].head(1)) != df['state']

print(df)

pos_ax = df.plot(kind='scatter', x='x_pred_m', y='y_pred_m', color='red', label='Predicted Path')
df.plot(kind = 'scatter', x='x_est_m', y='y_est_m', ax=pos_ax, label='Estimated Path', linestyle='--')

statelst = [] #for each dict item, have a list of tuples of positions made using append
statedict = {}

for i in range(len(df['stateChange'])):
    if(df['stateChange'][i] == True):
        curr_state = df['state'][i]
        if curr_state not in statedict:
            statedict[curr_state] = []
        statedict[curr_state].append(((df['x_est_m'][i]),df['y_est_m'][i]))

print(statedict)
        
for key, value in statedict.items():
    x_val = [x[0] for x in value]
    y_val = [x[1] for x in value]
    pos_ax.scatter(x_val, y_val, marker='D',label = key)
#pos_ax.plot(df['x_est_m'][i], df['y_est_m'][i], marker='o',label=curr_state, markersize=10)
        
# when the state changes, it displays on the graph

pos_ax.set_xlabel('X-Position')
pos_ax.set_ylabel('Y-Position')

pos_ax.legend()
pos_ax.set_title('Predicted vs. Estimated Postion')
pos_ax.grid()

plt.savefig('position.png', dpi=300)



rot_ax = df.plot(kind='line', x='time_ms', y=['heading_gyro','heading_pred'], label=['Estimated Heading', 'Predicted Heading'], title="Predicted vs. Estimated Angle")



plt.savefig('angle.png', dpi = 300)


#win = tk.Tk()



plt.show()