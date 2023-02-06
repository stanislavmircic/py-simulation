import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

history_df = pd.read_hdf('history.h5', key='history')
column_headers = list(history_df.columns.values)
print("The Column Header :", column_headers)

right_forearm_joint_position = []
right_forearm_joint_velocity = []
for _,row in history_df.iterrows():
    right_forearm_joint = row['right_forearm_joint']
    right_forearm_joint_position.append(right_forearm_joint[0])
    right_forearm_joint_velocity.append(right_forearm_joint[1])
    #print(right_forearm_joint.shape())

plt.title("right_forearm_joint position") 
plt.xlabel("simulation steps") 
plt.ylabel("position") 
plt.plot(right_forearm_joint_velocity) 
plt.show()