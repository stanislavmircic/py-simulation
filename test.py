import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import cv2 as cv

pixelWidth = 128
pixelHeight = 128
names_df = pd.read_hdf('history.h5', key='names')
print(names_df)

#get time axis
first_name = names_df['names'].iloc[0]
date_df = pd.read_hdf('history.h5', key=first_name)

#print all links and joints
for name_index, name in enumerate(names_df['names']):
    if name == 'date' or name == 'image':
        continue
    data_name = names_df['names'].iloc[name_index]
    history_df = pd.read_hdf('history.h5', key=data_name)
    number_of_graph_rows = int(len(history_df.columns)/2)
    if(number_of_graph_rows<1):
        number_of_graph_rows = 1
    fig, axs = plt.subplots(number_of_graph_rows,2)
    fig.suptitle(data_name + ' parameters')


    for index, col in enumerate(history_df.columns):
        selected_df = history_df.loc[:,col]
        print(selected_df)
        temp_column = selected_df.to_numpy()
        new_np_arr = []
        [new_np_arr.append(np.array(row)) for row in temp_column]
        array_np = np.array(new_np_arr)
        if(len(axs.shape)==1):
            axs[index].plot(array_np)
            axs[index].set_title(data_name + ' - '+ col)
        else:
            axs[int(index/2), index%2].plot(array_np)
            axs[int(index/2), index%2].set_title(data_name + ' - '+ col)
    fig.show()


data_name = names_df['names'].iloc[1]


# history_df = pd.read_hdf('history.h5', key='image')
# print(history_df)
# out = cv.VideoWriter('output.avi', cv.VideoWriter_fourcc(*'DIVX'), 240, (pixelWidth, pixelHeight))
# for index, row in history_df.iterrows():
#     one_frame = row['image']
#     one_frame = one_frame.reshape(pixelWidth,pixelWidth,3)
#     out.write(np.uint8(one_frame))
#     #cv.imshow('Frame', one_frame)
# #[out.write(f) for _,row in history_df]
# out.release()
# print('Video created')
