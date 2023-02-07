import numpy as np
import pandas as pd
import cv2 as cv

pixelWidth = 128
pixelHeight = 128

history_df = pd.read_hdf('history.h5', key='image')
#print(history_df)
out = cv.VideoWriter('output.avi', cv.VideoWriter_fourcc(*'DIVX'), 60, (pixelWidth, pixelHeight))
for index, row in history_df.iterrows():
    one_frame = row['image']
    one_frame = one_frame.reshape(pixelWidth,pixelWidth,3)
    out.write(np.uint8(one_frame))
    #cv.imshow('Frame', one_frame)
#[out.write(f) for _,row in history_df]
out.release()
print('Video created')