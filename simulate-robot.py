import pybullet as p
import time
import pybullet_data
import numpy as np
from datetime import datetime
import cv2 as cv
import pandas as pd
import os


#-------------------- init variables ----------------------------
right_forearm_ID = 0
right_lowerarm_ID = 0
right_hand_ID = 0

left_forearm_ID = 0
left_lowerarm_ID = 0
left_hand_ID = 0

base_ID = 0

maxForce = 0
mode = p.VELOCITY_CONTROL

pixelWidth = 128
pixelHeight = 128
camTargetPos = [0.5, 0.5, 0]
camDistance = 4
pitch = -10.0
roll = 0
upAxisIndex = 2
headUpDown = 1.2
headLeftRight = 0
cudsteering = 0
clrsteering = 0
cameraProjectionMatrix = [
    1.0825318098068237, 0.0, 0.0, 0.0, 0.0, 1.732050895690918, 0.0, 0.0, 0.0, 0.0,
    -1.0002000331878662, -1.0, 0.0, 0.0, -0.020002000033855438, 0.0
]
all_states = []
data_column_names = ['date', 'base_link']
column_names = [['Date'], ['Position','Orientation']]






#---------------------------- init physics engine  -----------------------------

np.set_printoptions(precision=3)
physicsClient = p.connect(p.GUI)#p.GUI or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])

#---------------- load all objects ----------------------------------------------
#  
planeid = p.loadURDF("plane.urdf")
robotID = p.loadURDF("simple-robot.urdf",startPos, startOrientation)
box1ID = p.loadURDF("box.urdf",[3,3.1,1], startOrientation)
box2ID = p.loadURDF("box.urdf",[3,3.2,2], startOrientation)
box3ID = p.loadURDF("box.urdf",[3,3.3,3], startOrientation)
box4ID = p.loadURDF("box.urdf",[3,3.5,4], startOrientation)
box5ID = p.loadURDF("box.urdf",[3.3,3,5], startOrientation)
box6ID = p.loadURDF("box.urdf",[3.4,3,6], startOrientation)
box7ID = p.loadURDF("box.urdf",[3.6,3,7], startOrientation)
box8ID = p.loadURDF("box.urdf",[3.7,3,8], startOrientation)



#------------------------ get names of all links/joints ---------------------------------

for temp_joint in range(p.getNumJoints(robotID)):
    
    p.setJointMotorControl2(robotID, temp_joint,controlMode=mode, force=maxForce)
    nameOfLink = p.getJointInfo(robotID, temp_joint)[12]
    data_column_names.append(nameOfLink.decode()+'_link')
    data_column_names.append(nameOfLink.decode()+'_joint')

    column_names.append(['position_absolute', \
    'orientation_relative', \
    #'center_of_mass_position_in_robots_coordinates', \
    #'center_of_mass_orientation_in_robots_coordinates', \
    #'position_absolute2', \
    #'orientation_absolute2',\
    'absolute_linear_velocity',\
    'absolute_angular_velocity'])
    
    column_names.append(['joint_position',\
    'joint_velocity', \
    'joint_reaction_forces', \
    #'applied_joint_motor_torque'\
    ])

    p.enableJointForceTorqueSensor(robotID, temp_joint, 1)
    if(nameOfLink.decode()=='right_forearm'):
        right_forearm_ID = temp_joint
    if(nameOfLink.decode()=='right_lowerarm'):
        right_lowerarm_ID = temp_joint
    if(nameOfLink.decode()=='right_hand'):
        right_hand_ID = temp_joint
    if(nameOfLink.decode()=='left_forearm'):
        left_forearm_ID = temp_joint
    if(nameOfLink.decode()=='left_lowerarm'):
        left_lowerarm_ID = temp_joint
    if(nameOfLink.decode()=='left_hand'):
        left_hand_ID = temp_joint
    if(nameOfLink.decode()=='base_link'):
        base_ID = temp_joint

data_column_names.append('camera')
column_names.append(['focus_position_relative'])

data_column_names.append('image')
column_names.append(['image'])
rgba=[]


#------------------------- Start simulation -------------------------------------
before = time.time()
last_time = before
for step in range (10000):

    
    startTime = datetime.now()


    #---------------------- Keyboard controll -----------------------------------
    keys = p.getKeyboardEvents()
    lrsteering = 0.0
    udsteering = 0.0
    cudsteering = 0.0
    clrsteering = 0.0
    for k, v in keys.items():
        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):
            lrsteering = 100.2
        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)):
            lrsteering = -100.2
        if (k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN)):
            udsteering = 100.2
        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
            udsteering = -100.2
        if (k == 104 and (v & p.KEY_IS_DOWN)):
            clrsteering = 0.1
        if (k == 107 and (v & p.KEY_IS_DOWN)):
            clrsteering = -0.1
        if (k == 117 and (v & p.KEY_IS_DOWN)):
            cudsteering = 0.1
        if (k == 106 and (v & p.KEY_IS_DOWN)):
            cudsteering = -0.1
        if (k == 113 and (v & p.KEY_IS_DOWN)):
            quit()
    c = 10
    p.applyExternalForce(robotID,-1,[c*udsteering, 0,0],[0,0,0], p.LINK_FRAME)
    p.applyExternalForce(robotID,-1,[0, 0.6*c*-lrsteering,0],[0.4,0,0], p.LINK_FRAME)



    if(step%4 ==0):
        current_state = [(time.time())-before]
        base_pos_orientation = p.getBasePositionAndOrientation(robotID)
        robotPos = base_pos_orientation[0] 
        robotOrn = base_pos_orientation[1]
        orientation_euler = p.getEulerFromQuaternion(robotOrn)
        current_state.append([robotPos,orientation_euler])
        #------------------------ get states of all links and joints ---------------------------
        for temp_joint in range(p.getNumJoints(robotID)):

            position_absolute,\
            orientation_absolute,\
            center_of_mass_position_in_robots_coordinates,\
            center_of_mass_orientation_in_robots_coordinates,\
            position_absolute2,\
            orientation_absolute2,\
            absolute_linear_velocity,\
            absolute_angular_velocity = p.getLinkState(robotID, temp_joint,1,1)
            position_relative = tuple(np.subtract(position_absolute, robotPos))
            orientation_absolute = p.getEulerFromQuaternion(orientation_absolute)

            joint_position, \
            joint_velocity, \
            joint_reaction_forces, \
            applied_joint_motor_torque =  p.getJointState(robotID,temp_joint)

            current_state.append([position_relative, orientation_absolute, absolute_linear_velocity, absolute_angular_velocity])
            current_state.append([joint_position, joint_velocity, joint_reaction_forces])


        #----------------------------------- Calculate camera orientation ---------------------
        #defines where camera looks horizont or down
        headUpDown = headUpDown+cudsteering
        if(headUpDown>5):
            headUpDown = 5
        if(headUpDown<0):
            headUpDown = 0
        headLeftRight = headLeftRight + clrsteering

        #get transformation matrix from quaternion of robot base rotation 
        rot_mat = p.getMatrixFromQuaternion(robotOrn)

        #calculate position of camera and point of view based on rotation of base
        array_of_el = np.array(rot_mat)
        matrix = array_of_el.reshape((3, 3))
        camera_view_vector  = np.array([headUpDown, headLeftRight, 0])
        transf_view = matrix.dot(camera_view_vector)
        camera_origin_vector  = np.array([0.3, 0, 0.8])
        transf_origin= matrix.dot(camera_origin_vector)
        camera_up_vector  = np.array([0, 0, 1.0])
        transf_up= matrix.dot(camera_up_vector)
        #print(transf_origin, transf_view)
        viewMatrix = p.computeViewMatrix((robotPos[0]+transf_origin[0],robotPos[1]+transf_origin[1],robotPos[2]+transf_origin[2]), (robotPos[0]+transf_view[0],robotPos[1]+transf_view[1],robotPos[2]+transf_view[2]),transf_up)

        current_state.append([camera_view_vector])

        #--------------------------- get camera image ---------------------------------------------
        w, h, rgba, depth, mask = p.getCameraImage(pixelWidth,
                                pixelHeight,
                                viewMatrix=viewMatrix,
                                projectionMatrix=cameraProjectionMatrix,
                                shadow=1,
                                lightDirection=[1, 1, 1])
        
        image_ar = np.array(rgba)
        image_ar = image_ar.reshape(pixelWidth,pixelHeight,4)
        image_ar = image_ar[:,:,0:3]
        flat_image = image_ar.flatten()


        #-------------------------- add state to history -------------------------------------------
        current_state.append([flat_image])
        all_states.append(current_state)

    #------------------------- make another step o simulation ----------------------------------

    p.stepSimulation()
    #time.sleep(1./240.)
    #print(datetime.now() - startTime)
    now = time.time()
    print(now-last_time)
    last_time = now
p.disconnect()


#--------------------------------- save history ------------------------------------------------

#first save names of data
temp_df = pd.DataFrame(data_column_names, columns=['names'])
os.remove('history.h5')
temp_df.to_hdf('history.h5', 'names')

for idx, data_name in enumerate(data_column_names):
    extracted_one_column = [row[idx] for row in all_states]
    new_df = pd.DataFrame(extracted_one_column, columns=column_names[idx])
    new_df.to_hdf('history.h5', data_column_names[idx])



#image = cv.imread('testimage.bmp')
#cv.imshow('a',image)
cv.waitKey(0)
