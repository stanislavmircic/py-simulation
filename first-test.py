import pybullet as p
import time
import pybullet_data
import numpy as np
from datetime import datetime




np.set_printoptions(precision=3)
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeid = p.loadURDF("plane.urdf")
startPos = [0,0,1]
startOrientation = p.getQuaternionFromEuler([0,0,0])
robotID = p.loadURDF("simple-robot.urdf",startPos, startOrientation)
box1ID = p.loadURDF("box.urdf",[3,3.1,1], startOrientation)
box2ID = p.loadURDF("box.urdf",[3,3.2,2], startOrientation)
box3ID = p.loadURDF("box.urdf",[3,3.3,3], startOrientation)
box4ID = p.loadURDF("box.urdf",[3,3.5,4], startOrientation)
box5ID = p.loadURDF("box.urdf",[3.3,3,5], startOrientation)
box6ID = p.loadURDF("box.urdf",[3.4,3,6], startOrientation)
box7ID = p.loadURDF("box.urdf",[3.6,3,7], startOrientation)
box8ID = p.loadURDF("box.urdf",[3.7,3,8], startOrientation)
#boxId = p.loadURDF("r2d2.urdf",[3,3,1], startOrientation)
#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)


right_forearm_ID = 0
right_lowerarm_ID = 0
right_hand_ID = 0

left_forearm_ID = 0
left_lowerarm_ID = 0
left_hand_ID = 0

base_ID = 0

maxForce = 0
mode = p.VELOCITY_CONTROL

pixelWidth = 200
pixelHeight = 200
camTargetPos = [0.5, 0.5, 0]
camDistance = 4
pitch = -10.0
roll = 0
upAxisIndex = 2
headUpDown = 1.2
headLeftRight = 0
cameraProjectionMatrix = [
    1.0825318098068237, 0.0, 0.0, 0.0, 0.0, 1.732050895690918, 0.0, 0.0, 0.0, 0.0,
    -1.0002000331878662, -1.0, 0.0, 0.0, -0.020002000033855438, 0.0
]

for temp_joint in range(p.getNumJoints(robotID)):
    p.setJointMotorControl2(robotID, temp_joint,controlMode=mode, force=maxForce)
    nameOfLink = p.getJointInfo(robotID, temp_joint)[12]
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



for i in range (1000000):
    startTime = datetime.now()
    keys = p.getKeyboardEvents()
    lrsteering = 0.0
    udsteering = 0.0
    for k, v in keys.items():
        if (k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):
            lrsteering = 100.2
        if (k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)):
            lrsteering = -100.2
        if (k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN)):
            udsteering = 100.2
        if (k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
            udsteering = -100.2
    c = 100
    p.applyExternalForce(robotID,-1,[c*udsteering, c*-lrsteering,0],[0,0,0], p.LINK_FRAME)
    #p.applyExternalTorque(robotID,right_forearm_ID,[0,-udsteering,0],p.LINK_FRAME)
    #print(udsteering)
    #p.applyExternalTorque(robotID,right_lowerarm_ID,[0,0,-lrsteering],p.LINK_FRAME)  
    
    #joint_reaction_forces = 3 Reaction forces and 3 Moments (torques) 
    #   
    #jont_position_angle,joint_angular_velocity, joint_reaction_forces,_ = p.getJointState(robotID, right_forearm_ID)

    #print(np.array([jont_position_angle]),np.array([joint_angular_velocity]),np.array(rf),np.array(jt))
    
    #p.applyExternalForce(robotID,right_forearm_ID,[0,lrsteering,udsteering],[0,0,0],p.LINK_FRAME)
    

#    position_absolute,\
#    orientation_absolute,\
#    center_of_mass_position_in_robots_coordinates,\ - useless, always constant
#    center_of_mass_orientation_in_robots_coordinates,\  - useless, always constant
#    position_absolute2,\ - almost the same as position_absolute
#    orientation_absolute2,\ the same as orientation_absolute
#    absolute_linear_velocity,\  maybe usefull to calculate acceleration
#    absolute_angular_velocity 


    
    position_absolute,\
    orientation_absolute,\
    center_of_mass_position_in_robots_coordinates,\
    center_of_mass_orientation_in_robots_coordinates,\
    position_absolute2,\
    orientation_absolute2,\
    absolute_linear_velocity,\
    absolute_angular_velocity = p.getLinkState(robotID, base_ID,1,1)

    # print(np.array(position_absolute),\
    # np.array(orientation_absolute),\
    # np.array(center_of_mass_position_in_robots_coordinates),\
    # np.array(center_of_mass_orientation_in_robots_coordinates),\
    # np.array(position_absolute2),\
    # np.array(orientation_absolute2),\
    # np.array(absolute_linear_velocity),\
    # np.array(absolute_angular_velocity) )
    
    robotPos, robotOrn = p.getBasePositionAndOrientation(robotID)
   
    #defines where camera looks horizont or down
    headUpDown = headUpDown+0.001*udsteering
    if(headUpDown>5):
        headUpDown = 5
    if(headUpDown<0):
        headUpDown = 0
    headLeftRight = headLeftRight + 0.001*lrsteering

    #get transformation matrix from quaternion of robot base rotation 
    rot_mat = p.getMatrixFromQuaternion(orientation_absolute)

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


    img_arr = p.getCameraImage(pixelWidth,
                               pixelHeight,
                               viewMatrix=viewMatrix,
                               projectionMatrix=cameraProjectionMatrix,
                               shadow=1,
                               lightDirection=[1, 1, 1])
    
    
    #print(m_steering)
    #startTime = datetime.now()
    p.stepSimulation()
    #time.sleep(1./240.)
    print(datetime.now() - startTime)
p.disconnect()
