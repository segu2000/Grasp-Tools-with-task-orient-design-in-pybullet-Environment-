import pybullet as p
import time
import math
import random
import pybullet_data
from datetime import datetime
import numpy as np

######################################################### Simulation Setup ############################################################################

clid = p.connect(p.GUI)
if (clid < 0):
    p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())

planeId = p.loadURDF("plane.urdf", [0, 0, -1])

p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
sawyerId = p.loadURDF("./data/sawyer_robot/sawyer_description/urdf/sawyer.urdf", [0, 0, 0], [0, 0, 0, 3],
                      useFixedBase=1)  # load sawyer robot


tableId = p.loadURDF("./data/table/table.urdf", [1.1, 0.000000, -0.3],
                     p.getQuaternionFromEuler([(math.pi / 2), 0, (math.pi / 2)]), useFixedBase=1, flags=8)


######################################################### Load Object Here!!!!#############################################################################

# load object, change file name to load different objects
# p.loadURDF(finlename, position([X,Y,Z]), orientation([a,b,c,d]))
# Example:
# objectId = p.loadURDF("random_urdfs/001/001.urdf", [1.25 ,0.25,-0.1], p.getQuaternionFromEuler([0,0,1.56])) # pi*0.5

xpos = 0.95
ypos = 0
ang = 3.14 * 0.5
orn = p.getQuaternionFromEuler([0, 0, ang])

object_path ='/data/random_urdfs/010/010.urdf'
objectId = p.loadURDF(object_path, xpos, ypos, -0.03, orn[0], orn[1], orn[2], orn[3])

# tray
tray_x = 1.08526707685750519
tray_y = 0.36920265473142814



trayId = p.loadURDF("./data/tray/tray.urdf", [tray_x, tray_y, 0], [0, 0, 0, 3])



###########################################################################################################################################################


p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
p.resetBasePositionAndOrientation(sawyerId, [0, 0, 0], [0, 0, 0, 1])

sawyerEndEffectorIndex = 16
numJoints = p.getNumJoints(sawyerId)  # 65 with ar10 hand

# useRealTimeSimulation = 0
# p.setRealTimeSimulation(useRealTimeSimulation)
# p.stepSimulation()
# all R joints in robot
js = [3, 4, 8, 9, 10, 11, 13, 16, 21, 22, 23, 26, 27, 28, 30, 31, 32, 35, 36, 37, 39, 40, 41, 44, 45, 46, 48, 49, 50,
      53, 54, 55, 58, 61, 64]
# lower limits for null space
ll = [-3.0503, -5.1477, -3.8183, -3.0514, -3.0514, -2.9842, -2.9842, -4.7104, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17,
      0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.17, 0.85, 0.34,
      0.17]
# upper limits for null space
ul = [3.0503, 0.9559, 2.2824, 3.0514, 3.0514, 2.9842, 2.9842, 4.7104, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57,
      0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 1.57, 1.57, 0.17, 2.15, 1.5, 1.5]
# joint ranges for null space
jr = [0, 0, 0, 0, 0, 0, 0, 0, 1.4, 1.4, 1.4, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4, 0, 1.4, 1.4,
      0, 1.4, 1.4, 0, 1.3, 1.16, 1.33]
# restposes for null space
rp = [0] * 35
# joint damping coefficents
jd = [1.1] * 35


######################################################### Inverse Kinematics Function ##########################################################################

# Finger tip ID: index:51, mid:42, ring: 33, pinky:24, thumb 62
# Palm ID: 20
# move palm (center point) to reach the target postion and orientation
# input: targetP --> target postion
#        orientation --> target orientation of the palm
# output: joint positons of all joints in the robot
#         control joint to correspond joint position
def palmP(targetP, orientation):
    jointP = [0] * 65
    jointPoses = p.calculateInverseKinematics(sawyerId, 19, targetP, targetOrientation=orientation, jointDamping=jd)
    j = 0
    for i in js:
        jointP[i] = jointPoses[j]
        j = j + 1

    for i in range(p.getNumJoints(sawyerId)):
        p.setJointMotorControl2(bodyIndex=sawyerId,
                                jointIndex=i,
                                controlMode=p.POSITION_CONTROL,
                                targetPosition=jointP[i],
                                targetVelocity=0,
                                force=50000,
                                positionGain=0.03,
                                velocityGain=1)
    return jointP


######################################################### Hand Direct Control Functions ##########################################################################

# control the lower joint and middle joint of pinky finger, range both [0.17 - 1.57]
def pinkyF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[21, 26, 22, 27],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of ring finger, range both [0.17 - 1.57]
def ringF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[30, 35, 31, 36],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of mid finger, range both [0.17 - 1.57]
def midF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[39, 44, 40, 45],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of index finger, range both [0.17 - 1.57]
def indexF(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[48, 53, 49, 54],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, lower, middle, middle],
                                targetVelocities=[0, 0, 0, 0],
                                forces=[500, 500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1, 1])


# control the lower joint and middle joint of thumb, range: low [0.17 - 1.57], mid [0.34, 1.5]
def thumb(lower, middle):
    p.setJointMotorControlArray(bodyIndex=sawyerId,
                                jointIndices=[58, 61, 64],
                                controlMode=p.POSITION_CONTROL,
                                targetPositions=[lower, middle, middle],
                                targetVelocities=[0, 0, 0],
                                forces=[500, 500, 500],
                                positionGains=[0.03, 0.03, 0.03],
                                velocityGains=[1, 1, 1])


######################################################### detected information ##########################################################################
'''
19 b'palm'
20 b'rb1'
21 b'finger1.1a'
22 b'finger1.1b'
23 b'finger1.1c'
24 b'fingertip1'
25 b'rb2'
26 b'finger1.2a'
27 b'finger1.2b'
28 b'finger1.2c'
29 b'rb3'
30 b'finger2.1a'
31 b'finger2.1b'
32 b'finger2.1c'
33 b'fingertip2'
34 b'rb4'
35 b'finger2.2a'
36 b'finger2.2b'
37 b'finger2.2c'
38 b'rb5'
39 b'finger3.1a'
40 b'finger3.1b'
41 b'finger3.1c'
42 b'fingertip3'
43 b'rb6'
44 b'finger3.2a'
45 b'finger3.2b'
46 b'finger3.2c'
47 b'rb7'
48 b'finger4.1a'
49 b'finger4.1b'
50 b'finger4.1c'
51 b'fingertip4'
52 b'rb8'
53 b'finger4.2a'
54 b'finger4.2b'
55 b'finger4.2c'
56 b'rb9'
57 b'rb10'
58 b'thumb1'
59 b'thumb2'
60 b'rb11'
61 b'thumb3.1'
62 b'thumbtip'
63 b'rb12'
64 b'thumb3.2'
hand = [21, 22, 23, 26, 27, 28, 30, 31, 32, 35, 36 ,37, 39, 40, 41, 44, 45, 46, 48, 49, 50, 53, 54, 55, 58, 61, 64]
[0.2196998776260993, 0.9841056922424084, 0.16991782178342238, 0.21967883521345558, 0.9846229478397389, 0.1699958046620013, 0.5711534611694058, 0.5914229523765463, 0.16999954970542672, 0.573730600144428, 0.5902151809391006, 0.17000660753266578, 0.9359158730554522, 0.265116872922352, 0.170003190706592, 0.9361250259528252, 0.2652466938834658, 0.17003347470289248, 0.9068051254489781, 0.2490975329073341, 0.17008149880963058, 0.9066050389575453, 0.2502858674912193, 0.16999999999999976, 1.5698468053021237, 0.34006621802344955, 0.3400508342876441]

'''


def info():
    palmContact = []
    thumbContact = []
    indexContact = []
    midContact = []
    ringContact = []
    pinkyContact = []
    palmLinks = [19, 20, 25, 29, 34, 38, 43, 47, 52, 56, 57]
    thumbLinks = [58, 59, 60, 61, 62, 63, 64]
    indexLinks = [48, 49, 50, 51, 53, 54, 55]
    middleLinks = [39, 40, 41, 42, 44, 45, 46]
    ringLinks = [30, 31, 32, 33, 35, 36, 37]
    pinkyLinks = [21, 22, 23, 24, 26, 27, 28]

    contact = p.getContactPoints(sawyerId, objectId)  # pubullet quick guide
    nums = len(contact)
    if (nums == 0):
        print("There are no contact points")
        return [], [], [], [], [], []

    for i in range(nums):
        temp = []
        if (contact[i][3] in palmLinks):
            temp.append(contact[i][3])
            temp.append(contact[i][6])
            temp.append(contact[i][9])
            temp.append(contact[i][10])
            temp.append(contact[i][11])
            temp.append(contact[i][12])
            temp.append(contact[i][13])
            palmContact.append(temp)

        if (contact[i][3] in thumbLinks):
            temp.append(contact[i][3])
            temp.append(contact[i][6])
            temp.append(contact[i][9])
            temp.append(contact[i][10])
            temp.append(contact[i][11])
            temp.append(contact[i][12])
            temp.append(contact[i][13])
            thumbContact.append(temp)

        if (contact[i][3] in indexLinks):
            temp.append(contact[i][3])
            temp.append(contact[i][6])
            temp.append(contact[i][9])
            temp.append(contact[i][10])
            temp.append(contact[i][11])
            temp.append(contact[i][12])
            temp.append(contact[i][13])
            indexContact.append(temp)

        if (contact[i][3] in middleLinks):
            temp.append(contact[i][3])
            temp.append(contact[i][6])
            temp.append(contact[i][9])
            temp.append(contact[i][10])
            temp.append(contact[i][11])
            temp.append(contact[i][12])
            temp.append(contact[i][13])
            midContact.append(temp)

        if (contact[i][3] in ringLinks):
            temp.append(contact[i][3])
            temp.append(contact[i][6])
            temp.append(contact[i][9])
            temp.append(contact[i][10])
            temp.append(contact[i][11])
            temp.append(contact[i][12])
            temp.append(contact[i][13])
            ringContact.append(temp)

        if (contact[i][3] in pinkyLinks):
            temp.append(contact[i][3])
            temp.append(contact[i][6])
            temp.append(contact[i][9])
            temp.append(contact[i][10])
            temp.append(contact[i][11])
            temp.append(contact[i][12])
            temp.append(contact[i][13])
            pinkyContact.append(temp)

    return palmContact, thumbContact, indexContact, midContact, ringContact, pinkyContact


######################################################### Simulation ##########################################################################
currentP = [0] * 65
k = 0

handInitial = [0.1745631085544039, 0.1782199842342839, 0.17000049054390223, 0.17574630988469556, 0.16997785827638068, 0.2097479613791115, 0.32065248846823124, 0.2139570666281873, 0.1699361852141938, 0.31638080766518206, 0.20761372512381612, 0.16999919477480716, 0.5718429771568684, 1.4576079422113783, 0.18251364922936328, 0.5778739348713193, 1.4014264228848239, 0.1699980975645333, 0.7752309113564865, 1.5285909347630509, 0.17018867728755527, 0.7757628503042919, 1.5308398766139475, 0.17013692296726954, 1.2036168509325875, 0.883009649447393, 0.8835241500002111]
orientation = [1.2892775535583496, 2.827588395276342, 1.2237756252288818]
palmPosition = [0.9225473534199409, -0.12, -0.10]


grasp_PalmPosition = [0.853691144981351, 0.12436911449813544, -0.14864369114498138]
grasp_Orientation = [1.5876940250396729, 2.89372713963059, 0.43010997772216797]

handClose = [1.3193114076895784, 1.3826360091043157, 0.25745267141552725, 1.3198822885551442, 1.3845745803953289, 0.25960343038521616, 1.3442366410245705, 1.5206210691644306, 0.2517103253086803, 1.3411362971486356, 1.5240080417419137, 0.1700057437056814, 1.3779657904385751, 1.517056953560024, 0.1700059281414019, 1.3784086733764946, 1.5178879133825733, 0.17001923705995461, 1.3857306085939638, 1.5455491074090784, 0.17009886631340063, 1.3853553478060845, 1.548004401344025, 0.16999999992298168, 1.5697483939123225, 1.1337019033975515, 1.133699213551791]
pullUp_PalmPosition = [0.826878014539998, 0.01482111323426544, 0.19864954828951718]
pullUp_Orientation = [1.4876940250396729, 2.89372713963059, 0.43010997772216797]
handOpen = [0.17511074800897442, 0.17915391508142833, 0.20380569227987275, 0.17654380101883083, 0.16999976286797286, 0.2357988886060401, 0.17753175446149025, 0.17699822236515184, 0.16993128277401526, 0.1703083778737264, 0.1701125069859854, 0.1700100499416112, 0.17181476487283692, 0.17193023964447912, 0.17001094069996706, 0.17012614191205025, 0.1742355591908302, 0.16999999999999996, 0.17011563032427163, 0.1811414227588713, 0.169999559733343, 0.1699957995558682, 0.1704627324158768, 0.1700676061046703, 0.8499999340038903, 0.3406534408714631, 0.3405000355929386]#[0.23196868234123635, 0.3245622538618081, 0.22133849203313863, 0.23344900723784756, 0.31842096695252925, 0.22836056508471872, 0.2950744295631299, 0.3304491600188486, 0.18848576462826117, 0.28799729364344373, 0.32567330444701215, 0.17000768885052503, 0.35550505669931687, 0.2680087921182787, 0.17000728392749667, 0.3542664043605116, 0.2704357154892403, 0.16999999999999996, 0.5088865173298415, 0.2596289908114146, 0.16999999686429118, 0.5085758744196797, 0.25245664404380824, 0.17000553947804822, 0.9823500246222918, 0.34071838762075174, 0.34052199635088465]
final_PalmPosition = [0.9892775535583496, 0.327588395276342, 0.1237756252288818]
final_Orientation = [1.2892775535583496, 2.827588395276342, 1.2237756252288818]


##################################################################################################################################################################################################
initial_palmPosition = [0.79, -0.05, 0.3]
initial_orientation = [1.2892775535583496, 2.827588395276342, 1.2237756252288818]

palmPosition = [palmPosition[0]-0.1,palmPosition[1]-0.05,palmPosition[2]]
##################################################################################################################################################################################################
##################################################################### Wirte functions here ##########################################################################################################
# write the code for step 9-12
p.setGravity(0, 0, -10)

while 1:
    k = k + 1

    i = 0
    while 1:
        i += 1
        currentP = palmP([initial_palmPosition[0], initial_palmPosition[1], initial_palmPosition[2]],
                         p.getQuaternionFromEuler([initial_orientation[0], initial_orientation[1], initial_orientation[2]]))
        time.sleep(0.03)
        p.stepSimulation()

        if (i == 100):
            print('Initial location')
            break

    i = 0
    while 1:
        i += 1
        currentP = palmP([grasp_PalmPosition[0], grasp_PalmPosition[1], grasp_PalmPosition[2]],
                         p.getQuaternionFromEuler([grasp_Orientation[0], grasp_Orientation[1], grasp_Orientation[2]]))
        time.sleep(0.03)
        p.stepSimulation()

        if (i == 100):
            print('Grasp location')
            break

    i = 0
    while 1:
        i += 1
        thumb(handClose[25], handClose[26]),
        indexF(handClose[18], handClose[22]),
        midF(handClose[15], handClose[16]),
        ringF(handClose[6], handClose[7]),
        pinkyF(handClose[0], handClose[1]),
        p.stepSimulation()
        if i == 300:
            print("Object in hand")
            break

    i = 0
    while 1:
        i += 1
        currentP = palmP([pullUp_PalmPosition[0], pullUp_PalmPosition[1], pullUp_PalmPosition[2]],
                         p.getQuaternionFromEuler([pullUp_Orientation[0], pullUp_Orientation[1], pullUp_Orientation[2]]))
        time.sleep(0.03)
        p.stepSimulation()

        if (i == 100):
            print('Pulled up location')
            break

    i = 0
    while 1:
        i += 1
        currentP = palmP([final_PalmPosition[0], final_PalmPosition[1], final_PalmPosition[2]],
                         p.getQuaternionFromEuler([final_Orientation[0], final_Orientation[1], final_Orientation[2]]))
        time.sleep(0.03)
        p.stepSimulation()

        if (i == 100):
            print('Final location')
            break

    i = 0
    while 1:
        i += 1
        thumb(handOpen[24], handOpen[26]),
        indexF(handOpen[19], handOpen[21]),
        midF(handOpen[14], handOpen[16]),
        ringF(handOpen[8], handOpen[9]),
        pinkyF(handOpen[2], handOpen[5]),
        p.stepSimulation()
        if i == 300:
            print("Object in tray")
            break


p.disconnect()
print("disconnected")
