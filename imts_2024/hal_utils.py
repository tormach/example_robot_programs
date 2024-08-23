from machinekit import hal
import numpy as np

#FILE:hal_utils.py

def getTorques():
    t0 = hal.Pin('lcec.0.0.torque-actual-value')
    torque0 = t0.get()
    t1 = hal.Pin('lcec.0.1.torque-actual-value')
    torque1 = t1.get()
    t2 = hal.Pin('lcec.0.2.torque-actual-value')
    torque2 = t2.get()
    t3 = hal.Pin('lcec.0.3.torque-actual-value')
    torque3 = t3.get()
    t4 = hal.Pin('lcec.0.4.torque-actual-value')
    torque4 = t4.get()
    t5 = hal.Pin('lcec.0.5.torque-actual-value')
    torque5 = t5.get()
    torques = np.array([torque0, torque1, torque2, torque3, torque4, torque5])
    return torques


def getJoints():
    joint0 = hal.Pin('lcec.0.0.position-actual-value')
    val0 = joint0.get()
    joint1 = hal.Pin('lcec.0.1.position-actual-value')
    val1 = joint1.get()
    joint2 = hal.Pin('lcec.0.2.position-actual-value')
    val2 = joint2.get()
    joint3 = hal.Pin('lcec.0.3.position-actual-value')
    val3 = joint3.get()
    joint4 = hal.Pin('lcec.0.4.position-actual-value')
    val4 = joint4.get()
    joint5 = hal.Pin('lcec.0.5.position-actual-value')
    val5 = joint5.get()
    joints = np.array([val0, -val1, val2, val3, val4, 0.0])
    return joints
