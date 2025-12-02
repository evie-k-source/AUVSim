import os
import numpy as np
import math
import time
from datetime import datetime

class Logger:
    def __init__(self, logName):
        os.makedirs("logs", exist_ok=True)
        self.logFile = open(f"logs\\{logName}_{datetime.now().strftime("%Y%m%dT%H%M%S")}.log", 'w')
        self.logStart = time.time()
        self.write(f"Start Log: {logName}")
    
    def __del__(self):
        self.stop()
        
    def write(self, message):
        self.logFile.write(f"{time.time()-self.logStart:.3f}: \t{message}\n")
        
    def stop(self):
        self.logFile.close()

def linearFrameTransform(inertialRotation):
    if(inertialRotation.shape != (3,1)):
        if(AUVSim.Instance != None):
            AUVSim.Instance.writeLog(f"ERROR: Cannot perform linear frame transform: inertial rotation has shape: {inertialRotation.shape}")
        else:
            print(f"ERROR: Cannot perform linear frame transform: inertial rotation has shape: {inertialRotation.shape}")
        return
    phi = inertialRotation[0,0]     # Rotation about x
    theta = inertialRotation[1,0]   # Rotation about y
    psi = inertialRotation[2,0]     # Rotation about z
    return np.array([[np.cos(psi)*np.cos(theta), -np.sin(psi)*np.cos(phi)+np.cos(psi)*np.sin(theta)*np.sin(phi), np.sin(psi)*np.sin(phi)+np.cos(psi)*np.cos(phi)*np.sin(theta)],\
                     [np.sin(psi)*np.cos(theta), np.cos(psi)*np.cos(phi)+np.sin(phi)*np.sin(theta)*np.sin(psi), -np.cos(psi)*np.sin(phi)+np.sin(theta)*np.sin(psi)*np.cos(phi)],\
                     [-np.sin(theta), np.cos(theta)*np.sin(phi), np.cos(theta)*np.cos(phi)]])

def angularFrameTransform(inertialRotation):
    if(inertialRotation.shape != (3,1)):
        if(AUVSim.Instance != None):
            AUVSim.Instance.writeLog(f"ERROR: Cannot perform linear frame transform: inertial rotation has shape: {inertialRotation.shape}")
        else:
            print(f"ERROR: Cannot perform linear frame transform: inertial rotation has shape: {inertialRotation.shape}")
        return
    phi = inertialRotation[0,0]     # Rotation about x
    theta = inertialRotation[1,0]   # Rotation about y
    psi = inertialRotation[2,0]     # Rotation about z
    return np.array([[1, np.sin(phi)*np.tan(theta), np.cos(phi)*np.tan(theta)],\
                     [0, np.cos(phi), -np.sin(phi)],\
                     [0, np.sin(phi)/np.cos(theta), np.cos(phi)/np.cos(theta)]])

def eulerAngleToUnitVector(eulerAngles):
    return np.array([[np.cos(eulerAngles[2,0])*np.cos(eulerAngles[1,0])],\
                     [np.sin(eulerAngles[2,0])*np.cos(eulerAngles[1,0])],\
                     [np.sin(eulerAngles[1,0])]])
    
# Use only for GUI updates, not simulation
def polarToCartesian(polarCoords, angleOffset = 0, cartesianOffset = (0,0)):
    return (polarCoords[0]*math.cos(polarCoords[1] + angleOffset) + cartesianOffset[0],\
            polarCoords[0]*math.sin(polarCoords[1] + angleOffset) + cartesianOffset[1])