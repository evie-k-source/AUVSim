import tkinter as tk
import time
from datetime import datetime
import numpy as np
import math

LOOP_DELAY = 1
SIM_SPEED = 5
MAX_CLICK_DISTANCE = 10
AUV_SHAPE = ((0,0),(10,-0.75*math.pi),(10,0),(10,0.75*math.pi)) # Stored in polar coords
SELECT_RADIUS = 10
AUV_COLOR = "white"
AUV_SELECT_COLOR = "yellow"
DISPLAY_COLOR = "black"
POI_COLOR = "red"
POI_RADIUS = 2
POI_INDICATOR_LENGTH = 0
DEFAULT_DEPTH = -50
MAX_TARGET_SPEED = 0.5 # m/s

SEAWATER_DENSITY = 1030 # kg/m3
DRAG_COEFFICIENT = 1.05

# FIXME: arbitrary auv stats, should be pulled from file with precalced values
# Assumes rectangular prism of uniform mass
MAX_MOTOR_FORCE = 100 # newtons
MIN_MOTOR_THROTTLE = -0.1
MAX_MOTOR_THROTTLE = 1
AUV_MASS = 250 # kg
AUV_WIDTH = 1.016 #meters
AUV_LENGTH = 1.7272 #meters
AUV_HEIGHT = 1.2954 #meters
AUV_MOMENT_X = 1/12*AUV_MASS*(AUV_WIDTH*AUV_WIDTH + AUV_HEIGHT*AUV_HEIGHT)
AUV_MOMENT_Y = 1/12*AUV_MASS*(AUV_LENGTH*AUV_LENGTH + AUV_HEIGHT*AUV_HEIGHT)
AUV_MOMENT_Z = 1/12*AUV_MASS*(AUV_WIDTH*AUV_WIDTH + AUV_LENGTH*AUV_LENGTH)
LINEAR_DRAG_CONSTANT_X = -1/2 * SEAWATER_DENSITY * DRAG_COEFFICIENT * AUV_WIDTH * AUV_HEIGHT
LINEAR_DRAG_CONSTANT_Y = -1/2 * SEAWATER_DENSITY * DRAG_COEFFICIENT * AUV_LENGTH * AUV_HEIGHT
LINEAR_DRAG_CONSTANT_Z = -1/2 * SEAWATER_DENSITY * DRAG_COEFFICIENT * AUV_WIDTH * AUV_LENGTH
# FIXME: Verify drag torques
ROTATIONAL_DRAG_CONSTANT_X = -1/32 * SEAWATER_DENSITY * DRAG_COEFFICIENT * AUV_LENGTH * (AUV_WIDTH ** 3 + AUV_HEIGHT ** 3)
ROTATIONAL_DRAG_CONSTANT_Y = -1/32 * SEAWATER_DENSITY * DRAG_COEFFICIENT * AUV_WIDTH * (AUV_LENGTH ** 3 + AUV_HEIGHT ** 3)
ROTATIONAL_DRAG_CONSTANT_Z = -1/32 * SEAWATER_DENSITY * DRAG_COEFFICIENT * AUV_HEIGHT * (AUV_WIDTH ** 3 + AUV_LENGTH ** 3)


class AUVSim:
    Instance = None
    def __init__(self):
        AUVSim.Instance = self
        self.logger = Logger("AUVSimLog")
        self.selectedAUV = None
        self.setupGUI()
        self.auvs = []
        self.auvs.append(AUV([200,200], 0, "Orpheus", ["p2p", [[100, 100, -50], [100, 300, -50], [300, 300, -50], [300, 100, -50]]]))
        self.auvs.append(AUV([100,150], 2*math.pi/3, "Eurydice", ["p2p", [[200, 150, -50], [150, 236.6, -50], [250, 236.6, -50]]]))
        self.stepCount = 0
    
    def writeLog(self, message):
        if(self.logger == None):
            print("WARNING: logger has not been initialized")
            return
        self.logger.write(message)
        
    def setupGUI(self):
        # Root Window
        self.rootWindow = tk.Tk()
        
        # Main Display
        self.mainDisplay = tk.Canvas(self.rootWindow, width = 400, height = 400, bg=DISPLAY_COLOR)
        self.mainDisplay.grid(row = 0,column = 0)
        self.mainDisplay.bind("<Button-1>", self.mainDisplayButton1Handler)
        
        # Info Box
        self.infoFrame = tk.Frame(self.rootWindow)
        self.infoFrame.grid(row = 0, column = 1, sticky = "nsew")
        tk.Label(self.infoFrame, text="Info", width = 24).grid(row = 0, column = 0, columnspan = 2, sticky = "ew")
        self.infoBox1 = tk.Label(self.infoFrame)
        self.infoBox1.grid(row = 1, column = 0, sticky = "nsew")
        self.infoBox2 = tk.Label(self.infoFrame)
        self.infoBox2.grid(row = 1, column = 1, sticky = "nsew")
        self.updateInfoBox1()
        
        # Menubar
        self.menubar = tk.Menu(self.rootWindow)
        self.rootWindow.config(menu=self.menubar)
        fileMenu = tk.Menu(self.menubar)
        self.menubar.add_cascade(label="File",menu=fileMenu)
    
    def start(self):
        self.lastTime = time.time()
        self.rootWindow.after(LOOP_DELAY, self.loop)
        self.rootWindow.mainloop()
        
    def loop(self):
        newTime = time.time()
        timestep = newTime - self.lastTime
        self.updateSimulation(SIM_SPEED*timestep)
        self.redrawMainDisplay()
        self.updateInfoBox2()
        self.stepCount += 1
        self.lastTime = newTime
        self.rootWindow.after(LOOP_DELAY, self.loop)
        
    def updateInfoBox1(self):
        if(self.selectedAUV == None):
            self.infoBox1.config(text="")
        else:
            self.infoBox1.config(text="Name:\nLocation:\nDepth:\nHeading:\nVelocity:\nRotation Velocity:\nTarget Speed:\nTarget Heading:\nMotor 1:\nMotor 2:")
        
    def updateInfoBox2(self):
        auv = self.selectedAUV
        if(auv == None):
            self.infoBox2.config(text="")
        else:
            self.infoBox2.config(text=f"{auv.name}\n({auv.position[0,0]:.3f}, {auv.position[1,0]:.3f})\n{auv.position[2,0]:.3f}\n{180/math.pi*auv.orientation[2,0]:.3f}\n{auv.bodyLinearVelocity[0,0]:.3f}\n{180/math.pi*auv.bodyAngularVelocity[2,0]:.3f}\n{auv.targetVelocity[0,0]:.3f}\n{180/math.pi*auv.targetOrientation[2,0]:.3f}\n{auv.motorThrottle[0]*100:.1f}%\n{auv.motorThrottle[1]*100:.1f}%")
    
    def mainDisplayButton1Handler(self, event):
        minDis = SELECT_RADIUS ** 2
        closestAUV = None
        for auv in self.auvs:
            auvDis = (event.x - auv.position[0,0]) ** 2 + (event.y - auv.position[1,0]) ** 2
            if(auvDis < minDis):
                minDis = auvDis
                closestAUV = auv
        self.selectedAUV = closestAUV
        self.updateInfoBox1()
        self.updateInfoBox2()
    
    def redrawMainDisplay(self):
        self.mainDisplay.delete("auv")
        for auv in self.auvs:
            color = AUV_COLOR
            auvPosition = (auv.position[0,0], auv.position[1,0])
            if self.selectedAUV != None and auv.name == self.selectedAUV.name:
                color = AUV_SELECT_COLOR
                if(auv.autonomyMode == 1):
                    self.mainDisplay.create_line(auvPosition, [auvPosition[0] + POI_INDICATOR_LENGTH*np.cos(auv.targetOrientation[2,0]), auvPosition[1] + POI_INDICATOR_LENGTH*np.sin(auv.targetOrientation[2,0])], tag = "auv", fill = POI_COLOR)
                    self.mainDisplay.create_oval(self.selectedAUV.targetPosition[0,0] - POI_RADIUS, self.selectedAUV.targetPosition[1,0] - POI_RADIUS,\
                                                 self.selectedAUV.targetPosition[0,0] + POI_RADIUS, self.selectedAUV.targetPosition[1,0] + POI_RADIUS, fill = POI_COLOR, tag = "auv")
            self.mainDisplay.create_polygon(\
                    polarToCartesian(AUV_SHAPE[0], auv.orientation[2,0], auvPosition) + \
                    polarToCartesian(AUV_SHAPE[1], auv.orientation[2,0], auvPosition) + \
                    polarToCartesian(AUV_SHAPE[2], auv.orientation[2,0], auvPosition) + \
                    polarToCartesian(AUV_SHAPE[3], auv.orientation[2,0], auvPosition),\
                    tag = "auv", fill = color)
    
    def updateSimulation(self, timestep = 1):
        for auv in self.auvs:
            auv.updateSimulation(timestep)
        
class AUV:
    def __init__(self, position = [0,0], heading = 0, name = "AUV", autonomyInfo = [""]):
        if(AUVSim.Instance == None):
            print("ERROR: AUVSim must be instantiated before AUVs")
            return
        self.name = name
        AUVSim.Instance.writeLog(f"new AUV: {self.name}")
        self.initializeConstants()
        self.initializeKinematics(position, heading)
        self.setAutonomy(autonomyInfo)
    
    def setAutonomy(self, autonomyInfo = [""]):
        if(autonomyInfo[0] == "p2p"):
            self.autonomyMode = 1
            self.targets = np.array(autonomyInfo[1])
            self.targetIndex = 0
            self.targetPosition = np.transpose([self.targets[0]])
        elif(autonomyInfo[0] == "follow"):
            self.targetAUV = autonomyInfo[1]
            self.targetOffset = autonomyInfo[2]
            self.autonomyMode = 2
        else:
            self.autonomyMode = 0 # No autonomy
        
    def initializeConstants(self):
        self.constants = AUV.AUVConstants()
        
        self.constants.mass = AUV_MASS
        self.constants.momentsOfInertia = np.array([[AUV_MOMENT_X], [AUV_MOMENT_Y], [AUV_MOMENT_Z]])
        self.constants.dragConstants = np.array([[LINEAR_DRAG_CONSTANT_X, LINEAR_DRAG_CONSTANT_Y, LINEAR_DRAG_CONSTANT_Z],\
                                       [ROTATIONAL_DRAG_CONSTANT_X, ROTATIONAL_DRAG_CONSTANT_Y, ROTATIONAL_DRAG_CONSTANT_Z]]).T
        self.constants.maxMotorForces = np.array([[[MAX_MOTOR_FORCE, 0], [0,0], [0, -MAX_MOTOR_FORCE*AUV_WIDTH/2]],\
                                         [[MAX_MOTOR_FORCE, 0], [0,0], [0, MAX_MOTOR_FORCE*AUV_WIDTH/2]]])
        self.constants.throttleRanges = np.array([[MIN_MOTOR_THROTTLE, MAX_MOTOR_THROTTLE], [MIN_MOTOR_THROTTLE, MAX_MOTOR_THROTTLE]])
        
    def initializeKinematics(self, initPosition, initHeading):
        AUVSim.Instance.writeLog(f"Initialize {self.name} kinematics")
        self.position = np.array([[initPosition[0]], [initPosition[1]], [DEFAULT_DEPTH]])
        self.orientation = np.array([[0], [0], [initHeading]])
        self.inertialLinearVelocity = np.zeros([3,1])
        self.inertialAngularVelocity = np.zeros([3,1])
        self.bodyLinearVelocity = np.zeros([3,1])   # Probably don't need to track
        self.bodyAngularVelocity = np.zeros([3,1])  # Probably don't need to track
        self.controlsVelocityError = np.zeros([3,1])
        self.inertialVelocities = np.transpose(np.concatenate((self.inertialLinearVelocity, self.inertialAngularVelocity), axis=1))
        self.bodyVelocities = np.transpose(np.concatenate((self.bodyLinearVelocity, self.bodyAngularVelocity), axis=1))
    
    def updateSimulation(self, timestep = 1):
        self.updateAutonomy()
        self.updateControls()
        self.updateDynamics(timestep)
        self.updateKinematics(timestep)
    
    def updateAutonomy(self):
        # Uses target position to determine target velocity
        # If AUV reaches target, update to the next point in the list
        if(self.autonomyMode == 0): # No Autonomy
            self.targetOrientation = self.orientation
            self.targetVelocity = np.zeros([3,1])
            return
        if(self.autonomyMode == 1): # p2p
            positionError = self.targetPosition - self.position
            if(np.mean(np.square(positionError))<4):
                self.targetIndex = (self.targetIndex + 1) % len(self.targets)
                self.targetPosition = np.transpose([self.targets[self.targetIndex]])
                positionError = self.targetPosition - self.position
            self.targetOrientation = np.array([[0],[0],[np.arctan2(positionError[1,0], positionError[0,0])]])
            unitVectorOrientation = eulerAngleToUnitVector(self.orientation)
            self.targetVelocity = np.array([[np.clip(np.dot(unitVectorOrientation[:,0], positionError[:,0]),0,MAX_TARGET_SPEED)],[0],[0]])
            return
        if(self.autonomyMode == 2): # follow
            self.targetOrientation = self.orientation
            self.targetVelocity = np.zeros([3,1])
            return
        AUVSim.Instance.writeLog("ERROR: Unrecognized Autonomy Mode")
    
    def updateControls(self, timestep = 1):
        # Implementation of PD controller for now
        # TODO: implement actual motor controller
        # TODO: PID Controller
        # Update motor throttles
        # Uses target velocity of the AUV to determine motor power
        if(self.autonomyMode == 0):
            self.motorThrottle = np.zeros(2)
            return
        angP = 2
        angD = -5
        orientationError = self.targetOrientation - self.orientation
        velocityError = self.targetVelocity - self.bodyLinearVelocity
        necessaryRotation = orientationError[2,0] % (2*math.pi)
        angVelocity = self.bodyAngularVelocity[2,0]
        if(necessaryRotation < math.pi): # Turn Left
            differential = angP*necessaryRotation + angD*angVelocity
        else: # Turn Right
            differential = angP*(necessaryRotation-2*math.pi) + angD*angVelocity
        linP = 4
        linD = -0.25
        deltaAcceleration = (velocityError - self.controlsVelocityError)/timestep
        linear = linP*velocityError[0,0] + linD*deltaAcceleration[0,0]
        self.motorThrottle = np.array([linear-differential,linear+differential])
        self.motorThrottle = self.motorThrottle / np.max(np.abs([1, self.motorThrottle[0], self.motorThrottle[1]]))
        self.motorThrottle = np.clip(self.motorThrottle,self.constants.throttleRanges[:,0],self.constants.throttleRanges[:,1])
        self.controlsVelocityError = velocityError
    
    def updateDynamics(self, timestep = 1):
        # Based on https://www.researchgate.net/publication/253652041_UNDERWATER_VEHICLE_DYNAMIC_MODELING
        # We'll just start with a simple force sim for now
        # Update force and torques based on motor power
        force = np.zeros((3,2))
        throttle = np.clip(self.motorThrottle,self.constants.throttleRanges[:,0],self.constants.throttleRanges[:,1])
        for motorNum in range(len(throttle)):
            force = force + throttle[motorNum] * self.constants.maxMotorForces[motorNum]
        # Add drag to forces from controls
        drag = self.constants.dragConstants*np.concat([self.bodyLinearVelocity, self.bodyAngularVelocity*np.abs(self.bodyAngularVelocity)], axis=1)
        force = force + drag
        # Update velocity
        self.bodyLinearVelocity = self.bodyLinearVelocity + force[:,[0]]/self.constants.mass*timestep
        self.bodyAngularVelocity = self.bodyAngularVelocity + force[:,[1]]/self.constants.momentsOfInertia*timestep
    
    def updateKinematics(self, timestep = 1):
        self.inertialLinearVelocity = np.matmul(linearFrameTransform(self.orientation), self.bodyLinearVelocity)
        self.inertialAngularVelocity = np.matmul(angularFrameTransform(self.orientation), self.bodyAngularVelocity)
        self.orientation = self.orientation + timestep * self.inertialAngularVelocity
        self.orientation[0,0] %= 2*math.pi
        self.orientation[1,0] %= 2*math.pi
        self.orientation[2,0] %= 2*math.pi
        self.position = self.position + timestep * self.inertialLinearVelocity

    class AUVConstants:
        def __init__(self, file = ""):
            pass
            
        
class Logger:
    def __init__(self, logName):
        self.logFile = open(f"{logName}_{datetime.now().strftime("%Y%m%dT%H%M%S")}", 'w')
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

simulator = AUVSim()
simulator.start()
