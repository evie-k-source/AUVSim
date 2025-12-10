import numpy as np
import json
from AUVUtilities import *

MAX_TARGET_SPEED = 0.5 # m/s
SEAWATER_DENSITY = 1030 # kg/m3
FOLLOWER_MAX_DEVIATION = 0.25 # percentage of expected distance

class AUV:
    def __init__(self, logger, position = [0,0,-50], heading = 0, name = "AUV", constants = None, autonomyInfo = [""]):
        self.logger = logger
        self.name = name
        self.writeLog(f"new AUV: {self.name}")
        if constants == None:
            self.constants = AUVConstants()
        else:
            self.constants = constants
        self.initializeKinematics(position, heading)
        self.setAutonomy(autonomyInfo)
    
    def writeLog(self, message):
        if(self.logger == None):
            print("WARNING: logger has not been initialized")
            return
        self.logger.write(message)
    
    def setAutonomy(self, autonomyInfo = {"type": ""}):
        if(autonomyInfo["type"] == "p2p"):
            self.autonomyMode = 1
            self.targets = np.array(autonomyInfo["targets"])
            self.targetIndex = 0
            self.targetPosition = np.transpose([self.targets[0]])
        elif(autonomyInfo["type"] == "lawnmower"):
            self.autonomyMode = 1
            targets = []
            widthSum = 0
            pathOrientation = autonomyInfo["orientation"]
            if autonomyInfo["leftTurn"]:
                pathShift = [autonomyInfo["pathSpan"]*np.cos(pathOrientation - np.pi/2), autonomyInfo["pathSpan"]*np.sin(pathOrientation - np.pi/2)]
            else:
                pathShift = [autonomyInfo["pathSpan"]*np.cos(pathOrientation + np.pi/2), autonomyInfo["pathSpan"]*np.sin(pathOrientation + np.pi/2)]
            lineStart = autonomyInfo["start"]
            while widthSum < autonomyInfo["totalScanWidth"]:
                targets.append([lineStart[0], lineStart[1], autonomyInfo["depth"]])
                targets.append([lineStart[0] + autonomyInfo["pathLength"] * np.cos(pathOrientation), lineStart[1] + autonomyInfo["pathLength"] * np.sin(pathOrientation), autonomyInfo["depth"]])
                lineStart = [targets[-1][0] + pathShift[0], targets[-1][1] + pathShift[1]]
                pathOrientation = pathOrientation + np.pi
                widthSum += autonomyInfo["pathSpan"]
            self.targets = np.array(targets)
            self.targetIndex = 0
            self.targetPosition = np.transpose([self.targets[0]])
        elif(autonomyInfo["type"] == "follow"):
            self.targetAUVName = autonomyInfo["target"]
            self.targetOffset = np.transpose([autonomyInfo["offset"]])
            self.targets = np.array([])
            self.followingDeviation = 0
            self.autonomyMode = 2
        else:
            self.autonomyMode = 0 # No autonomy
            self.targets = np.array([])
        
        if "followers" in autonomyInfo:
            self.followers = autonomyInfo["followers"]
        else:
            self.followers = []
        
    def initializeKinematics(self, initPosition, initOrientation):
        self.writeLog(f"Initialize {self.name} kinematics")
        self.position = np.transpose([initPosition])
        self.orientation = np.transpose([initOrientation])
        self.inertialLinearVelocity = np.zeros([3,1])
        self.inertialAngularVelocity = np.zeros([3,1])
        self.bodyLinearVelocity = np.zeros([3,1])
        self.bodyAngularVelocity = np.zeros([3,1])
        self.controlsVelocityError = np.zeros([3,1])
        self.inertialVelocities = np.transpose(np.concatenate((self.inertialLinearVelocity, self.inertialAngularVelocity), axis=1))
        self.bodyVelocities = np.transpose(np.concatenate((self.bodyLinearVelocity, self.bodyAngularVelocity), axis=1))
    
    def updateSimulation(self, timestep = 1):
        self.updateAutonomy()
        self.updateControls()
        self.updateDynamics(timestep)
        self.updateKinematics(timestep)
    
    def updateSensors(self, timestep = 1, auvDict = {}):
        # FIXME: Assumes perfect sensors for the following
        self.sensorAUVs = dict((name, auv) for name, auv in auvDict.items() if name != self.name)
        self.sensorOrientation = np.copy(self.orientation)
        self.sensorLinearVelocity = np.copy(self.bodyLinearVelocity)
        self.sensorAngularVelocity = np.copy(self.bodyAngularVelocity)
        self.sensorPosition = np.copy(self.position)
    
    def updateAutonomy(self):
        # Uses target position to determine target velocity
        # If AUV reaches target, update to the next point in the list
        if(self.autonomyMode == 0): # No Autonomy
            self.targetOrientation = self.sensorOrientation
            self.targetVelocity = np.zeros([3,1])
            return
        if(self.autonomyMode == 1): # p2p
        # Check followers Deviation
            maxFollowerDeviation = 0
            for followerName in self.followers:
                if followerName in self.sensorAUVs:
                    maxFollowerDeviation = max(maxFollowerDeviation, self.sensorAUVs[followerName].followingDeviation)
            adjustedMaxSpeed = MAX_TARGET_SPEED * (1 - min(maxFollowerDeviation, FOLLOWER_MAX_DEVIATION)/FOLLOWER_MAX_DEVIATION)
            
            positionError = self.targetPosition - self.sensorPosition
            if(np.mean(np.square(positionError))<4):
                self.targetIndex = (self.targetIndex + 1) % len(self.targets)
                self.targetPosition = np.transpose([self.targets[self.targetIndex]])
                positionError = self.targetPosition - self.sensorPosition
            self.targetOrientation = np.array([[0],[0],[np.arctan2(positionError[1,0], positionError[0,0])]])
            unitVectorOrientation = eulerAngleToUnitVector(self.sensorOrientation)
            self.targetVelocity = np.array([[np.clip(np.dot(unitVectorOrientation[:,0], positionError[:,0]),0,adjustedMaxSpeed)],[0],[0]])
            return
        if(self.autonomyMode == 2): # follow
            if self.targetAUVName in self.sensorAUVs:
                targetAUV = self.sensorAUVs[self.targetAUVName]
                self.targetPosition = targetAUV.sensorPosition + np.matmul(linearFrameTransform(targetAUV.sensorOrientation), self.targetOffset)
                positionError = self.targetPosition - self.sensorPosition
                self.targetOrientation = np.array([[0],[0],[np.arctan2(positionError[1,0], positionError[0,0])]])
                unitVectorOrientation = eulerAngleToUnitVector(self.sensorOrientation)
                self.targetVelocity = np.array([[np.clip(np.dot(unitVectorOrientation[:,0], positionError[:,0]),0,MAX_TARGET_SPEED)],[0],[0]])
                self.followingDeviation = np.sqrt(np.sum(np.square(positionError))/np.sum(np.square(self.targetOffset)))
            else:
                self.targetOrientation = self.sensorOrientation
                self.targetVelocity = np.zeros([3,1])
                self.followingDeviation = 0
            return
        self.writeLog("ERROR: Unrecognized Autonomy Mode")
    
    def updateControls(self, timestep = 1):
        # Implementation of PD controller
        # TODO: implement actual motor controller
        # Update motor throttles
        # Uses target velocity of the AUV to determine motor power
        if(self.autonomyMode == 0):
            self.motorThrottle = np.zeros(2)
            return
        angP = 2
        angD = -5
        orientationError = self.targetOrientation - self.sensorOrientation
        velocityError = self.targetVelocity - self.sensorLinearVelocity
        necessaryRotation = orientationError[2,0] % (2*math.pi)
        angVelocity = self.sensorAngularVelocity[2,0]
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
        # Update force and torques based on motor power
        force = np.zeros((3,2))
        throttle = np.clip(self.motorThrottle,self.constants.throttleRanges[:,0],self.constants.throttleRanges[:,1])
        for motorNum in range(len(throttle)):
            force = force + throttle[motorNum] * self.constants.maxMotorForces[motorNum]
        # Add drag to forces from controls
        drag = self.constants.dragConstants*np.concatenate([self.bodyLinearVelocity, self.bodyAngularVelocity*np.abs(self.bodyAngularVelocity)], axis=1)
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
        def __init__(self, logger, filename = ""):
            self.logger = logger
            if(filename == ""):
                self.modelName = ""
                self.mass = 1
                self.dimensions = np.ones((1,3))
                self.momentsOfInertia = np.ones((3,1))
                self.dragConstants = -np.ones((3,2))
                self.maxMotorForces = np.array([[]])
                self.throttleRanges = np.array([[]])
                return
            
            self.writeLog(f"Loading AUV Model: {filename}")
            auvFile = open(filename, 'r')
            data = json.load(auvFile)
            auvFile.close()
            
            self.modelName = data["model"]
            
            physics = data["physicsConstants"]
            self.mass = physics["mass"]
            self.dimensions = np.array([physics["dimensions"]])
            self.momentsOfInertia = np.array([physics["momentsOfInertia"]]).T
            self.dragConstants = np.array([physics["linearDrag"], physics["rotationalDrag"]]).T

            motors = data["controls"]["motors"]
            motorThrottles = []
            motorForces = []
            for motor in motors:
                motorThrottles.append(motor["throttleRange"])
                motorForces.append(np.array([motor["maxForce"], motor["maxTorque"]]).T)
            self.maxMotorForces = np.array(motorForces)
            self.throttleRanges = np.array(motorThrottles)
            
            self.writeLog(f"Finished loading {filename}")
        
        def writeLog(self, message):
            if(self.logger == None):
                print("WARNING: logger has not been initialized")
                return
            self.logger.write(message)
            
