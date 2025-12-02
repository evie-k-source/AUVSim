import tkinter as tk
from tkinter import messagebox
import time
import numpy as np
import os
import sys
import json
from AUVUtilities import *
from AUV import AUV

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
MAX_ZOOM_LEVEL = 2
MIN_ZOOM_LEVEL = 0.25
ZOOM_RATE = 1.1


class AUVSim:
    Instance = None
    def __init__(self):
        AUVSim.Instance = self
        self.logger = Logger("AUVSimLog")
        self.loadAUVs()
        self.selectedAUV = None
        self.setupGUI()
        self.auvs = []
        self.auvs.append(AUV(self.logger, [200,200], 0, "Orpheus", self.auvModels["Orpheus"], ["p2p", [[100, 100, -50], [100, 300, -50], [300, 300, -50], [300, 100, -50]]]))
        self.auvs.append(AUV(self.logger, [100,150], 2*math.pi/3, "Eurydice", self.auvModels["Orpheus"], ["p2p", [[200, 150, -50], [150, 236.6, -50], [250, 236.6, -50]]]))
        self.stepCount = 0
    
    def writeLog(self, message):
        if(self.logger == None):
            print("WARNING: logger has not been initialized")
            return
        self.logger.write(message)

    def loadAUVs(self):
        try:
            AUVFilenames = os.listdir("auvs")
        except(FileNotFoundError):
            self.writeLog("ERROR: AUV directory not found")
            messagebox.showwarning("No AUV Directory", "AUV Models are missing")
        self.auvModels = {}
        for filename in AUVFilenames:
            if sys.platform == "win32":
                model = AUV.AUVConstants(self.logger, f"auvs\\{filename}")
            else:
                model = AUV.AUVConstants(self.logger, f"auvs/{filename}")
            self.auvModels[model.modelName] = model
    
    def setupGUI(self):
        # Root Window
        self.rootWindow = tk.Tk()
        
        # Main Display
        self.mainDisplay = tk.Canvas(self.rootWindow, width = 400, height = 400, bg=DISPLAY_COLOR)
        self.mainDisplay.grid(row = 0,column = 0)
        self.mainDisplay.bind("<Button-1>", self.mainDisplayButton1Handler)
        self.mainDisplay.bind("<Button-2>", self.startMovingMap)
        self.mainDisplay.bind("<ButtonRelease-2>", self.stopMovingMap)
        if sys.platform == "win32":
            self.mainDisplay.bind("<MouseWheel>", self.zoom)
        else:
            self.mainDisplay.bind("<Button-4>", self.zoomLinux)
            self.mainDisplay.bind("<Button-5>", self.zoomLinux)
        self.rootWindow.bind("<Control-c>", self.centerMovingMap)
        self.isMovingMap = False
        self.mapOffset = [0,0]
        self.zoomLevel = np.float64(1.0)
        
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
        if(self.isMovingMap):
            self.updateMovingMap()
        self.redrawMainDisplay()
        self.updateInfoBox2()
        self.stepCount += 1
        self.lastTime = newTime
        self.rootWindow.after(LOOP_DELAY, self.loop)
        
    def updateInfoBox1(self):
        if(self.selectedAUV == None):
            self.infoBox1.config(text="")
        else:
            self.infoBox1.config(text="Name:\nModel:\nLocation:\nDepth:\nHeading:\nVelocity:\nRotation Velocity:\nTarget Speed:\nTarget Heading:\nMotor 1:\nMotor 2:")
        
    def updateInfoBox2(self):
        auv = self.selectedAUV
        if(auv == None):
            self.infoBox2.config(text="")
        else:
            self.infoBox2.config(text=f"{auv.name}\n{auv.constants.modelName}\n({auv.position[0,0]:.3f}, {auv.position[1,0]:.3f})\n{auv.position[2,0]:.3f}\n{180/math.pi*auv.orientation[2,0]:.3f}\n{auv.bodyLinearVelocity[0,0]:.3f}\n{180/math.pi*auv.bodyAngularVelocity[2,0]:.3f}\n{auv.targetVelocity[0,0]:.3f}\n{180/math.pi*auv.targetOrientation[2,0]:.3f}\n{auv.motorThrottle[0]*100:.1f}%\n{auv.motorThrottle[1]*100:.1f}%")
    
    def mainDisplayButton1Handler(self, event):
        minDis = SELECT_RADIUS ** 2
        closestAUV = None
        for auv in self.auvs:
            auvPosition = ((auv.position[0,0] + self.mapOffset[0])*self.zoomLevel, (auv.position[1,0] + self.mapOffset[1])*self.zoomLevel)
            auvDis = (event.x - auvPosition[0]) ** 2 + (event.y - auvPosition[1]) ** 2
            if(auvDis < minDis):
                minDis = auvDis
                closestAUV = auv
        self.selectedAUV = closestAUV
        self.updateInfoBox1()
        self.updateInfoBox2()
    
    def startMovingMap(self, event):
        self.isMovingMap = True
        screen_x, screen_y = self.rootWindow.winfo_pointerxy()
        self.lastMouse = (screen_x, screen_y)

    def stopMovingMap(self, event):
        self.isMovingMap = False

    def updateMovingMap(self):
        screen_x, screen_y = self.rootWindow.winfo_pointerxy()
        self.mapOffset[0] += (screen_x - self.lastMouse[0])/self.zoomLevel
        self.mapOffset[1] += (screen_y - self.lastMouse[1])/self.zoomLevel
        self.lastMouse = (screen_x, screen_y)

    def zoom(self, event):
        #update zoom level
        newZoomLevel = self.zoomLevel * ZOOM_RATE ** (event.delta/120)
        newZoomLevel = max(MIN_ZOOM_LEVEL, min(MAX_ZOOM_LEVEL, newZoomLevel))
        #FIXME: update center of screen so that we're zooming towards the top left
        print(f"{event.x}, {event.y}")
        #self.mapOffset[0] += event.x/self.zoomLevel-
        #self.mapOffset[1] += event.y/self.zoomLevel*(self.zoomLevel-newZoomLevel)
        self.zoomLevel = newZoomLevel

    def zoomLinux(self, event):
        print(f"zoom {event.delta}")

    def centerMovingMap(self, event):
        self.mapOffset = [0,0]
        self.zoomLevel = np.float64(1.0)
    
    def redrawMainDisplay(self):
        self.mainDisplay.delete("auv")
        for auv in self.auvs:
            color = AUV_COLOR
            auvPosition = (auv.position[0,0] + self.mapOffset[0], auv.position[1,0] + self.mapOffset[1])
            if self.selectedAUV != None and auv.name == self.selectedAUV.name:
                color = AUV_SELECT_COLOR
                if(auv.autonomyMode == 1):
                    scaledTargetPosition = ((self.selectedAUV.targetPosition[0,0] + self.mapOffset[0])*self.zoomLevel, (self.selectedAUV.targetPosition[1,0] + self.mapOffset[1])*self.zoomLevel)
                    #self.mainDisplay.create_line(auvPosition, [auvPosition[0] + POI_INDICATOR_LENGTH*np.cos(auv.targetOrientation[2,0]), auvPosition[1] + POI_INDICATOR_LENGTH*np.sin(auv.targetOrientation[2,0])], tag = "auv", fill = POI_COLOR)
                    self.mainDisplay.create_oval(scaledTargetPosition[0] - POI_RADIUS, scaledTargetPosition[1] - POI_RADIUS,\
                                                 scaledTargetPosition[0] + POI_RADIUS, scaledTargetPosition[1] + POI_RADIUS, fill = POI_COLOR, tag = "auv")
            scaledPosition = (auvPosition[0] * self.zoomLevel, auvPosition[1] * self.zoomLevel)
            auvShapeScale = max(1,self.zoomLevel)
            self.mainDisplay.create_polygon(\
                    polarToCartesian(AUV_SHAPE[0], auv.orientation[2,0], scaledPosition, auvShapeScale) + \
                    polarToCartesian(AUV_SHAPE[1], auv.orientation[2,0], scaledPosition, auvShapeScale) + \
                    polarToCartesian(AUV_SHAPE[2], auv.orientation[2,0], scaledPosition, auvShapeScale) + \
                    polarToCartesian(AUV_SHAPE[3], auv.orientation[2,0], scaledPosition, auvShapeScale),\
                    tag = "auv", fill = color)
    
    def updateSimulation(self, timestep = 1):
        for auv in self.auvs:
            auv.updateSimulation(timestep)


if __name__ == "__main__":
    simulator = AUVSim()
    simulator.start()
