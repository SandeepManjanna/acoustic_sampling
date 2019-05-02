#!/usr/bin/python2.7

"soundmap_exploration.py: read the measurements from the soundMap app, build an acoustic map and guide the robot"

import random
import os
import shutil
import sys
import traceback
import itertools
import time
import numpy as np
from sympy import Polygon, Point, Symbol, Segment
import ffmpy
import soundfile as sf

from matplotlib.ticker import FormatStrFormatter
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from matplotlib import cm

from gps_tools import *
from coverage_planner_8actions import CoveragePlanner
import world

class Measurement(object):
    def __init__(self, latitude, longitude, data):
        self.latitude = latitude
        self.longitude = longitude
        self.data = data
        self.time = time.time()

    def __str__(self):
        return str(self.latitude)+":"+str(self.longitude)+":"+str(self.data)+":"+str(self.time)

class Exploration(object):
    def __init__(self, latitude, longitude, width, height, spacing, orientation):
        self.world = world.World(latitude, longitude, width, height, spacing, orientation)
        self.curr_time = 0
        self.timeStamp = "../../../logger_"+str(int(time.time()))
        self.fieldMap = []
        self.gpMap = []
        self.fieldMap = []
        self.occupancyGrid = np.ones((self.world.height,self.world.width))

        print "Done with exploration constructor"

    def plotMean(self):
        Y = numpy.zeros(len(self.world.grid[0]))
        X = numpy.zeros(len(self.world.grid))
        print X.shape
        print Y.shape
        Z = self.gpMap
        V = self.cur_variance
        plt.figure()
        plt.contourf(X, Y, Z)
        plt.colorbar()
        C = plt.contour(X, Y, Z, 8, colors='black', linewidth=.5)
        plt.gca().yaxis.set_major_formatter(FormatStrFormatter('%.8f'))
        plt.gca().xaxis.set_major_formatter(FormatStrFormatter('%.8f'))
        plt.figure()
        plt.contourf(X, Y, V)
        plt.colorbar()
        C = plt.contour(X, Y, V, 8, colors='black', linewidth=.5)
        plt.gca().yaxis.set_major_formatter(FormatStrFormatter('%.8f'))
        plt.gca().xaxis.set_major_formatter(FormatStrFormatter('%.8f'))
        plt.show() 

    def measurement_received(self):
        #gps_location = [data.latitude, data.longitude]
        #print gps_location
        #self.world.add_measurement(data.time, gps_location, data.data)
        print "Added world measurement"
        up = self.world.update()

        print "Updated the GP map"
        print up
        #print self.world.cell_locations
        try:
            locations, mean, variances = self.world.predict(self.world.cell_locations)
            print "predict the variance and mean"
        except:
            traceback.print_exc(file=sys.stdout)
            print "Exception while predicting from GP Model"

        #L = numpy.array(locations)
        #X_utm = L[:,0].reshape(self.world.width, self.world.height)
        #Y_utm = L[:,1].reshape(self.world.width, self.world.height)
        
        #SANDEEP:Do keep a check on x and y. Changed this from width height to height, width.
        self.gpMap = mean.reshape(self.world.height, self.world.width)
        #print self.cur_mean.shape
        self.cur_variance = variances.reshape(self.world.height, self.world.width)
        self.fieldMap = self.gpMap * self.occupancyGrid
        #self.fieldMap = self.fieldMap * 1000
        #print self.gpMap
        #print self.fieldMap
        #Save the fieldmaps into a text file
        with open(self.timeStamp+'_gpMap.txt','a') as f_handle:
            f_handle.write("TimeStamp : %s\n" % str(time.ctime()))
            np.savetxt(f_handle,self.gpMap, delimiter=',')
        #print self.cur_variance.shape
        #self.plotMean()
        #print "plot the map"
        #self.world.map_plot()

class Explorer(object):
    def __init__(self, path1, path2, path3, path4, path5, uname):
        self.srcPath = os.path.join(path1,uname)+"/"
        print "in Explorer constructor"
        self.destPath = os.path.join(path2,uname)+"/"
        self.latlonPath = os.path.join(path3,uname)+"/"
        self.othrlatlonPath = os.path.join(path4,uname)+"/"
        self.othrlatlonDestPath = os.path.join(path5,uname)+"/"
        if not os.path.exists(self.destPath):
            print "creating new destination directory"
            os.makedirs(self.destPath)
        if not os.path.exists(self.latlonPath):
            print "creating new latlon directory"
            os.makedirs(self.latlonPath)
        if not os.path.exists(self.othrlatlonPath):
            os.makedirs(self.othrlatlonPath)
        if not os.path.exists(self.othrlatlonDestPath):
            os.makedirs(self.othrlatlonDestPath)

        self.uname = uname
        self.othUsers = []
        self.rewardMap = []
        print "Done with explorer constructor"

    def convert_neighbors_xy(self, exploration):
        xy = []
        for i in self.othUsers:
            lat = float(i.split(",")[0])
            lon = float(i.split(",")[1])
            xy.append(exploration.world.cell_corresponding_to_gps(lat,lon))
        return xy

    def total_neighbor_dist(self, y, x):
        tot_dist = 0.0
        for i in self.neighborXY:
            tot_dist += np.sqrt((x-i[0])**2 + (y-i[1])**2)
        #print "Tot distance = "+str(tot_dist)
        if self.neighborXY == []:
            tot_dist = 1.0
        return tot_dist

    def distance_self(self, y, x):
        #print "self distance ="+str(np.sqrt((x-self.xy_location[0])**2 + (y-self.xy_location[1])**2))
        if self.xy_location[0] == x and self.xy_location[1] == y:
            return 1.0
        return np.sqrt((x-self.xy_location[0])**2 + (y-self.xy_location[1])**2)

    def set_reward_map(self, exploration):
        self.rewardMap = np.zeros((exploration.fieldMap.shape[0],exploration.fieldMap.shape[1]))
        self.neighborXY = self.convert_neighbors_xy(exploration)
        print "GPMAP :"
        print exploration.gpMap
        for i in range(0,self.rewardMap.shape[0]):
            for j in range(0,self.rewardMap.shape[1]):
                self.rewardMap[i][j] = exploration.gpMap[i][j] * self.total_neighbor_dist(i,j) *1000 / self.distance_self(i,j)
                self.rewardMap[i][j] = self.rewardMap[i][j] * exploration.occupancyGrid[i][j]
        #self.rewardMap[self.xy_location[1]][self.xy_location[0]] = -5
        print "REWARD MAP :"
        print self.rewardMap


    def process_sound_files(self,exploration):
        full_list = [os.path.join(self.srcPath,i) for i in os.listdir(self.srcPath)]
        if full_list == []:
            return False
        for f in sorted(full_list ,key=os.path.getmtime):
            fNm = f.split("/")[-1]
            lat = float(fNm.split("_")[1])
            lon = float(fNm.split("_")[2])
            ff = ffmpy.FFmpeg(inputs={f: None},outputs={'output.wav': None})
            ff.run()
            rms = [np.sqrt(np.mean(block**2)) for block in sf.blocks('output.wav', blocksize=1024000, overlap=512)][0]
            print rms
            rms = rms*1000
            os.remove('output.wav')
            self.data = Measurement(lat, lon, rms)
            self.xy_location = exploration.world.cell_corresponding_to_gps(lat,lon)
            gps_location = [self.data.latitude, self.data.longitude]
            print gps_location
            if not exploration.world.add_measurement(self.data.time, gps_location, self.data.data):
                print "The location is outside the Region, hence skipping this data point"
                os.remove(f)
                if os.path.isfile("output.wav"):
                    os.remove("output.wav")
                return False
            #exploration.measurement_received(self.data)
            xy = exploration.world.cell_corresponding_to_gps(self.data.latitude, self.data.longitude)
            #exploration.occupancyGrid[xy[1]][xy[0]] = -5
            os.rename(f,os.path.join(self.destPath,fNm))

            #Reading and storing latlon of other users
            if os.listdir(self.othrlatlonPath) != []:
                nfileNm = os.path.join(self.othrlatlonPath,os.listdir(self.othrlatlonPath)[0])
                destFileNm = os.path.join(self.othrlatlonDestPath,nfileNm.split("/")[-1])
                with open(nfileNm) as f:
                    mylist = f.read().splitlines()
                for i in mylist:
                    self.othUsers.append(i)
                os.rename(nfileNm,destFileNm)
        return True

    def set_nextlatlon(self,gps_point):
        #Send the lat and lon for next movement of the explorer.
        #Need to decide how to communicate
        self.next_lat = gps_point.latitude
        self.next_lon = gps_point.longitude
        fileNm = os.path.join(self.latlonPath,"latlon_"+str(int(time.time()))+".txt")
        f = open(fileNm, 'w')
        f.write(str(self.next_lat)+","+str(self.next_lon))
        f.close()

def main():
    filePath = "../../../soundMap/uploads/"
    destPath = "../../../soundMap/dest/"
    latlonPath = "../../../soundMap/latlon/"
    otherslatlonPath = "../../../soundMap/otherLatlon/"
    otherslatlonDestPath = "../../../soundMap/otherLatLonDest/"

    users = []
    explorers = []
    num_users = 2
    psf_response = []
    
    latitude, longitude = 45.5078, -73.56859 #MTL Jazz festival small
    width, height, spacing, orientation = 180, 330, 30.0, -(numpy.pi/6)
    exploration = Exploration(latitude, longitude, width, height, spacing, orientation)

    #Adding initial Prior data points based on the stage locations:
    exploration.world.add_measurement(1234, [45.50834, -73.56757], 50.847738)
    exploration.world.add_measurement(1234, [45.50806, -73.56616], 800.657363)
    exploration.world.add_measurement(1234, [45.50777, -73.56815], 645.629844)
    exploration.world.add_measurement(1234, [45.50994, -73.56643], 603.987588)
    exploration.world.add_measurement(1234, [45.50928, -73.56588], 103.449844)
    exploration.world.add_measurement(1234, [45.50841, -73.56548], 321.987441)
    exploration.world.add_measurement(1234, [45.50736, -73.56693], 456.278464)

    print "Created Exploration object"

    #clear all old user directories
    for i in os.listdir(filePath):
        #os.rmdir(os.path.join(filePath,i))
        shutil.rmtree(os.path.join(filePath,i), ignore_errors=True)

    #wait for all users to login
    while len(os.listdir(filePath)) < num_users:
        print "Waiting for all "+str(num_users)+" users to login"
        time.sleep(5)
    users = os.listdir(filePath)
    
    #Create 1 explorer object for each user
    for i in range(0,len(users)):
        explorers.append(Explorer(filePath,destPath,latlonPath,otherslatlonPath,otherslatlonDestPath,users[i]))

    #latitude, longitude = 45.50754, -73.57252 #MTL Jazz festival
    #width, height, spacing, orientation = 550, 750, 50.0, -(numpy.pi/6)
    
    #explorer1 = Explorer(filePath,destPath,latlonPath,uname)
    print "Created all the Explorer objects"
    while(True):
        #print "In the iteration loop"
        psf_response=[]
        for i in range(0,len(users)):
            psf_response.append(explorers[i].process_sound_files(exploration))
            if psf_response[i] == True:

        #psf_response = explorer1.process_sound_files(exploration)
        #while psf_response == False:
        #    print "Waitiing for the sound file"
        #    time.sleep(10)
        #    psf_response = explorer1.process_sound_files(exploration)

                print "Done with processing the sound file and adding the measurement"
                exploration.measurement_received()
                print "Done with updating GP model and Predicting"
                print explorers[i].data
                explorers[i].set_reward_map(exploration)
                #covPlanner = CoveragePlanner(exploration.fieldMap, exploration.world.cell_corresponding_to_gps(explorers[i].data.latitude, explorers[i].data.longitude), 30)
                covPlanner = CoveragePlanner(explorers[i].rewardMap, exploration.world.cell_corresponding_to_gps(explorers[i].data.latitude, explorers[i].data.longitude), 1)
                currentMap = np.zeros((height,width))
                wayPtsYX, currentMap = covPlanner.mdpPolicySearch()
                print "Done with MDP value iteration"
                #SANDEEP: Wont be using currentMap as of now because we are using Variance as reward. This is the reason why we dont even need the occupancy grid map.
                #Setting the Occupancy Grid
                xy = exploration.world.cell_corresponding_to_gps(explorers[i].data.latitude, explorers[i].data.longitude)
                exploration.occupancyGrid[xy[1]][xy[0]] = -5
                exploration.occupancyGrid[wayPtsYX[-1,0]][wayPtsYX[-1,1]] = -5
                #Convert wayPtsYX into latlon
                print wayPtsYX
                gps_next = exploration.world.gps_corresponding_to_cell(wayPtsYX[-1,1],wayPtsYX[-1,0])
                print str(gps_next.latitude)+"_"+str(gps_next.longitude)
                explorers[i].set_nextlatlon(exploration.world.gps_corresponding_to_cell(wayPtsYX[-1,1],wayPtsYX[-1,0]))
            
            #else:
                #print "No sound file uploaded by User "+users[i]
if __name__ == '__main__':
    try:
        main()
    except:
        traceback.print_exc(file=sys.stdout)
        print "Exception in the main"
