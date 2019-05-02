#!/usr/bin/env python

import scipy.io
import numpy as np
import rospy

class CoveragePlanner:

    def __init__(self, dataMap, curr_pos):
        print "In the Init of coverage planner"
        #self.mat = scipy.io.loadmat(data_path)
        #self.latitude_mesh = self.mat.get('latMesh')
        #self.longitude_mesh = self.mat.get('lonMesh')
        self.sensor_data = dataMap
        self.curr_pos = [0,0]
        self.curr_pos[0] = int(curr_pos[0])
        self.curr_pos[1] = int(curr_pos[1])
        self.num_steps = 500
        print self.curr_pos

        ###SANDEEP
        #self.reward = self.sensor_data / np.amax(self.sensor_data)
        self.reward = self.sensor_data * 10
        self.actStr = (['Left','Up','Right','Down'])
        self.gamma = 0.9
        self.V_new=np.zeros(self.sensor_data.shape)
        self.Policy=np.zeros(self.sensor_data.shape,dtype=np.int)
        self.polStr=np.chararray(self.sensor_data.shape,itemsize=5)
        self.polStr[:]=' '
        self.eps = 0.00001
    
    def valueIteration(self):
        k = 0
        exit = 0
        V_old = np.zeros(self.sensor_data.shape)
        while (exit == 0):
            k = k+1
            V_old = np.copy(self.V_new)
            exit = 1
            for i in range(0, self.sensor_data.shape[0]):
                for j in range(0, self.sensor_data.shape[1]):
                    summations = np.array([0,0,0,0])
                    summations = self.computeSumActions(V_old,i,j)
                    self.V_new[i,j] = summations.max()
                    self.Policy[i,j] = summations.argmax()
                    self.polStr[i,j] = self.actStr[self.Policy[i,j]]
                    if (abs(self.V_new[i][j] - V_old[i][j]) > self.eps):
                        exit = 0;
    
    def computeSumActions(self, V_old, i, j):
        summations = np.array([0,0,0,0])

        #hFactor - Horizontal winds E to W or W to E
        #vFactor - Vertical winds N to S or S to N
        #Wind Direction(Deg) : Wind Speed(kmph) : Wave Direction(Deg)

        windDirection = 52;
        windSpeed = 0; # currently Not using wind direction or speed
        waveDirection = 14;
        wSpeed = windSpeed/300000;
        hFactor = np.sin(np.deg2rad(windDirection))*10;
        vFactor = np.cos(np.deg2rad(windDirection))*10;

        #Need to go to places with high rewards.
        #Boundary check. If out of statespace, send low values so that the robot doesn't exit the boundary
        if(j-1 <= -1):
            summations[0] = -5
        else:
            summations[0] = self.reward[i,j] + self.gamma*V_old[i,j-1] + wSpeed*vFactor

        if(i-1 <= -1):
            summations[1] = -5
        else:
            summations[1] = self.reward[i,j] + self.gamma*V_old[i-1,j] + wSpeed*hFactor
        
        if(j+1 >= self.reward.shape[1]):
            summations[2] = -5
        else:
            summations[2] = self.reward[i,j] + self.gamma*V_old[i,j+1] + wSpeed*vFactor

        if(i+1 >= self.reward.shape[0]):
            summations[3] = -5
        else:
            summations[3] = self.reward[i,j] + self.gamma*V_old[i+1,j] + wSpeed*hFactor

        return summations

    def mdpPolicySearch(self):
        print "Planning the trajectory"
        self.valueIteration()
        #curState = np.array([np.floor(self.sensor_data.shape[0]/2),np.floor(self.sensor_data.shape[1]/2)],dtype=np.int)
        curState = self.curr_pos 
        nextState = np.copy(curState)
        stateSeqs = np.array([curState],dtype=np.int)
        prevAction = 0;
        for i in range(0,self.num_steps):
            #print stateSeqs
            #print curState
            print i;
            curAction = self.Policy[curState[0]][curState[1]]
            ###We want to add into the stateSeq only if the action has changed
            if curAction != prevAction:
                stateSeqs = np.append(stateSeqs,[curState],axis=0)
            prevAction = curAction
            if (curAction == 0):
                if (curState[1]-1 > -1):
                    nextState[1] = curState[1]-1
            elif (curAction == 1):
                                if (curState[0]-1 > -1):
                                        nextState[0] = curState[0]-1
            elif (curAction == 2):
                                if (curState[1]+1 < self.sensor_data.shape[1]):
                                        nextState[1] = curState[1]+1
            elif (curAction == 3):
                                if (curState[0]+1 < self.sensor_data.shape[0]):
                                        nextState[0] = curState[0]+1
            
            self.reward[curState[0]][curState[1]] = -5
            curState = np.copy(nextState)
            ###We want to add into the stateSeq only if the action has changed
            ###stateSeqs = np.append(stateSeqs,[curState],axis=0)
            self.valueIteration()
        stateSeqs = np.append(stateSeqs,[curState],axis=0)
        print stateSeqs
        return (stateSeqs, self.reward)

#if __name__ == '__main__':
#    data_path = '/home/sandeep/workspaces/sel_coverage_ws/TrialData/chlorophyll.mat'
#    covPlanner = CoveragePlanner(data_path)
#    stateSeqs = covPlanner.mdpPolicySearch()
#    print stateSeqs
