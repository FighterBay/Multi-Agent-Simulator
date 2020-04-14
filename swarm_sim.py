from __future__ import division
import PID
import pygame
from pygame.locals import *
import time
import math
import random

class agent:

    def __init__(self,x,y,id,threshold_distance):
        self.id = id  #Unique identifier among the robots
        self.x = x
        self.y = y

        self.tar_x = 0
        self.tar_y = 0
        #self.radius = 5

        self.weight = 0.5
        self.timestep = 0.1
        self.speed = 0.2
        self.pid = PID.PID(1,0,0.1)  #same PID for roll and pitch
        self.speed_x = 0
        self.speed_y = 0
        self.threshold_distance = threshold_distance



    def _weighted_func(self,dist):

        if dist <= self.threshold_distance:
            return -1 * self.weight * dist
        else:
            return self.weight * dist

    def compute_target_position(self,lstLocalNeighbors):

        tSum_x = self.x
        tSum_y = self.y

        for i in range(0,len(lstLocalNeighbors)):
            if(lstLocalNeighbors[i].id != self.id):
                diff_x = lstLocalNeighbors[i].x - self.x
                diff_y = lstLocalNeighbors[i].y - self.y
                tSum_x +=  self._weighted_func(abs(diff_x)) * diff_x
                tSum_y +=  self._weighted_func(abs(diff_y)) * diff_y

                #tSum_x += self.weight * (lstLocalNeighbors[i].x - self.x)      #Add the distance
                #tSum_y += self.weight * (lstLocalNeighbors[i].y - self.y)  #Add the distance

        self.tar_x = tSum_x
        self.tar_y = tSum_y




    def update_position(self):

        self.speed_x = self.pid.Compute(self.x, self.tar_x, self.timestep)
        self.speed_y = self.pid.Compute(self.y, self.tar_y, self.timestep)

        self.x = self.x + (self.speed_x * self.timestep)
        self.y = self.y + (self.speed_y * self.timestep)


def get_euclidean(agt1,agt2):
    return math.sqrt(((agt1.x - agt2.x)**2) + ((agt1.y - agt2.y)**2))


def get_neighbors(lstAgent,key_agt):        #key_gt with whose respect to we have to find neighbors
    lstLocalNeighbors = []
    beamwidth = 60    #consider a light house that throws a beam of light of the said beamwidth, the agent(ship) that is the closest in that beamwidth is taken into consideration rest are ignored for that particular beamwidth(segment).

    minDis = {}
    for agt in lstAgent:
        if (agt.id != key_agt.id):
            dt_x = key_agt.x - agt.x
            dt_y = agt.y - key_agt.y. #Y increases downwards


            segment = int(math.degrees(math.atan2(dt_y,dt_x))/beamwidth)
            str_segment = str(segment)
            str_segment_d = str_segment + "_d"  #sort of using DP to prevent recalculation of distances
            dist = get_euclidean(agt,key_agt)
            if str_segment in minDis and str_segment_d in minDis:
                if dist < minDis[str_segment_d]:
                    minDis[str_segment_d] = dist
                    minDis[str_segment] = agt
            else:
                minDis[str_segment_d] = dist
                minDis[str_segment] = agt


    for key in minDis:
        if not "_d" in key:
            lstLocalNeighbors.append(minDis[key])


    return lstLocalNeighbors


def render(lstAgent, disp, img):


    for agt in lstAgent:
        rect = img.get_rect(center=(agt.x,agt.y))
        disp.blit(img,rect)




pygame.init()
wid = 1000
ht = 800
DISPLAY=pygame.display.set_mode((wid,ht),0,32)
pygame.display.set_caption("Multi Agent")
black=(0,0,0)
DISPLAY.fill(black)
pygame.display.flip()
lstAgent = []
img = pygame.image.load('greenbox.bmp') #

for i in range(0,20):
    lstAgent.append(agent(random.randint(0,1000),random.randint(0,800),i,2))


while True:
    for agt in lstAgent:
        agt.compute_target_position(get_neighbors(lstAgent,agt))
        agt.update_position()

    DISPLAY.fill(black)
    render(lstAgent,DISPLAY,img)
    pygame.display.update()

    for event in pygame.event.get():
        if event.type==QUIT:
            pygame.quit()
            exit()
