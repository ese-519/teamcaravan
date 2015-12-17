#!/usr/bin/env python
from std_msgs.msg import String
import web
from hemo_map import *
import os
import json
import threading
from Queue import *

rootPath = 'REPLACE/ME'
contentPath = rootPath + 'static/app/'
        
urls = ( 
   '/', 'index',
   '/map', 'mapData',
   '/pos', 'botPos',
   '/req/(.+)', 'sendRequest',
   '/(css/.+)', 'resource',
   '/(js/.+)', 'resource',
   '/(bower_components/.+)', 'resource',
   '/(partials/.+)', 'resource',
   '/(favicon.ico)', 'resource'
)

app = web.application(urls, globals())

global m, destLoc, inTrip, jobQueue, curJob
inTrip = False
destLoc = 1
jobQueue = Queue()
m = Map(rootPath + "levine.mp")

class index:
    def GET(self):
      with open (contentPath + 'index.html') as myfile:
        data = myfile.read()
        return data

class botPos:
    global curLoc, destLoc, m

    def GET(self):
        global curLoc, destLoc, m, inTrip
        c = m.getLines()
        s = c[curLoc]

        nxtLoc = curLoc + 1
        if nxtLoc > len(m.hallways):
            nxtLoc = 1

        if (inTrip):
            d = c[nxtLoc]
        else:
            d = c[curLoc]
            
        res_dic = {}

        if (c != None):
            res_dic["x"] = s[0]
            res_dic["y"] = s[1]
            res_dic["x2"] = d[0]
            res_dic["y2"] = d[1]

        return json.dumps(res_dic, sort_keys=True)

class mapData:
    def GET(self):
        global m
    
        map_line_data = m.getLines()
        dic = {}

        idx = 1;

        res_dic = {}
        res_dic["size"] = str(map_line_data[0])

        for item in map_line_data[1:]:
            line = {  "x1" : str(item[0]),
                      "y1" : str(item[1]),
                      "x2" : str(item[2]),
                      "y2" : str(item[3])
            }

            dic[str(idx)] = line
            idx += 1
            
            res_dic["lines"] = dic

        return json.dumps(res_dic, sort_keys=True)

class sendRequest:
  global curLoc, destLoc, inTrip

  def POST(self, dest):
    global curLoc, destLoc, inTrip
    # print dest

    if (dest == 'q'):
        ROS_INFO("Terminating trip")
        endTrip()

    elif (int(dest) != curLoc and not inTrip):
        inTrip = True
        ROS_INFO(dest);
        startTrip(dest)

class resource:
  def GET(self, url):
    with open (contentPath + url) as myfile:
            data = myfile.read()
            return data

def tripCallback():
    global curLoc, destLoc, jobQueue, curJob, inTrip, m

    if (ser.inWaiting() > 0):
      data = ser.read(1)
      if (data == globes.ACK_MSG):
        print "ACK"

        if curJob.type == 'f':
            curLoc = curLoc + 1
            if curLoc > len(m.hallways):
                curLoc = 1

        if (jobQueue.empty()):
            endTrip()
        else:
            curJob = jobQueue.get()
      else:
        print "Not ACK", data

    curJob.send()

    if (curJob.type == 's'):
        callbackDelay = 3
    else:
        callbackDelay = 0.1

    if (inTrip):
        threading.Timer(callbackDelay, tripCallback).start()

def startTrip(dest):
    global jobQueue, curJob, inTrip, destLoc, curDeg, curLoc
    
    while (not jobQueue.empty()):
        jobQueue.get()

    curLoc = 1
    m = Map(rootPath + "levine.mp")
    path = m.findPath(dest)

    doors = m.doors

    deg = curDeg
    loc = curLoc

    for p in path.turns:
        distance = p[0]
        hasDoor = False
        for d in doors:
            if (d[0] == loc):
                hasDoor = True
                distance = d[1]

        if hasDoor:
            jobQueue.put(Job("d", distance))
            jobQueue.put(Job("f", p[0] - distance))
        else:
            jobQueue.put(Job("f", distance))

        jobQueue.put(Job("l", p[1] - deg))
        deg = p[1]

        loc += 1
        if (loc > len(m.hallways)):
            loc = 1

        if loc == dest:
            jobQueue.put(Job("s", 0))

    jobQueue.put(Job("b", 0))

    curJob = jobQueue.get()
    tripCallback()
    inTrip = True
    destLoc = int(dest)

def endTrip():
    global inTrip, curLoc, destLoc
    inTrip = False
    destLoc = curLoc
    brake()

def ROS_INFO(s):
    print s

def start():
    app.run()

if __name__ == "__main__":
    start()