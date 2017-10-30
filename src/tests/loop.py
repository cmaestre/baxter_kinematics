# -*- coding: utf-8 -*-
#!/usr/bin/env python

from __future__ import print_function

import rospy

import sys
#from gitagent.msg import *
import baxter_kinematics.msg
from threading import Lock
#import random
import roslib; roslib.load_manifest('baxter_kinematics')
import actionlib
import Queue
import action_server
import traceback
import time
#import threading

class Server:
    def __init__(self):
        rospy.init_node('server', anonymous=True)
        self.queueGoalHandles = Queue.Queue()
        self.server = action_server.MyActionServer('server', baxter_kinematics.msg.moveGoalAction, self.execute, False)
        self.server.start()
        self.keep_track_threads = []
        self.lock = Lock()

    #Needs to be used similarly as handle_serve, in order to block threads respectively.
    def execute(self, goalhandle):
        print ('I got a request')

        # Add tag to identify current thread
        # Task status: -1 ~ FAIL, 0 ~ PENDING, 1 ~ SUCCESS, 10 ~ no thread active
        index = -1

        self.lock.acquire()
        goal = goalhandle.get_goal()
        print ('extracted goal content: ' + str(goal.sender))
        self.keep_track_threads.append({'senderId':int(goal.sender), 'task_status':0})
        # It is assumed that the senderId is unique, that is the server cannot 
        # get 2 a second task from the same agent, without returning with the first.
        # {senderID:task_status}
        #self.keep_track_threads.append({data.sender:0})
        index = len(self.keep_track_threads) - 1
        print (self.keep_track_threads)
        # Put task in a queue, make thread wait until task status is set to success/fail 
        self.queueGoalHandles.put(goalhandle)
        self.lock.release()

        while self.keep_track_threads[index]['task_status'] == 0:
            print ('Task execution is pending')
            rospy.sleep(1)
        print ('Thread: ' + str(self.keep_track_threads[index]['task_status']) + ' has returned')
#        result.act_outcome = self.keep_track_threads[index]['task_status']
        result = self.keep_track_threads[index]['task_status']
        self.server.set_succeeded(goalhandle, result)
        self.lock.acquire()
        self.keep_track_threads[index]['task_status'] = 10
        self.lock.release()
        print (self.keep_track_threads)
        print ('result', result)

    def theLoop(self):
        print ('Server started')
        while not rospy.is_shutdown():
            #print 'Waiting for goal...'
            #Non-blocking
            try:
                item = self.queueGoalHandles.get(False)
                print (item)

                #Assuming we always accept the goal - set this goal as the current goal
                self.server.accept_new_goal(item)

                i = 0
                while i <= 10:
                    print (i)
                    i = i + 1
                    time.sleep(1)
                index = next(index for (index, d) in enumerate(self.keep_track_threads) if d['senderId'] == int(item.get_goal().sender))
                self.keep_track_threads[index]['task_status'] = 1
                #self.keep_track_threads[0]['task_status'] = 1
                print (self.keep_track_threads)
            except Queue.Empty:
                #print 'empty queue'
                pass

if __name__ == '__main__':

    try:
        serve = Server()
        serve.theLoop()
    except rospy.ROSInterruptException:
        traceback.print_exc()
    except (AttributeError, TypeError, ValueError, NameError):
        traceback.print_exc()
    except:
        print("Unexpected error:", sys.exc_info()[0])
        traceback.print_exc()