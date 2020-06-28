#!/usr/bin/env python
# Libraries
import rospy
import os
import SimpleHTTPServer
def kill():
	os.system("kill -KILL " + str(os.getpid()))

if __name__ == '__main__':
	os.chdir(os.path.dirname(__file__) + "/../contents")
	rospy.init_node("webserver")
	rospy.on_shutdown(kill)
	SimpleHTTPServer.test()
