#!/usr/bin/env python
import roslib; roslib.load_manifest('position_tracker')
import rospy

from position_tracker.msg import Position

import socket
from select import select

def processPosition(data):
	global position
	position = data
		
if __name__ == '__main__':
	rospy.init_node('position_tracker_web')

	position = Position()

	rospy.Subscriber("position", Position, processPosition)

	serverSocket = None
	inputReady = None

	try:
		serverSocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		port = rospy.get_param('/brown/position_tracker_web/port', "9100")
		port = int(port)

		serverSocket.bind(('',port))
		serverSocket.listen(5)
		incoming = [serverSocket]

		try:
			while True:
				inputReady,outputReady,excepted = select(incoming, [], [])

				incoming = [serverSocket]

				for input in inputReady:
					if (input == serverSocket):
						#new connection
						connection, address = serverSocket.accept()
						print "Connection from: %s:%s" % address
						incoming.append(connection)
					else:
						#processing socket
						data = input.recv(1)
						if data:
							#reply 
							ignore,outputReady,ignore2 = select([],[input],[])
							for output in outputReady:
								msg = "";
								msg += "[";
								msg += str(position.x) + ",";
								msg += str(position.y) + ",";
								msg += str(position.theta);
								msg += "]\n";

								output.send(msg)
							incoming.append(connection)

		except rospy.ROSInterruptException:
			pass
	except Exception, err:
		print err
	finally:
		if (inputReady != None):
			for input in inputReady:
				input.close()
		if (serverSocket != None):
			serverSocket.close()
		print "\n\n\nclosing..."
