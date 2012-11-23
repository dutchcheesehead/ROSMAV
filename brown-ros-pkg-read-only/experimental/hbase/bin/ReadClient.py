#!/usr/bin/env python
import sys
from sys import argv,exit
import roslib; roslib.load_manifest('hbase')
import rospy
 
from thrift import Thrift
from thrift.transport import TSocket
from thrift.transport import TTransport
from thrift.protocol import TBinaryProtocol
 
from hbase import Hbase
from hbase.ttypes import *

msgs = dict()

class ReadClient:
	# should session_name be user-defined? or just use timestamp?
	def __init__(self, session_id):
		host = '192.168.0.1'
		# default thrift port is 9090
		port = 9091

		# Make socket
		transport = TSocket.TSocket(host, port)
		# Buffering is critical. Raw sockets are very slow
		transport = TTransport.TBufferedTransport(transport)
		# Wrap in a protocol
		protocol = TBinaryProtocol.TBinaryProtocol(transport)

		self.hbaseclient = Hbase.Client(protocol)
		transport.open()

		self.session_id = session_id
		self.topictype_dict = self.getTopic2TypeDict()
		self.topicchecksum_dict = self.getTopic2ChecksumDict()


	def readAll(self):
		# hbase> get "session_table", "row_key", "column_name"		
		result = self.hbaseclient.get("session_table", "0", "timestamp:"+self.session_id)
		num_entries = result[0].value

		for i in range(1, int(num_entries)+1):			
			row_time = self.hbaseclient.get("session_table", "%d"%i, "timestamp:"+self.session_id)
			timestamp = row_time[0].value
			print "timestamp: " + timestamp

			row_topic = self.hbaseclient.get("session_table", "%d"%i, "topic:"+self.session_id)
			topic = row_topic[0].value
			print "topic: " + topic

			row_msg = self.hbaseclient.get("session_table", "%d"%i, "msg:"+self.session_id)
			msg_value = row_msg[0].value

			msg_type = self.topictype_dict[topic]
			print "msg type: " + msg_type

			if(msg_type not in msgs):
				m = msg_type.split('.')
				roslib.load_manifest(m[0])
 				_tmp = __import__(m[0]+'.msg',globals(), locals(), [m[-1]])
				msgs[msg_type] = getattr(_tmp,m[-1])
				
			obj = msgs.get(msg_type)()
			obj.deserialize(msg_value)
			print "msg:\n",obj, "\n"

	def getTopic2TypeDict(self):
		# hbase> get "session_table", "0", "topic:session_id"		
		result = self.hbaseclient.get("session_table", "0", "topic:"+self.session_id)
		topics = result[0].value
		topic_list = topics[0:len(topics)-1].split("|")
		topic_dict = {}
		for i in range(0, len(topic_list)):
			topic = topic_list[i].split(":")[0]			
			msg_type = topic_list[i].split(":")[1]			
			topic_dict[topic] = msg_type			
		return topic_dict

	def getTopic2ChecksumDict(self):
		# hbase> get "session_table", "0", "topic:session_id"		
		result = self.hbaseclient.get("session_table", "0", "topic:"+self.session_id)
		topics = result[0].value
		topic_list = topics[0:len(topics)-1].split("|")
		topic_dict = {}
		for i in range(0, len(topic_list)):
			topic = topic_list[i].split(":")[0]			
			checksum = topic_list[i].split(":")[2]			
			topic_dict[topic] = checksum			
		return topic_dict


	def getAllTimestamps(self):
		result = self.hbaseclient.get("session_table", "0", "timestamp:"+self.session_id)
		num_entries = result[0].value

		timestamps = []

		for i in range(1, int(num_entries)+1):			
			row_time = self.hbaseclient.get("session_table", "%d"%i, "timestamp:"+self.session_id)
			time_entry = row_time[0].value
			timestamps.append(time_entry)			
		return timestamps


if __name__ == '__main__':
	if (len(argv) < 2):
		print "Must provide session id"
		exit(-1)

	session1 = ReadClient(argv[1])
#	session1.readAll()

	topictype_dict = session1.getTopic2TypeDict()
	print topictype_dict

	topicchecksum_dict = session1.getTopic2ChecksumDict()
	print topicchecksum_dict

	timestamps = session1.getAllTimestamps()
	print timestamps

