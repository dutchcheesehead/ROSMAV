"""This module provides a class that allows for (slightly) easier dynamic access to ROS"""

import roslib; roslib.load_manifest('rospy')
import rospy
roslib.load_manifest('rosservice')
import rosservice
import re 

from base64 import standard_b64encode
from types import TypeType

#fuerte/electric decisions
importedTime = False

try:
	import roslib.rostime
	from roslib.rostime import Time
	from roslib.rostime import Duration
	importedTime = True
except ImportError, ign:
	pass

try:
	if not importedTime:
		import rospy.rostime
		from rospy.rostime import Time
		from rospy.rostime import Duration
		importedTime = True
except ImportError, ign:
	pass

if not importedTime:
	raise ImportError

class ROSProxy(object):
	def __init__(self):
		self.mans = {}
		self.mods = {}

	def __GetTopics(self):
		return [x[0] for x in rospy.get_published_topics()]
	topics = property(fget=__GetTopics)

	def __GetServices(self):
		return rosservice.get_service_list()
	services = property(fget=__GetServices)

	def typeStringFromTopic(self, topic):
		try:
			return [x[1] for x in rospy.get_published_topics() if x[0] == topic][0]
		except:
			print "Can't find topic %s" % (topic,)
			return None

	def typeStringFromService(self, service):
		try:
			return rosservice.get_service_type(service)
		except:
			print "Can't find service %s" % (service,)
			return None

	def __classFromTypeString(self, typeString, subname):
		basemodule, itype = typeString.split('/')

		if not (basemodule in self.mans):
			try:
				roslib.load_manifest(basemodule)	
				self.mans[basemodule] = True;
			except:
				print "Can't find class for %s" % (typeString,)
				return None

		modname = basemodule + '.'+ subname + '._' + itype
		if not (modname in self.mods):
			try:
				mod = __import__(modname)
				self.mods['modname'] = mod
			except:
				return None

		mod = self.mods['modname']

		return getattr(getattr(getattr(mod,subname),'_' + itype), itype)

	def msgClassFromTypeString(self, typeString):
		return self.__classFromTypeString(typeString, 'msg')

	def srvClassFromTypeString(self, typeString):
		return self.__classFromTypeString(typeString, 'srv')

	def classFromTopic(self, topic):
		return self.msgClassFromTypeString(self.typeStringFromTopic(topic))

	def classFromService(self, service):
		return self.srvClassFromTypeString(self.typeStringFromService(service))

	def callService(self, service, arguments, callback=False, wait=True):
		def defCallback(x):
			pass

		if callback == False:
			callback = defCallback

		if wait:
			try:
				rospy.wait_for_service(service)
			except:
				callback(None)
				raise
		try:
			function = rospy.ServiceProxy(service, self.classFromService(service))
			if isinstance(arguments, list):
				response = function(*arguments)
			else:
				response = function(arguments)
			callback(response)
		except:
			callback(None)
			raise

	def generalize(self, inst):
		if hasattr(inst,'__slots__'):
			obj = {}
			for i in xrange(len(inst.__slots__)):
				field = inst.__slots__[i]
				if (hasattr(inst,'_slot_types') and inst._slot_types[i] == 'uint8[]'):
					obj[field] = self.generalize(standard_b64encode(getattr(inst,field)))
				else:
					obj[field] = self.generalize(getattr(inst,field))
			return obj
		elif isinstance(inst,tuple) or isinstance(inst,list):
			return [self.generalize(x) for x in inst]
		else:
			return inst

	braces = re.compile(r'\[[^\]]*\]') 
	atomics = ['bool', 'byte','int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64', 'float32', 'float64', 'string']

	def specify(self, typeStr, obj):
		if isinstance(typeStr,TypeType):
			cls = typeStr
		elif isinstance(typeStr,list):
			lst = []
			for i in xrange(len(typeStr)):
				lst.append(self.specify(typeStr[i],obj[i]))
			return lst
		elif typeStr != self.braces.sub('',typeStr):
			return [self.specify(self.braces.sub('',typeStr), x) for x in obj]
		elif typeStr in self.atomics:
			if typeStr == 'string':
				return obj.encode('ascii','ignore')
			return obj
		elif typeStr == 'time' or typeStr == 'duration':
			inst = None
			if typeStr == 'time':
				inst = Time()
			else:
				inst = Duration()
			if 'nsecs' in obj and 'secs' in obj:
				inst.nsecs = obj['nsecs']
				inst.secs = obj['secs']
			else:
				inst = rospy.get_rostime()
			return inst
		else:
			if typeStr == 'Header':
				typeStr = 'std_msgs/Header'
			cls = self.msgClassFromTypeString(typeStr)

		if not hasattr(cls,'__call__'):
			return None
		inst = cls()
		for i in xrange(len(cls._slot_types)):
			field = cls.__slots__[i]
			typ = cls._slot_types[i]
			if field in obj:
				value = self.specify(typ,obj[field])
				if value != None:
					setattr(inst,field,value)
				else:
					print "Warning: passed object was only partially specified."
			elif typ=='time' or typ=='duration':
				setattr(inst, field, rospy.get_rostime())
			elif typ=='Header' or typ=='std_msgs/Header':
				inst.header.stamp = rospy.get_rostime()
		return inst
