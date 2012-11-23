import sys

import yaml
import re 

from time import sleep

#from rlglue.enivronment.Environment import Environment
from rlglue.environment import EnvironmentLoader
from rlglue.types import Observation
from rlglue.types import Action
from rlglue.types import Reward_observation_terminal

import roslib; roslib.load_manifest('rosglue')
import rospy

from ROSProxy import ROSProxy

from reward_termination import get_reward,check_termination_conditions

class ROSGlue(object):
	def __init__(self, problemSpec):
		self.problemSpec = problemSpec

		self.taskSpec = []

		self.latestObs = {}
		self.latestReward = 0
		self.latestTermination = False
		self.actionPubs = {}

		self.proxy = ROSProxy()

	def env_init(self):
		self.generateTaskSpec()
		self.envSetup()
		#debug
		#print self.taskSpec

		return self.taskSpec

	def envSetup(self):
		for name in self.problemSpec['actions']:
			info = self.problemSpec['actions'][name][0]
			if not info == 'service':
				self.actionPubs[name] = rospy.Publisher(name[1:],self.proxy.msgClassFromTypeString(info))

		for name in self.problemSpec['observations']:
			def handler(msg):
				self.handleMsg(name,msg)

			rospy.Subscriber(name, 
					self.proxy.classFromTopic(name), 
					handler, 
					queue_size=1)

		if self.problemSpec['reward']['type'] == 'glue':
			pass
		else:
			topic = [field for field in problemSpec['reward'] if not field == 'type' and not field == 'range'][0]
			field = [field for field in problemSpec['reward'][topic]][0]
			low = problemSpec['reward']['range'][0]
			high = problemSpec['reward']['range'][1]

			def handler(msg):
				val = getattr(msg, field)
				if (val < low):
					val = low
				if (val > high):
					val = high
				self.latestReward = val

			rospy.Subscriber(topic,
					 self.proxy.classFromTopic(topic),
					 handler,
					 queue_size=1)

		if self.problemSpec['termination']['type'] == 'glue':
			pass
		else:
			topic = [field for field in problemSpec['termination'] if not field == 'type'][0]
			field = problemSpec['termination'][topic][0]

			def handler(msg):
				val = getattr(msg, field)
				self.latestTermination = val

			rospy.Subscriber(topic,
					 self.proxy.classFromTopic(topic),
					 handler,
					 queue_size=1)

	def handleMsg(self, topic, msg):
		observation = {}
		for field in self.problemSpec['observations'][topic]:
			val = getattr(msg, field)

			#clip to min/max
			if val < self.problemSpec['observations'][topic][field][0]:
				val = self.problemSpec['observations'][topic][field][0]
			if val > self.problemSpec['observations'][topic][field][1]:
				val = self.problemSpec['observations'][topic][field][1]

			observation[field] = val

		self.latestObs[topic] = observation
	def generateTaskSpec(self):
		self.taskSpec = ['VERSION RL-Glue-3.0']

		self.taskSpec.append('PROBLEMTYPE')
		self.taskSpec.append(self.problemSpec['problemtype'])

		self.taskSpec.append('DISCOUNTFACTOR')
		self.taskSpec.append(self.problemSpec['discountfactor'])

		ints = ['INTS']
		doubles = ['DOUBLES']

		for observation in self.problemSpec['observations']:
			for field in self.problemSpec['observations'][observation]:
				elements = self.problemSpec['observations'][observation][field] 
				element = "(%s %s)" % (elements[0],elements[1])
				if isinstance(elements[0],int):
					ints.append(element)
				else:
					doubles.append(element)

		self.taskSpec.append('OBSERVATIONS')
		if len(ints) > 1:
			self.taskSpec.extend(ints)
		if len(doubles) > 1:
			self.taskSpec.extend(doubles)

		ints = ['INTS']
		doubles = ['DOUBLES']

		for action in self.problemSpec['actions']:
			for arg in self.problemSpec['actions'][action][1:]:
				for field in arg:
					elements = arg[field]
					element = "(%s %s)" % (elements[0],elements[1])
					if isinstance(elements[0],int):
						ints.append(element)
					else:
						doubles.append(element)

		self.taskSpec.append('ACTIONS')
		if len(ints) > 1:
			self.taskSpec.extend(ints)
		if len(doubles) > 1:
			self.taskSpec.extend(doubles)

		ints = ['INTS']
		doubles = ['DOUBLES']

		elements = self.problemSpec['reward']['range']
		element = "(%s %s)" % (elements[0],elements[1])
		if isinstance(elements[0],int):
			ints.append(element)
		else:
			doubles.append(element)

		self.taskSpec.append('REWARDS')
		if len(ints) > 1:
			self.taskSpec.extend(ints)
		if len(doubles) > 1:
			self.taskSpec.extend(doubles)

		self.taskSpec.append('EXTRA')
		self.taskSpec.append(self.problemSpec['extra'])

		self.taskSpec = [str(x) for x in self.taskSpec]
		self.taskSpec = ' '.join(self.taskSpec)

	def env_start(self):
		return self.observe()

	def env_step(self, thisAction):
		self.act(thisAction)
		return self.step()

	def act(self, thisAction):

		intIdx = 0
		doubleIdx = 0

		for action in self.problemSpec['actions']:
			if self.problemSpec['actions'][action][0] == 'service':
				args = []
				for arg in self.problemSpec['actions'][action][1:]:
					for field in arg:
						elements = arg[field]
						if isinstance(elements[0],int):
							args.append(thisAction.intArray[intIdx])
							intIdx = intIdx + 1
						else:
							args.append(thisAction.doubleArray[doubleIdx])
							doubleIdx = doubleIdx + 1
				self.proxy.callService(action,tuple(args))
			else:
				typeString = self.problemSpec['actions'][action][0]
				obj = proxy.msgClassFromTypeString(typeString)()
				for arg in self.problemSpec['actions'][action][1:]:
					for field in arg:
						elements = arg[field]
						if isinstance(elements[0],int):
							setattr(obj,field,intArray[intIdx])
							intIdx = intIdx + 1
						else:
							setattr(obj,field,doubleArray[doubleIdx])
							doubleIdx = doubleIdx + 1
				self.actionPubs[action].publish(obj)

	def observe(self):
		obs = Observation()

		for topic in self.problemSpec['observations']:
			for field in self.problemSpec['observations'][topic]:
				elements = self.problemSpec['observations'][topic][field] 

				#wait for observation
				while not topic in self.latestObs.keys():
					sleep(0.03)

				if isinstance(elements[0],int):
					obs.intArray.append(self.latestObs[topic][field])

				else:
					obs.doubleArray.append(self.latestObs[topic][field])

		return obs

	def step(self):
		rot = Reward_observation_terminal()

		rot.o = self.observe()

		if self.problemSpec['reward']['type'] == 'glue':
			rot.r = get_reward(self.latestObs)  
		else:
			rot.r = self.latestReward

		if self.problemSpec['termination']['type'] == 'glue':
			rot.terminal = check_termination_conditions(self.latestObs)
		else:
			rot.terminal = self.latestTermination

		return rot

	def env_cleanup(self):
		pass

	def env_message(self, inMessage):
		pass

if __name__ == "__main__":
	rospy.init_node('rosglue')

	filename = rospy.get_param('/brown/rosglue/envfile')
	problemSpec = yaml.load(open(filename))

	#debug
	#print problemSpec

	EnvironmentLoader.loadEnvironment(ROSGlue(problemSpec))

	#ROSGlue(problemSpec).env_init()
	rospy.spin()
