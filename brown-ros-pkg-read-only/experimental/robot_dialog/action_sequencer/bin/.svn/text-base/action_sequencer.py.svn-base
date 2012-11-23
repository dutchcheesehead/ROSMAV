#!/usr/bin/env python
import roslib; roslib.load_manifest('action_sequencer')
import rospy
from std_msgs.msg import String
import actionlib
import move.msg

import re

# ?? http://www.ros.org/wiki/msg says: In rospy, arrays are deserialized as tuples for performance reasons, but you can set fields to  tuples and lists interchangeably.
# does this mean if I have a "String[] data" in my msg, I can say "data = ('123','456')" ?


class ActionClass:
    def __init__(self,name):
        self.name = name
        self.actionClasses = [] # Each entry is a directed graph of other action classes.
        self.actionInstances = [] # Each entry is a single action instance.

class ActionInstance:
    def __init__(self,name,as_package,as_type,as_topic,as_actionType):
        self.name = name
        self.as_package = as_package
        self.as_type = as_type
        self.as_topic = as_topic
        self.as_actionType = as_actionType

###############################################################################
def buildAhm(filename):
    # TODO: we could do a multipass on this to actually bind the action instances with the action classes, instead of doing by-text references.  This can be done when scalability becomes an issue.
    # Surely there's a more elegant way to do what i'm trying to do with all the 'line = f.readline()'s ?
    ahm = []
    as_msg_modules = dict()

    f = open(filename,"r")
    line = f.readline()
    while True:
        if line == '': break
        if isCommentLine(line):
            line = f.readline()
        else:
            tokens = line.split()
            if tokens[0] == 'c': # Class line
                aclass = findActionClass(ahm,tokens[1])
                if not aclass: # New class found
                    rospy.loginfo("AHM: Added class "+tokens[1])
                    aclass = ActionClass(tokens[1])
                    ahm.append(aclass)
                if tokens[2] == 'c': # Action class contains list of action classes
                    rospy.loginfo("AHM: Added class set "+str(tokens[3:])+" to class "+tokens[1])
                    aclass.actionClasses.append(tokens[3:])
                    line = f.readline() # this is what's making everything messy with the readlines...
                    tokens = line.split()
                    while tokens[0] == '-':
                        # TODO: Build the action class graph here!
                        line = f.readline()
                        tokens = line.split()
                elif tokens[2] == 'i': # Action class contains list of action instances
                    rospy.loginfo("AHM: Added instances "+str(tokens[3:])+" to class "+tokens[1])
                    aclass.actionInstances.extend(tokens[3:])
                    line = f.readline()
                else:
                    rospy.logerr("Unknown line when building AHM: '"+line+"'.  Ignoring.")
                    line = f.readline()
                    #rospy.logwarn("Unknown line when building AHM: '"+line+"'")
            elif tokens[0] == 'i': # Instance line
                ainstance = findActionInstance(ahm,tokens[1])
                if not ainstance:
                    rospy.loginfo("AHM: Added instance "+tokens[1])
                    ainstance = apply(ActionInstance,tokens[1:])
                    ahm.append(ainstance)
                    as_msg_module_str = ainstance.as_package+'.msg'
                    # Import the msg type
                    if not as_msg_module_str in as_msg_modules:
                        rospy.loginfo("AHM: Importing module "+as_msg_module_str)
                        # Note that __import__('foo.msg') will return the module foo.  To get the
                        #   module foo.msg, we immediate use getattr.
                        as_msg_modules[as_msg_module_str] = getattr(__import__(as_msg_module_str), 'msg')
                else:
                    rospy.logwarn("We've already loaded action instance "+token[1]+"!  Ignoring.")
                line = f.readline()
    f.close()
    return (ahm, as_msg_modules)



# verb is a misnomer now.  it's really more of an "utterance", which is found by a regex.
def buildVerbDict(filename):
    verbDict = []
    f = open(filename,"r")
    for line in f:
        if not isCommentLine(line):
            [regex, action_class_name] = line.strip().split('|')
            rospy.loginfo("Loaded line from verb dictionary:\n"+
                          "    regex:             '"+regex+"'\n"+
                          "    action_class_name: '"+action_class_name+"'")
            verbDict.append((re.compile(regex, re.IGNORECASE), action_class_name))
    f.close()
    return verbDict

def parseCommand(command, ahm, verbDict, as_msg_modules):
    commandTokens = command.split()
    # Determine utterance from instruction
    ## Assume, for now, that each instruction has exactly one utterance. (TODO)
    # Determine action class from utterance
    (action_class_name, args) = findMatchingUtterance(command, verbDict)
    action_class = findActionClass(ahm, action_class_name)
    # Determine action instances from action class
    # potential ones: action instances here, or action classes which point to action instances
    #   TODO: We assume for now that we point directly at an action instance, and there's not DAG action class traversal.
    potential_action_instances = action_class.actionInstances
    if potential_action_instances:
        action_instance = findActionInstance(ahm, potential_action_instances[0])
        if action_instance:
            rospy.loginfo("Found action instance "+potential_action_instances[0]+" in AHM!")
        else:
            rospy.logerr("Cannot find action instance "+potential_action_instances[0]+" in AHM!") #todo: this shouldn't be fatal in the long run, since we can choose multiple ones...  plus, we'll run into this at ahm build time, so it might even just be a logwarn.

    else:
        rospy.logerr("Cannot find suitable action instance for action class "+action_class.name+"!")
    # Launch action instance
    as_msg_module = as_msg_modules[action_instance.as_package+'.msg']
    as_action = getattr(as_msg_module, action_instance.as_actionType+'Action')
    as_goal   = getattr(as_msg_module, action_instance.as_actionType+'Goal')
    client = actionlib.SimpleActionClient(action_instance.as_topic, as_action)
    client.wait_for_server()
    goal = as_goal()
    # TODO/WARNING: Next line is specific to move.msg.MoveGoal!  Do we need this specificity?  [This might tie into what Candy was saying about verb slots.]  Additionally, how do we reconcile this when do action class to action class communication?
    # NOTE: We can tack on attributes to any class, just like in javascript.  So this _probably_ won't break things (ie crashing), but it might give unexpected results.
    goal.command = list(args) #TODO we should eventually pass this through the AHM.  We may want to add/remove args at each level.  How do we do this?
    client.send_goal(goal)
    r = client.wait_for_result()
    return client.get_result()



###############################################################################
def findActionClass(ahm,name):
    # Linear search over all action classes.  Pick the first that matches in name.
    #   If none are found, return None.
    for node in ahm:
        if isinstance(node, ActionClass) and (node.name == name):
            return node
    return None

def findActionInstance(ahm,name):
    # Linear search over all action classes.  Pick the first that matches in name.
    #   If none are found, return None.
    for node in ahm:
        if isinstance(node, ActionInstance) and (node.name == name):
            return node
    return None

def findMatchingUtterance(command, regex_dict):
    for (compiled_regex, action_class_name) in regex_dict:
        result = compiled_regex.match(command)
        if result:
            rospy.loginfo("Matched command '"+command+"' to action class "+action_class_name)
            return (action_class_name, list(result.groups())) #todo: do we need list?
    return None

def isCommentLine(line):
    line = line.strip()
    return line=="" or line[0] == "#"

###############################################################################
def main():
    # Load regex dictionary from file, and import all message modules specified therein.
    (ahm, as_msg_modules) = buildAhm("action_sequencer_ahm.txt")
    verbDict = buildVerbDict("action_sequencer_verbs.txt")

    # Start ROS node
    rospy.init_node('action_sequencer')
    rospy.Subscriber('command_string', String, lambda string: parseCommand(string, ahm, verbDict, as_msg_modules))

    parseCommand("move forward", ahm, verbDict, as_msg_modules)
    rospy.sleep(1.0)
    parseCommand("stop", ahm, verbDict, as_msg_modules)
    rospy.spin()


if __name__ == '__main__':
    main()





# To call (for now):
# rostopic pub /command_string std_msgs/String "move forward"
