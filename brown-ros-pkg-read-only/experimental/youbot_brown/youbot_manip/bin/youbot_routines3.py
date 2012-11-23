#!/usr/bin/python

import roslib
roslib.load_manifest('youbot_manip')
import rospy
from tabletop_object_detector.srv import TabletopSegmentation, TabletopObjectRecognition, TabletopObjectRecognitionRequest
import tf
import numpy

class youbotObjectDetector():

    def __init__(self):
        rospy.loginfo("youbotObjectDetector: checking for object segmentation service.")
        rospy.wait_for_service("/tabletop_segmentation")
        rospy.loginfo("youbotObjectDetector: tabletop_segmentation service ready.")
        self.seg_srv = rospy.ServiceProxy("/tabletop_segmentation", TabletopSegmentation)

        rospy.loginfo("youbotObjectDetector: checking for object recognition service.")
        rospy.wait_for_service("/tabletop_object_recognition")
        rospy.loginfo("youbotObjectDetector: object recognition service ready.")
        self.recog_srv = rospy.ServiceProxy("/tabletop_object_recognition", TabletopObjectRecognition)

    def call_tabletop_object_detection(self):
        rospy.loginfo("Tabletop object detection service call.")

        try:
            det_res = self.seg_srv()
        except rospy.ServiceException, e:
            rospy.logerr("error when calling %s: %s"%("/tabletop_segmentation", e))
            self.throw_exception()
            return ([], None)        
        rospy.loginfo("object segmentation returned successfully")

        print det_res

        recog_req = TabletopObjectRecognitionRequest()
        recog_req.table = det_res.table
        recog_req.clusters = det_res.clusters
        recog_req.num_models = 2
        recog_req.perform_fit_merge = True

        try:
            recog_res = self.recog_srv()
        except rospy.ServiceException, e:
            rospy.logerr("error when calling %s: %s"%("/tabletop_object_recognition", e))
            self.throw_exception()
            return ([], None)        
        rospy.loginfo("object recognition returned successfully")

        print recog_res

        '''
        #print out the models
        icnt=0
        jcnt=0
        for i in recog_res.models:
            icnt = icnt + 1
            print "Results for set %s"%icnt
            for j in i.model_list:
                jcnt = jcnt + 1
                print "\tModel estimate number %s"%jcnt
                print "\tModel id: %s"%(j.model_id)
                print "\tModel Pose: "
                print j.pose
                print "\tModel confidence: "
                print j.confidence
                print "\tDetector name: %s"%(j.detector_name)
                print "\n"
        '''

if __name__ == '__main__':

    rospy.init_node('youbot_manip', anonymous=True)
    youbot_Object_Detector = youbotObjectDetector()
    youbot_Object_Detector.call_tabletop_object_detection()





