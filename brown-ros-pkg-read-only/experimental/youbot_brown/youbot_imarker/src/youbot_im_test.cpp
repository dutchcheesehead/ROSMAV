
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <interactive_markers/interactive_marker_server.h>
#include "youbot_imarker/diffval.h"
#include "youbot_with_kinect_ik.cpp"

#include "trajectory_msgs/JointTrajectory.h"
#include "brics_actuator/CartesianWrench.h"

#include <sensor_msgs/JointState.h>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <iostream>
#include <assert.h>

#include "brics_actuator/JointPositions.h"

#include <boost/units/systems/si/length.hpp>
#include <boost/units/systems/si/plane_angle.hpp>
#include <boost/units/io.hpp>

#include <boost/units/systems/angle/degrees.hpp>
#include <boost/units/conversion.hpp>

#include <LinearMath/btVector3.h>
#include <LinearMath/btMatrix3x3.h>
#include <LinearMath/btQuaternion.h>

double orx1,ory1,orz1;
double arm_joint4_val, arm_joint5_val;
bool chit;
//bool diffbr;

ros::Publisher diffval_pub;

ros::Publisher armPositionsPublisher;
ros::Publisher gripperPositionPublisher;

using namespace std;

/*
static void prVector3(const btVector3 &vecarg)
{
  cout << vecarg[0] << " " << vecarg[1] << " " << vecarg[2];
}
*/

static void prQuaternion(const btQuaternion &qarg)
{
  cout << qarg.x() << " " << qarg.y() << " " << qarg.z() << " " << qarg.w();
}

static void prMatrix3(const btMatrix3x3 &marg)
{
  cout << marg[0][0] << " " << marg[0][1] << " " << marg[0][2] << "\n";
  cout << marg[1][0] << " " << marg[1][1] << " " << marg[1][2] << "\n";
  cout << marg[2][0] << " " << marg[2][1] << " " << marg[2][2] << "\n";
}

int control_robot(youbot_imarker::diffval dval)
{
  // Get the robot's current end effector pose
  tf::TransformListener listener;
  tf::StampedTransform transform;
  
  try
    {
      //listener.waitForTransform("/base_link", "/gripper_palm_link", ros::Time(0),ros::Duration(1.0));
      //listener.lookupTransform("/base_link", "/gripper_palm_link", ros::Time(0),transform);
      listener.waitForTransform("/arm_link_0", "/arm_link_5", ros::Time(0),ros::Duration(1.0));
      listener.lookupTransform("/arm_link_0", "/arm_link_5", ros::Time(0),transform);
    }
    catch (tf::TransformException ex){
      ROS_ERROR("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

  double px,py,pz;
  double nx,ny,nz;
  double qx,qy,qz,qw;

  px=py=pz=0.0;
  nx=ny=nz=0.0;
  qx=qy=qz=qw=0.0;

  px = transform.getOrigin().x();
  py = transform.getOrigin().y();
  pz = transform.getOrigin().z();

  printf("Gripper pose(translation):\n");
  printf("px: %3.15f, ", px);  
  printf("py: %3.15f, ", py);  
  printf("pz: %3.15f\n", pz);  

  qx = transform.getRotation().x();
  qy = transform.getRotation().y();
  qz = transform.getRotation().z();
  qw = transform.getRotation().w();

  btQuaternion qrot(qx,qy,qz,qw);

  cout << "qval: " << qx << ", " << qy << ", " << qz << ", " << qw << "\n";
  cout << "Quaternion: ";
  prQuaternion(qrot);
  cout << "\n";

  btMatrix3x3 rotmat(qrot);

  cout << "rotmat: ";
  prMatrix3(rotmat);
  cout << "\n";

  printf("diffval:\n");
  printf("xx: %3.15f, ", dval.xx);  
  printf("yy: %3.15f, ", dval.yy);  
  printf("zz: %3.15f\n", dval.zz);  
  
  nx = px + dval.xx;
  ny = py + dval.yy;
  nz = pz + dval.zz;

  printf("Sending pose:\n");
  printf("nx: %3.15f, ", nx);  
  printf("ny: %3.15f, ", ny);  
  printf("nz: %3.15f\n\n", nz);  

  std::vector<IKSolution> vsolutions;
  std::vector<IKReal> vfree(2);
  IKReal eerot[9],eetrans[3];
  /*
  eerot[0] = atof(argv[1]); eerot[1] = atof(argv[2]); eerot[2] = atof(argv[3]); eetrans[0] = atof(argv[4]);
  eerot[3] = atof(argv[5]); eerot[4] = atof(argv[6]); eerot[5] = atof(argv[7]); eetrans[1] = atof(argv[8]);
  eerot[6] = atof(argv[9]); eerot[7] = atof(argv[10]); eerot[8] = atof(argv[11]); eetrans[2] = atof(argv[12]);
  for(std::size_t i = 0; i < vfree.size(); ++i)
    vfree[i] = atof(argv[13+i]);
  */

  eerot[0]=rotmat[0][0]; 
  eerot[1]=rotmat[0][1]; 
  eerot[2]=rotmat[0][2]; 
  eetrans[0]=nx;
  
  eerot[3]=rotmat[1][0]; 
  eerot[4]=rotmat[1][1]; 
  eerot[5]=rotmat[1][2];
  eetrans[1]=ny;
  
  eerot[6]=rotmat[2][0]; 
  eerot[7]=rotmat[2][1]; 
  eerot[8]=rotmat[2][2]; 
  eetrans[2]=nz;

  vfree[0]=3;
  vfree[1]=4;

  bool bSuccess = ik(eetrans, eerot, &vfree[0], vsolutions);
  
  if( !bSuccess ) {
    fprintf(stderr,"Failed to get ik solution\n");
    return -1;
  }
  
  printf("Found %d ik solutions:\n", (int)vsolutions.size());
  std::vector<IKReal> sol(getNumJoints());
  for(std::size_t i = 0; i < vsolutions.size(); ++i) {
    printf("sol%d (free=%d): ", (int)i, (int)vsolutions[i].GetFree().size());
    std::vector<IKReal> vsolfree(vsolutions[i].GetFree().size());
    vsolutions[i].GetSolution(&sol[0],vsolfree.size()>0?&vsolfree[0]:NULL);
    for( std::size_t j = 0; j < sol.size(); ++j)
      printf("%.15f, ", sol[j]);
    printf("\n");
  }

  // Need to get the robot's current configuration
  
  static const int use_sol_number=0; 
  
  static const int numberOfArmJoints = 5;
  static const int numberOfGripperJoints = 2;
  
  brics_actuator::JointPositions command;
  vector <brics_actuator::JointValue> armJointPositions;
  vector <brics_actuator::JointValue> gripperJointPositions;
  
  armJointPositions.resize(numberOfArmJoints); //TODO:change that
  gripperJointPositions.resize(numberOfGripperJoints);

  std::vector<IKReal> sol_temp(getNumJoints());
  
  std::vector<IKReal> vsolfree_temp(vsolutions[use_sol_number].GetFree().size());

  vsolutions[use_sol_number].GetSolution(&sol_temp[0],vsolfree_temp.size()>0?&vsolfree_temp[0]:NULL);

  //for( std::size_t j = 0; j < sol.size(); ++j)
  //printf("%.15f, ", sol[j]);

  std::stringstream jointName;

  // ::io::base_unit_info <boost::units::si::angular_velocity>).name();
  int i=0;
  for (; i < (numberOfArmJoints-2); ++i) {
    
    jointName.str("");
    jointName << "arm_joint_" << (i + 1);
    
    armJointPositions[i].joint_uri = jointName.str();
    armJointPositions[i].value = sol_temp[i];
    
    armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
    cout << "Joint " << armJointPositions[i].joint_uri << " = " << armJointPositions[i].value << " " << armJointPositions[i].unit << endl;
    
  };

  // Fill in joint values for the free-indices
  // This is for arm_joint_4
  jointName.str("");
  jointName << "arm_joint_" << (i + 1);
  
  armJointPositions[i].joint_uri = jointName.str();
  armJointPositions[i].value = arm_joint4_val;
  
  armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
  cout << "Joint " << armJointPositions[i].joint_uri << " = " << armJointPositions[i].value << " " << armJointPositions[i].unit << endl;
  
  // This is for arm_joint_5
  i++;
  jointName.str("");
  jointName << "arm_joint_" << (i + 1);
  
  armJointPositions[i].joint_uri = jointName.str();
  armJointPositions[i].value = arm_joint5_val;
  
  armJointPositions[i].unit = boost::units::to_string(boost::units::si::radians);
  cout << "Joint " << armJointPositions[i].joint_uri << " = " << armJointPositions[i].value << " " << armJointPositions[i].unit << endl;

  // Gripper joints
  gripperJointPositions[0].joint_uri = "gripper_finger_joint_l";
  gripperJointPositions[0].value = 1.0;
  gripperJointPositions[0].unit = boost::units::to_string(boost::units::si::meter);
  
  gripperJointPositions[1].joint_uri = "gripper_finger_joint_r";
  gripperJointPositions[1].value = 1.0;
  gripperJointPositions[1].unit = boost::units::to_string(boost::units::si::meter);
  
  cout << "sending command ..." << endl;
  
  command.positions = armJointPositions;
  armPositionsPublisher.publish(command);
  
  command.positions = gripperJointPositions;
  gripperPositionPublisher.publish(command);
  
  return 1;
}

void processFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  /*
  ROS_INFO_STREAM( feedback->marker_name << " is now at "
      << feedback->pose.position.x << ", " << feedback->pose.position.y
      << ", " << feedback->pose.position.z );
  */
  youbot_imarker::diffval dval;
  bool pfsend_command=false;
  dval.xx=0.0;
  dval.yy=0.0;
  dval.zz=0.0;
  if(!chit)
    {
      orx1=feedback->pose.position.x;
      ory1=feedback->pose.position.y;
      orz1=feedback->pose.position.z;
      chit=true;
    }
  else
    {
      if (fabs(orx1-feedback->pose.position.x)>0.01)
	{
	  //publish new x
	  dval.xx = orx1-feedback->pose.position.x;
	  pfsend_command=true;
	}
      else
	{
	  dval.xx=0.0;
	}

      if (fabs(ory1-feedback->pose.position.y)>0.01)
	{
	  //publish new y
	  dval.yy = ory1-feedback->pose.position.y;
	  pfsend_command=true;
	}
      else
	{
	  dval.yy=0.0;
	}

      if (fabs(orz1-feedback->pose.position.z)>0.01)
	{
	  //publish new z
	  dval.zz = orz1-feedback->pose.position.z;
	  pfsend_command=true;
	}
      else
	{
	  dval.zz=0.0;
	}

      orx1=feedback->pose.position.x;
      ory1=feedback->pose.position.y;
      orz1=feedback->pose.position.z;
    }

  //if (!diffbr)
  //{
  if(pfsend_command)
    control_robot(dval);
  diffval_pub.publish(dval);
  //}

  //if((fabs(dval.xx)>0)||(fabs(dval.yy)>0)||(fabs(dval.zz)>0))
  //diffbr=true;

}

void joint_states_callback(const sensor_msgs::JointState& youbot_joint_state)
{
  //vector<string> jname = youbot_joint_state.name;
  vector<double> jpos = youbot_joint_state.position;
  //vector<string>::iterator itname = jname.begin();
  vector<double>::iterator itpos = jpos.begin();
  //cout << "joint_states_callback: \n";

  // we need joints 12 and 13
  for(int i=0; i<11; i++)
    {
      //itname++;
      itpos++;
    }

  //cout << (*itname) << ": " << (*itpos) << "   ";
  arm_joint4_val = (*itpos);
  //itname++;
  itpos++;
  arm_joint5_val = (*itpos);
  //cout << (*itname) << ": " << (*itpos) << "\n\n";  
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "simple_marker");
  orx1=0;
  ory1=0;
  orz1=0;
  arm_joint4_val=arm_joint5_val=0.0;
  chit=false;
  //diffbr=false;

  ros::NodeHandle n;

  ros::Subscriber lclzr_sub = n.subscribe("/joint_states",1,&joint_states_callback);

  armPositionsPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/arm_controller/position_command", 1);
  gripperPositionPublisher = n.advertise<brics_actuator::JointPositions > ("arm_1/gripper_controller/position_command", 1);

  /*
  tf::TransformListener listener;
  tf::StampedTransform transform;
  listener.lookupTransform("/base_footprint", "/gripper_palm_link", ros::Time(0), transform);
  double xee = transform.getOrigin().x();
  double yee = transform.getOrigin().y();
  double zee = transform.getOrigin().z();

  std::cout << "\nxee,yee,zee: " << xee << " " << yee << " " << zee << "\n";
  */

  //ros::Subscriber tf_sub = n.subscribe("/youbot_im_topic", 1, &youbot_im_callback);

  diffval_pub = n.advertise<youbot_imarker::diffval>("diffval",1);

  // create an interactive marker server on the topic namespace simple_marker
  interactive_markers::InteractiveMarkerServer server("simple_marker");

  // create an interactive marker for our server
  visualization_msgs::InteractiveMarker int_marker;
  //int_marker.header.frame_id = "/gripper_palm_link";
  int_marker.header.frame_id = "/arm_link_5";
  int_marker.name = "my_marker";
  int_marker.description = "Simple 3-DOF Control";

  // create a grey box marker
  visualization_msgs::Marker box_marker;
  box_marker.type = visualization_msgs::Marker::CUBE;
  /*
  box_marker.scale.x = 0.45;
  box_marker.scale.y = 0.45;
  box_marker.scale.z = 0.45;
  */
  box_marker.scale.x = 0.06;
  box_marker.scale.y = 0.06;
  box_marker.scale.z = 0.06;

  box_marker.color.r = 0.5;
  box_marker.color.g = 0.5;
  box_marker.color.b = 0.5;
  box_marker.color.a = 1.0;

  // create a non-interactive control which contains the box
  visualization_msgs::InteractiveMarkerControl box_control;
  box_control.always_visible = true;
  box_control.markers.push_back( box_marker );

  // add the control to the interactive marker
  int_marker.controls.push_back( box_control );

  // create a control which will move the box
  // this control does not contain any markers,
  // which will cause RViz to insert two arrows
  visualization_msgs::InteractiveMarkerControl move_control;

  move_control.orientation.w = 1;
  move_control.orientation.x = 1;
  move_control.orientation.y = 0;
  move_control.orientation.z = 0;

  move_control.name = "move_x";
  move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(move_control);

  move_control.orientation.w = 1;
  move_control.orientation.x = 0;
  move_control.orientation.y = 1;
  move_control.orientation.z = 0;

  move_control.name = "move_y";
  move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(move_control);

  move_control.orientation.w = 1;
  move_control.orientation.x = 0;
  move_control.orientation.y = 0;
  move_control.orientation.z = 1;

  move_control.name = "move_z";
  move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(move_control);


  //move_control.name = "move_y";
  //move_control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;

  // add the control to the interactive marker
  //int_marker.controls.push_back(move_control);


  // add the interactive marker to our collection &
  // tell the server to call processFeedback() when feedback arrives for it
  server.insert(int_marker, &processFeedback);

  // 'commit' changes and send to all clients
  server.applyChanges();

  // start the ROS main loop
  ros::spin();
}
// %Tag(fullSource)%
