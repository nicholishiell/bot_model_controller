#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// ROS indudes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

// Include for V-REP
#include "../include/v_repConst.h"
// Used data structures:
#include "vrep_common/ProximitySensorData.h"
#include "vrep_common/VrepInfo.h"
#include "vrep_common/JointSetStateData.h"
#include "vrep_common/VisionSensorData.h"

// Used API services:
#include "vrep_common/simRosEnablePublisher.h"
#include "vrep_common/simRosEnableSubscriber.h"
#include "vrep_common/simRosAuxiliaryConsolePrint.h"
#include "vrep_common/simRosAddStatusbarMessage.h"
#include "vrep_common/simRosAuxiliaryConsoleOpen.h"
#include "vrep_common/simRosAuxiliaryConsoleShow.h"

using namespace std;


// Global variables (modified by topic subscribers):
bool simulationRunning=true;
float simulationTime=0.0f;

float rotControlError = 0.;
float turning_kp = 1.0;
// These global variables control the state of the robot
bool holdingPuck = false;
bool randomWalk = true;
bool trackingPuck = false;
bool trackingGoal = false;
string behaviourStateString[5] = {"randomWalk", "closeServo", "trackingPuck",
				  "trackingGoal", "other"};

//===========================================================================
// Function Prototypes
//===========================================================================
void sendMsg2Console(ros::NodeHandle node, int outputHandle, string msg);

//===========================================================================
// Topic subscriber callbacks:
//===========================================================================
void infoCallback(const vrep_common::VrepInfo::ConstPtr& info){
  simulationTime=info->simulationTime.data;
  simulationRunning=(info->simulatorState.data&1)!=0;
}

void sensorCallback(const vrep_common::ProximitySensorData::ConstPtr& sens){
  //printf("Front sensor.\n");
}

void cameraCallback(const vrep_common::VisionSensorData::ConstPtr& sens){
  int nPackets = sens->packetSizes.data.size();
  printf("nPackets: %d\n", nPackets);
  
  int dataSize = 0;

  for(int i = 0; i < nPackets; i++){
    dataSize += sens->packetSizes.data[i];
  }

  // If there are more than 15 packets a blob has been detected.
  // The largest blob is listed first.
  if(dataSize >= 20){
    float blob_xPos = sens->packetData.data[19];
    float blob_yPos = sens->packetData.data[20];
    float blob_size = sens->packetData.data[18];
    
    printf("blobX = %f\n", blob_xPos); 
    printf("blobY = %f\n", blob_yPos); 
    
    // If the blob is with in the gripper turn on servo
    if(blob_yPos < 0.15 and fabs(blob_xPos) < 0.55){
      holdingPuck = true;
      randomWalk = true;
    }
    // If not in no puck in the gripper try to track it
    else{
      holdingPuck = false;
      rotControlError = blob_xPos-0.5;
      turning_kp = blob_size*1000.;//(float)100;
    }
  
    randomWalk = false;
    trackingPuck = true;
  }
  else{
    rotControlError = 0.;
    randomWalk = true;
    trackingPuck = false;
  }
}
//===========================================================================
// Helper Functions
//===========================================================================
void sendMsg2Console(ros::NodeHandle node, int outputHandle, string msg){
   
  ros::ServiceClient consoleClient =
    node.serviceClient<vrep_common::simRosAuxiliaryConsolePrint>("/vrep/simRosAuxiliaryConsolePrint");
  vrep_common::simRosAuxiliaryConsolePrint consoleMsg;
 
  consoleMsg.request.consoleHandle=outputHandle;
  consoleMsg.request.text = msg;

  consoleClient.call(consoleMsg);

  return;
}

void requestSensorPublisher(string topicName, int dataType, int sensorHangle){

  return ;
}

//===========================================================================
// Main Function
//===========================================================================
int main(int argc,char* argv[]){  


  // Parse the arguments passed to the node
  //===========================================================================
  int leftMotorHandle;
  int rightMotorHandle;
  int servoMotorHandle;
  int frontSensorHandle;
  int cameraRedHandle;
  int cameraBlueHandle;
  int outputHandle;   
  if (argc>=6){
    leftMotorHandle=atoi(argv[1]);
    rightMotorHandle=atoi(argv[2]);
    servoMotorHandle=atoi(argv[3]);
    frontSensorHandle=atoi(argv[4]);
    cameraRedHandle=atoi(argv[5]);
    outputHandle=atoi(argv[6]);
  }
  else{
    printf("6 object handles require!");
    sleep(5000);
    return 0;
  }
  //===========================================================================


  // Create a ROS node. The name has a random component: 
  //===========================================================================
  int _argc = 0;
  char** _argv = NULL;
  struct timeval tv;
  unsigned int timeVal=0;
  if (gettimeofday(&tv,NULL)==0)
    timeVal=(tv.tv_sec*1000+tv.tv_usec/1000)&0x00ffffff;
  std::string nodeName("botModelController");
  std::string randId(boost::lexical_cast<std::string>(timeVal+int(999999.0f*(rand()/(float)RAND_MAX))));
  nodeName+=randId;		
  ros::init(_argc,_argv,nodeName.c_str());
  //===========================================================================


  if(!ros::master::check())
    return(0);
  
  // Create a node for communicating with ROS.
  //===========================================================================
  ros::NodeHandle node("~");
  //===========================================================================


  // Subscribe to the vrep info topic to know when the simulation ends
  //===========================================================================
   ros::Subscriber vrepInfoSub = 
    node.subscribe("/vrep/info/",1,infoCallback);
  //===========================================================================

 
  // Get V-Rep to publish sensor data to topics
  //===========================================================================
  ros::ServiceClient enablePublisherClient=
    node.serviceClient<vrep_common::simRosEnablePublisher>
    ("/vrep/simRosEnablePublisher");
  
  vrep_common::simRosEnablePublisher frontSensorPublisherRequest;
  frontSensorPublisherRequest.request.topicName = "frontSensorData"+randId; 
  frontSensorPublisherRequest.request.queueSize = 1; 
  frontSensorPublisherRequest.request.streamCmd = simros_strmcmd_read_proximity_sensor; 
  frontSensorPublisherRequest.request.auxInt1 = frontSensorHandle; 
  
  enablePublisherClient.call(frontSensorPublisherRequest);  
 
  vrep_common::simRosEnablePublisher frontCameraPublisherRequest;
  frontCameraPublisherRequest.request.topicName = "frontCameraData"+randId; 
  frontCameraPublisherRequest.request.queueSize = 1; 
  frontCameraPublisherRequest.request.streamCmd = simros_strmcmd_read_vision_sensor; 
  frontCameraPublisherRequest.request.auxInt1 = cameraRedHandle; 

  enablePublisherClient.call(frontCameraPublisherRequest); 
  //===========================================================================


  // Now subscribe to the sensor topics
  //===========================================================================
  string frontSensorTopicName("/vrep/frontSensorData");
  frontSensorTopicName += randId;
  ros::Subscriber frontSensorSub = 
    node.subscribe(frontSensorTopicName.c_str(),1,sensorCallback);
  
  string frontCameraTopicName("/vrep/frontCameraData");
  frontCameraTopicName += randId; 
  ros::Subscriber frontCameraSub = 
    node.subscribe(frontCameraTopicName.c_str(),1,cameraCallback);
  //===========================================================================


  // Now setup a publisher to control the WHEEL MOTORS and get V-REP to subscribe
  //===========================================================================
  ros::Publisher wheelSpeedPublisher = 
    node.advertise<vrep_common::JointSetStateData>("wheels",1);
 
  ros::ServiceClient enableWheelSubscriberClient =
    node.serviceClient<vrep_common::simRosEnableSubscriber>
    ("/vrep/simRosEnableSubscriber");

  vrep_common::simRosEnableSubscriber wheelSpeedSubscriberRequest;
  
  wheelSpeedSubscriberRequest.request.topicName = "/"+nodeName+"/wheels"; 
  wheelSpeedSubscriberRequest.request.queueSize = 1; 
  wheelSpeedSubscriberRequest.request.streamCmd = simros_strmcmd_set_joint_state; 
  
  enableWheelSubscriberClient.call(wheelSpeedSubscriberRequest);
  //===========================================================================
  
  // Now setup a publisher to control the SERVO  and get V-REP to subscribe
  //===========================================================================
  ros::Publisher servoPublisher = 
    node.advertise<vrep_common::JointSetStateData>("servo",1);
 
  // Initialize client object
  ros::ServiceClient enableServoSubscriberClient =
    node.serviceClient<vrep_common::simRosEnableSubscriber>
    ("/vrep/simRosEnableSubscriber");

  // Initialize msg object to be sent to client 
  vrep_common::simRosEnableSubscriber servoSubscriberRequest;

  servoSubscriberRequest.request.topicName = "/"+nodeName+"/servo"; 
  servoSubscriberRequest.request.queueSize = 1; 
  //servoSubscriberRequest.request.streamCmd = simros_strmcmd_set_joint_target_position; 
  servoSubscriberRequest.request.streamCmd = simros_strmcmd_set_joint_state;
 
  if(!enableServoSubscriberClient.call(servoSubscriberRequest))
    printf("ERROR!\n");
  //===========================================================================================================================================================================================

  // The start of the control loop
  printf("botModelController started...\n");
  
  float servoPos = 0.;//M_PI;
  float trans_speed = 2.5;
  float rot_speed = 0.;
  float Kp = 2.0;
  float timeStamp = 0.;
  float deltaT = -1.;
  
  vrep_common::JointSetStateData servoPosition;
  
  while (ros::ok() and simulationRunning){
    float desiredLeftMotorSpeed = 0.;
    float desiredRightMotorSpeed = 0.;
 
    if(randomWalk){
      sendMsg2Console(node, outputHandle, behaviourStateString[0]);
      if(simulationTime - timeStamp > deltaT) {
	 rot_speed = 10.*(rand() / (float)RAND_MAX - 0.5);
	 timeStamp = simulationTime;
	 deltaT = 3.*(rand() / (float)RAND_MAX);
      }
      
    }
    else if(trackingPuck){
      sendMsg2Console(node, outputHandle, behaviourStateString[2]);
      //rot_speed = turning_kp*rotControlError;
      rot_speed = Kp*rotControlError;
    }
    else if(trackingGoal){

    }
    else{}
    sendMsg2Console(node, outputHandle, string("\n"));
    // Given the translation and rotation speeds dictated by the behaviour
    // state the desired left and right wheel motors speeds are calculated.
    desiredLeftMotorSpeed = (2.*trans_speed + rot_speed) / 2.;
    desiredRightMotorSpeed = (2.*trans_speed - rot_speed) / 2.;

    std::ostringstream ss;
    ss << "w = " << rot_speed << "\n" <<"error = "<<rotControlError<<"\n" 
       << "leftMotor = " << desiredLeftMotorSpeed <<"\n"
       << "rightMotor = " << desiredRightMotorSpeed <<"\n";
    std::string msg(ss.str());
    sendMsg2Console(node, outputHandle, msg);


    // Now that we know what speeds we need the wheels
    // to rotate at we can send that info to V-REP 
    vrep_common::JointSetStateData motorSpeeds;
   
    motorSpeeds.handles.data.push_back(leftMotorHandle);
    motorSpeeds.handles.data.push_back(rightMotorHandle);
    motorSpeeds.setModes.data.push_back(2); // 2 is the speed mode
    motorSpeeds.setModes.data.push_back(2);
    motorSpeeds.values.data.push_back(desiredLeftMotorSpeed);
    motorSpeeds.values.data.push_back(desiredRightMotorSpeed);
     
    vrep_common::JointSetStateData servoPosition;
    servoPosition.handles.data.push_back(servoMotorHandle);
    servoPosition.setModes.data.push_back(1);
    servoPosition.values.data.push_back(servoPos);
    servoPos += 10.;
    
    wheelSpeedPublisher.publish(motorSpeeds);
    
    if(holdingPuck){
      sendMsg2Console(node, outputHandle, behaviourStateString[1]);
      sendMsg2Console(node, outputHandle, string("\n"));
      servoPos = M_PI;
      servoPosition.handles.data.push_back(servoMotorHandle);
      servoPosition.setModes.data.push_back(1);
      servoPosition.values.data.push_back(servoPos);
  
      servoPublisher.publish(servoPosition);
    }
    else{
      servoPos = 0.;
      servoPosition.handles.data.push_back(servoMotorHandle);
      servoPosition.setModes.data.push_back(1);
      servoPosition.values.data.push_back(servoPos);
  
      servoPublisher.publish(servoPosition);
    }
    // handle ROS messages:
    ros::spinOnce();
  }
  // Close down the node.
  ros::shutdown();
  printf("...botModelController stopped\n");
  return(0);
}

