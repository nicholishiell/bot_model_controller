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

float rotControlErrorRed = 0.;
float rotControlErrorBlue = 0.;
float turning_kp = 1.0;

// Behaviour booleans

bool randomWalk = true;
bool trackingPuck = false;
bool trackingGoal = false;
bool closeServo = false;
bool openServo = false;
bool depositPuck = false;
bool rearEvade = false;
bool frontEvade = false;


string behaviourStateString[8] = {"randomWalk", "closeServo","openServo", 
				  "trackingPuck",  "trackingGoal", "depositPuck",
				  "evade","!!! ERROR !!!"};

// Sensor booleans
bool atGoal = false;
bool atPuck = false;
bool seePuck = false;
bool seeGoal = false;
bool frontSensor = false;
bool rearSensor = false;

// State booleans
bool holdingPuck = false;
bool servoOpen = true;
bool timedManuver = false;
bool movingForward = true;
bool speedZero = false;
bool interupted = false;

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

void frontSensorCallback(const vrep_common::ProximitySensorData::ConstPtr& sens){
  printf("Front sensor.\n");
  
  frontSensor = true;
}

void rearSensorCallback(const vrep_common::ProximitySensorData::ConstPtr& sens){
  printf("Rear sensor.\n"); 
  
  rearSensor = true;
}

void cameraBlueCallback(const vrep_common::VisionSensorData::ConstPtr& sens){
  if(depositPuck) return;

  int nPackets = sens->packetSizes.data.size();
   
  int dataSize = 0;

  for(int i = 0; i < nPackets; i++){
    dataSize += sens->packetSizes.data[i];
  }

  // If there are more than 15 packets a blob has been detected.
  // The largest blob is listed first.
  if(dataSize >= 20){
    float blob_xPos = sens->packetData.data[19];
    float blob_yPos = sens->packetData.data[20];
    
    // If the blob is large we are at the deposit site.
    if(blob_yPos < 0.5 and fabs(blob_xPos) - 0.5 < 0.1 or blob_yPos < 0.2 ){
      atGoal= true;
      seeGoal = true;
      rotControlErrorBlue = 0.;
    }
    // If not in no puck in the gripper try to track it
    else{
      atGoal = false;
      seeGoal = true;
      rotControlErrorBlue = blob_xPos-0.5;
    }
  }
  // If no blob information is sent then no blob was seen.
  else{
    atGoal = false;
    seeGoal = false;
    rotControlErrorBlue = 0.;
  }
}

void cameraRedCallback(const vrep_common::VisionSensorData::ConstPtr& sens){
  if(depositPuck) return;

  int nPackets = sens->packetSizes.data.size();
  
  int dataSize = 0;

  for(int i = 0; i < nPackets; i++){
    dataSize += sens->packetSizes.data[i];
  }

  // If there are more than 15 packets a blob has been detected.
  // The largest blob is listed first. We test for 20 because we 
  // want data that ends around 20th bin.
  if(dataSize >= 20){
    float blob_xPos = sens->packetData.data[19];
    float blob_yPos = sens->packetData.data[20];
    
    // If the blob is with in the gripper turn on servo
    if(blob_yPos < 0.15 and fabs(blob_xPos) < 0.55){
      atPuck = true;
      seePuck = true;
      rotControlErrorRed = 0.;
    }
    // If not in no puck in the gripper try to track it
    else{
      atPuck = false;
      seePuck = true;
      rotControlErrorRed = blob_xPos-0.5;
    }
  }
  // If no blob data is recieved
  else{
    rotControlErrorRed = 0.;
    atPuck = false;
    seePuck = false;
  }
}

void OpenServo(int servoMotorHandle, ros::Publisher servoPublisher){
  
  servoOpen = true;

  vrep_common::JointSetStateData servoPosition;
  float servoPos = 0.;
  
  servoPosition.handles.data.push_back(servoMotorHandle);
  servoPosition.setModes.data.push_back(1);
  servoPosition.values.data.push_back(servoPos);
  
  servoPublisher.publish(servoPosition);
}

void CloseServo(int servoMotorHandle, ros::Publisher servoPublisher){
  
  servoOpen = false;

  vrep_common::JointSetStateData servoPosition;
  float servoPos = M_PI;
  
  servoPosition.handles.data.push_back(servoMotorHandle);
  servoPosition.setModes.data.push_back(1);
  servoPosition.values.data.push_back(servoPos);
  
  servoPublisher.publish(servoPosition);
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

void requestSensorPublisher(string topicName, int dataType, int sensorHandle){

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
  int rearSensorHandle;
  int cameraRedHandle;
  int cameraBlueHandle;
  int outputHandle;   
  if (argc>=8){
    leftMotorHandle=atoi(argv[1]);
    rightMotorHandle=atoi(argv[2]);
    servoMotorHandle=atoi(argv[3]);
    frontSensorHandle=atoi(argv[4]);
    rearSensorHandle=atoi(argv[5]);
    cameraRedHandle=atoi(argv[6]);
    cameraBlueHandle=atoi(argv[7]);
    outputHandle=atoi(argv[8]);
  }
  else{
    printf("8 object handles require!");
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
  
  // Front Proxy Sensor
  vrep_common::simRosEnablePublisher frontSensorPublisherRequest;
  frontSensorPublisherRequest.request.topicName = "frontSensorData"+randId; 
  frontSensorPublisherRequest.request.queueSize = 1; 
  frontSensorPublisherRequest.request.streamCmd = simros_strmcmd_read_proximity_sensor; 
  frontSensorPublisherRequest.request.auxInt1 = frontSensorHandle; 
  
  enablePublisherClient.call(frontSensorPublisherRequest);  

  // Rear Proxy Sensor
  vrep_common::simRosEnablePublisher rearSensorPublisherRequest;
  rearSensorPublisherRequest.request.topicName = "rearSensorData"+randId; 
  rearSensorPublisherRequest.request.queueSize = 1; 
  rearSensorPublisherRequest.request.streamCmd = simros_strmcmd_read_proximity_sensor; 
  rearSensorPublisherRequest.request.auxInt1 = rearSensorHandle; 
  
  enablePublisherClient.call(rearSensorPublisherRequest);  
 
  vrep_common::simRosEnablePublisher frontCameraRedPublisherRequest;
  frontCameraRedPublisherRequest.request.topicName = "frontCameraRedData"+randId; 
  frontCameraRedPublisherRequest.request.queueSize = 1; 
  frontCameraRedPublisherRequest.request.streamCmd = simros_strmcmd_read_vision_sensor; 
  frontCameraRedPublisherRequest.request.auxInt1 = cameraRedHandle; 

  enablePublisherClient.call(frontCameraRedPublisherRequest); 
  
  vrep_common::simRosEnablePublisher frontCameraBluePublisherRequest;
  frontCameraBluePublisherRequest.request.topicName = "frontCameraBlueData"+randId; 
  frontCameraBluePublisherRequest.request.queueSize = 1; 
  frontCameraBluePublisherRequest.request.streamCmd = simros_strmcmd_read_vision_sensor; 
  frontCameraBluePublisherRequest.request.auxInt1 = cameraBlueHandle; 

  enablePublisherClient.call(frontCameraBluePublisherRequest); 

  //===========================================================================

  // Now subscribe to the sensor topics
  //===========================================================================
  string frontSensorTopicName("/vrep/frontSensorData");
  frontSensorTopicName += randId;
  ros::Subscriber frontSensorSub = 
    node.subscribe(frontSensorTopicName.c_str(),1,frontSensorCallback);

  string rearSensorTopicName("/vrep/rearSensorData");
  rearSensorTopicName += randId;
  ros::Subscriber rearSensorSub = 
    node.subscribe(rearSensorTopicName.c_str(),1,rearSensorCallback);
  
  string frontCameraRedTopicName("/vrep/frontCameraRedData");
  frontCameraRedTopicName += randId; 
  ros::Subscriber frontCameraRedSub = 
    node.subscribe(frontCameraRedTopicName.c_str(),1,cameraRedCallback);

  string frontCameraBlueTopicName("/vrep/frontCameraBlueData");
  frontCameraBlueTopicName += randId; 
  ros::Subscriber frontCameraBlueSub = 
    node.subscribe(frontCameraBlueTopicName.c_str(),1,cameraBlueCallback);
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
  servoSubscriberRequest.request.streamCmd = simros_strmcmd_set_joint_state;
 
  enableServoSubscriberClient.call(servoSubscriberRequest);
   
  //===========================================================================================================================================================================================

  // The start of the control loop
  printf("botModelController started...\n");
  
  float servoPos = 0.;//M_PI;
  float trans_speed = 5.;
  float rot_speed = 0.;
  float Kp = 2.0;
  float timeStamp = 0.;
  float deltaT = -1.;
   
  float desiredLeftMotorSpeed = 0.;
  float desiredRightMotorSpeed = 0.;
  
  int behaviourFlag = 0;
  
  vrep_common::JointSetStateData servoPosition;
    
  while (ros::ok() and simulationRunning){


    // Determine behaviour based on sensor booleans
    if(simulationTime - timeStamp > deltaT){
            
      depositPuck = (holdingPuck and atGoal); 
      
      randomWalk = (holdingPuck and not seeGoal) or (not holdingPuck and not seePuck);
      
      trackingPuck = (not holdingPuck and seePuck and not atPuck);

      closeServo = atPuck and not holdingPuck;

      openServo = atGoal and holdingPuck;

      trackingGoal = (holdingPuck and seeGoal and not atGoal);
      
      timeStamp = simulationTime;
    }

    if(frontSensor or rearSensor){
      rearEvade = rearSensor;
      frontEvade = frontSensor;

      depositPuck = false;
      randomWalk = false;
      trackingPuck = false;
      trackingGoal = false;

      rearSensor = false;
      frontSensor = false;
    }

    // Execute behaviour
    if(randomWalk){
      behaviourFlag = 0;

      movingForward = true;
      speedZero = false;

      if(rand() / (float)RAND_MAX < 0.1){
	rot_speed = 10.*(rand() / (float)RAND_MAX - 0.5);
      }
      deltaT = -1.;
    }
    else if (depositPuck){
      behaviourFlag = 5;
      
      OpenServo(servoMotorHandle, servoPublisher); 
      
      if(rand() / (float)RAND_MAX < 0.5)
	rot_speed = -1.;
      else
	rot_speed = 1.;
    
      movingForward = false;
      speedZero = false;
      
      deltaT = 5.;

      rotControlErrorBlue = 0.;
      rotControlErrorRed = 0.;
      
      atGoal = false;
      atPuck = false;
      seeGoal = false;
      seePuck = false;

      holdingPuck = false;

    }
    else if(trackingGoal){
      behaviourFlag = 4;

      movingForward = true;
      speedZero = false;

      CloseServo(servoMotorHandle, servoPublisher);

      rot_speed = Kp*rotControlErrorBlue;

      deltaT = -1.;
    }
    else if(trackingPuck){
      behaviourFlag = 3;

      movingForward = true;
      speedZero = false;
       
      OpenServo(servoMotorHandle, servoPublisher);
      rot_speed = Kp*rotControlErrorRed;

       deltaT = -1.;
    }
    else if(frontEvade){
      behaviourFlag = 6;
      
      movingForward = false;
      speedZero = false;

      if(rand() / (float)RAND_MAX < 0.5)
	rot_speed = -1.;
      else
	rot_speed = 1.;

      frontEvade = false;

      deltaT = 1.5;
    }
    
    else if(rearEvade){
      behaviourFlag = 6;
      
      movingForward = true;
      speedZero = false;

      if(rand() / (float)RAND_MAX < 0.5)
	rot_speed = -1.;
      else
	rot_speed = 1.;

      rearEvade = false;
      
      deltaT = 1.5;
    }
    
    else if (closeServo){
      CloseServo(servoMotorHandle, servoPublisher);
      holdingPuck = true;

      deltaT = -1.;
    }
    else if (openServo){
      OpenServo(servoMotorHandle, servoPublisher);
      holdingPuck = false;

      deltaT = -1.;
    }
    else{
      behaviourFlag = 7;
      movingForward = false;
      speedZero = true;
      
      deltaT = -1.;
    }


    // Set trans_speed Depending on speed flags;
    if(speedZero)
      trans_speed = 0.;
    else if(movingForward)
      trans_speed = 5.0;
    else
      trans_speed = -5.0;


    // Given the translation and rotation speeds dictated by the behaviour
    // state the desired left and right wheel motors speeds are calculated.
    desiredLeftMotorSpeed = (2.*trans_speed + rot_speed) / 2.;
    desiredRightMotorSpeed = (2.*trans_speed - rot_speed) / 2.;
    
    std::ostringstream ss;
    /*ss << "w = " << rot_speed << "\n" 
      << "errorBlue = "<<rotControlErrorBlue <<"\n" 
      << "errorRed = "<<rotControlErrorRed <<"\n" 
      << "leftMotor = " << desiredLeftMotorSpeed <<"\n"
      << "rightMotor = " << desiredRightMotorSpeed <<"\n"
      << "behaviour = " << behaviourStateString[behaviourFlag] <<"\n"
      << "holdingPuck = " << holdingPuck<<"\n"
      << "servoOpen = " << servoOpen<<"\n";*/
    ss << "behaviour = " << behaviourStateString[behaviourFlag] <<"\n"
       << "atGoal = " << atGoal <<"\n"
       << "atPuck = " << atPuck <<"\n"
       << "seePuck = " << seePuck <<"\n"
       << "seeGoal = " << seeGoal <<"\n"
       << "frontSensor = " <<frontSensor <<"\n"
       << "rearSensor = " << rearSensor<<"\n"
       << "timeStamp = " << timeStamp <<"\n"
       << "simTime = " << simulationTime <<"\n";
    
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
         
    wheelSpeedPublisher.publish(motorSpeeds);
    
    // handle ROS messages:
    ros::spinOnce();
  }
  // Close down the node.
  ros::shutdown();
  printf("...botModelController stopped\n");
  return(0);
}

