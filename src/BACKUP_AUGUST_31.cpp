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

// Finite State Machine used to control Behaviour
#include "FSM/FSM.h"

using namespace std;

// Create a node for communicating with ROS.
//ros::NodeHandle node("~");

// Global variables (modified by topic subscribers):
bool simulationRunning=true;
float simulationTime=0.0f;

float rotControlErrorRed = 0.;
float rotControlErrorBlue = 0.;

float targetPuckX = 0.;
float targetPuckY = 0.;

float targetGoalX = 0.;
float targetGoalY = 0.;

//float turning_kp = 1.0;

// Sensor booleans
bool atGoal = false;
bool atPuck = false;
bool seePuck = false;
bool seeGoal = false;
bool frontSensor = false;
bool rearSensor = false;
bool puck2Close2Goal = false;
bool movingForward = true;


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
    
    targetGoalX = blob_xPos;
    targetGoalY = blob_yPos;

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
    
    targetPuckX = blob_xPos;
    targetPuckY = blob_yPos;

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
 
  vrep_common::JointSetStateData servoPosition;
  float servoPos = 0.;
  
  servoPosition.handles.data.push_back(servoMotorHandle);
  servoPosition.setModes.data.push_back(1);
  servoPosition.values.data.push_back(servoPos);
  
  servoPublisher.publish(servoPosition);
}

void CloseServo(int servoMotorHandle, ros::Publisher servoPublisher){
  
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
 
bool CheckPuckGoalDistance(){
 
  float xDiff = fabs(targetGoalX - targetPuckX);
  float yDiff = fabs(targetGoalY - targetPuckY);
 
  if((xDiff < 0.07 and yDiff < 0.07) || yDiff < 0.05)
    return true;
  else
    return false;
  
}


void RequestPublisher(ros::NodeHandle node, string topicName, 
		      int queueSize, int streamCmd, int objectHandle){

  ros::ServiceClient enablePublisherClient=
    node.serviceClient<vrep_common::simRosEnablePublisher>
    ("/vrep/simRosEnablePublisher");
  
  vrep_common::simRosEnablePublisher publisherRequest;
  
  publisherRequest.request.topicName = topicName;
  publisherRequest.request.queueSize = queueSize; 
  publisherRequest.request.streamCmd = streamCmd; 
  publisherRequest.request.auxInt1 = objectHandle; 

  enablePublisherClient.call(publisherRequest);  
}


//===========================================================================
// Main Function
//===========================================================================
int main(int argc,char* argv[]){  

  StateManager * fsm = new StateManager();

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
  
   RequestPublisher(node, "frontSensorData"+randId, 1, 
		   simros_strmcmd_read_proximity_sensor, frontSensorHandle);
  
  RequestPublisher(node, "rearSensorData"+randId, 1, 
		   simros_strmcmd_read_proximity_sensor, rearSensorHandle);
  
  RequestPublisher(node, "frontCameraRedData"+randId, 1, 
		  simros_strmcmd_read_vision_sensor , cameraRedHandle);
  
  RequestPublisher(node, "frontCameraBlueData"+randId, 1, 
		   simros_strmcmd_read_vision_sensor, cameraBlueHandle);
  



  /* ros::ServiceClient enablePublisherClient=
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

  enablePublisherClient.call(frontCameraBluePublisherRequest); */

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
  
  // These values are passed to the FSM
  float trans_speed = 5.;
  float rot_speed = 0.;
  bool openServo = true;
 
  float desiredLeftMotorSpeed = 0.;
  float desiredRightMotorSpeed = 0.;
  
  vrep_common::JointSetStateData servoPosition;
    
  while (ros::ok() and simulationRunning){

    bool stimuli[7];
    stimuli[0] = seePuck;
    stimuli[1] = atPuck;
    stimuli[2] = seeGoal;
    stimuli[3] = atGoal;
    stimuli[4] = movingForward;
    stimuli[5] = CheckPuckGoalDistance();
    stimuli[6] = frontSensor || rearSensor;


    fsm->UpdateBehaviour(stimuli);
    
    fsm->SetPuckControlError(rotControlErrorRed);
    fsm->SetGoalControlError(rotControlErrorBlue);

    // ExecuteBehaviour will return a translational speed, rotational speed, and a boolean
    // to determine weather to open or close the servo.
    fsm->ExecuteBehaviour(trans_speed, rot_speed, openServo);
    
    // Depending on what behaviour dictates open/close servo for puck lock
    if(openServo)
      OpenServo(servoMotorHandle, servoPublisher);
    else
      CloseServo(servoMotorHandle, servoPublisher);

    // Given the translation and rotation speeds dictated by the behaviour
    // state the desired left and right wheel motors speeds are calculated.
    desiredLeftMotorSpeed = (2.*trans_speed + rot_speed) / 2.;
    desiredRightMotorSpeed = (2.*trans_speed - rot_speed) / 2.;
    

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
 
    // A message to publish to the console in V-REP for debugging.
    std::string behaviour = fsm->GetCurrentStateName();
    std::ostringstream ss;
    ss << "behaviour = " << behaviour  <<"\n"
       << "seePuck = " << seePuck <<"\n"
       << "havePuck = " << atPuck <<"\n"
       << "seeGoal = " << seeGoal <<"\n"
       << "atGoal = " << atGoal <<"\n"
       << "movingForward = " << movingForward<<"\n"
       << "puck2Close2Goal = " << puck2Close2Goal<<"\n"
       << "prox = " << (frontSensor || rearSensor) <<"\n"
       << "transSpeed = " << trans_speed<<"\n"
       << "rotSpeed = " << rot_speed<<"\n";
    
    std::string msg(ss.str());
    sendMsg2Console(node, outputHandle, msg);
   
    // Reset Prox sensors
    frontSensor = false;
    rearSensor = false;

    // handle ROS messages:
    ros::spinOnce();
  }
  // Close down the node.
  ros::shutdown();
  printf("...botModelController stopped\n");
  return(0);
}

