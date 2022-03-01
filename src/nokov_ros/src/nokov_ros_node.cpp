#include <math.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <iostream>
#include <vector>
#include<algorithm>
#include<fstream>
#include<chrono>

#include "m3x3.h"
#include "cortex.h"

#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include <Eigen/Core>
#include <Eigen/Eigen>
#include <Eigen/Geometry>

ros::Publisher visionPosPub;
ros::Publisher visionPosPub2;
ros::Publisher visionPosPub21;

void MyErrorMsgHandler(int iLevel, const char *szMsg)				
{
  const char *szLevel = NULL;

  if (iLevel == VL_Debug) {
    szLevel = "Debug";
  } else if (iLevel == VL_Info) {
    szLevel = "Info";
  } else if (iLevel == VL_Warning) {
    szLevel = "Warning";
  } else if (iLevel == VL_Error) {
    szLevel = "Error";
  }

  printf("    %s: %s\n", szLevel, szMsg);
}

// Vector for calculating time statistics
std::vector<double> vTimesCalculate;

void MyDataHandler(sFrameOfData* FrameOfData)
{
#ifdef COMPILEDWITHC11
  std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
  std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

  ros::Time currentTime = ros::Time::now();

  double p0[MAX_N_BODIES][3], // origin marker
	       p1[MAX_N_BODIES][3], // long axis marker
	       p2[MAX_N_BODIES][3]; // plane marker
	double matrix[MAX_N_BODIES][3][3];

  printf("Received multi-cast frame number %d\t, and body's number: %d\n", FrameOfData->iFrame, FrameOfData->nBodies);	

  sBodyData Body[MAX_N_BODIES];
  for(int i = 0; i < FrameOfData->nBodies; i++)
  {
    Body[i] = FrameOfData->BodyData[i];

    for(int j = 0; j < 3; j++)
    // for(int j = 0; j < Body[i].nMarkers; j++)
    {
      p0[i][j] = Body[i].Markers[0][j];
      p1[i][j] = Body[i].Markers[1][j];
      p2[i][j] = Body[i].Markers[2][j];
      // std::cout << Body[0].Markers[0][j] << "\t" << Body[0].Markers[1][j] << "\t" << Body[0].Markers[2][j] << std::endl;
    }

    M3x3_BuildVMarkerRotationMatrix(
        p0[i], 
        p1[i], 
        p2[i], 
        matrix[i]);

    // std::cout << "matrix[i]:" << std::endl;
    // std::cout << matrix[i][0][0] << "\t" << matrix[i][0][1]<< "\t" << matrix[i][0][2] << std::endl
    //           << matrix[i][1][0] << "\t" << matrix[i][1][1]<< "\t" << matrix[i][1][2] << std::endl
    //           << matrix[i][2][0] << "\t" << matrix[i][2][1]<< "\t" << matrix[i][2][2] << std::endl;
  }

  std::vector<Eigen::Matrix3d> vRotationMatrix(FrameOfData->nBodies, Eigen::Matrix3d::Identity());
  std::vector<Eigen::Quaterniond> vQuaternion(FrameOfData->nBodies, Eigen::Quaterniond(Eigen::Matrix3d::Identity()));
  std::vector<Eigen::Isometry3d> vTransformationMatrix(FrameOfData->nBodies, Eigen::Isometry3d::Identity());

  for(int i = 0; i < FrameOfData->nBodies; i++)
  {
    vRotationMatrix[i] << matrix[i][0][0], matrix[i][1][0], matrix[i][2][0],
                          matrix[i][0][1], matrix[i][1][1], matrix[i][2][1],
                          matrix[i][0][2], matrix[i][1][2], matrix[i][2][2];
    
    vRotationMatrix[i].transposeInPlace();

    // std::cout << "vRotationMatrix[i]:" << std::endl;
    // std::cout << vRotationMatrix[i] << std::endl;

    vQuaternion[i] = Eigen::Quaterniond(vRotationMatrix[i]);
    vQuaternion[i].normalize();

    vTransformationMatrix[i].rotate(vRotationMatrix[i]);
    vTransformationMatrix[i].pretranslate(Eigen::Vector3d((Body[i].Markers[0][0] / 1000.0),
                                                          (Body[i].Markers[0][1] / 1000.0),
                                                          (Body[i].Markers[0][2] / 1000.0)));
  }

  // std::cout << "Body[0].Markers[0][0]: " << Body[0].Markers[0][0] << std::endl;

	/****************if data is valid****************/
	if(Body[0].Markers[0][0] < 9999999.0)
	{
		/***********publish***********/
    geometry_msgs::PoseStamped visionPose;
		visionPose.header.stamp =  currentTime;
    visionPose.header.frame_id = "world";	
 		visionPose.pose.position.x = (Body[0].Markers[0][0] / 1000.0);				//x of Marker Back 	    					
 		visionPose.pose.position.y = (Body[0].Markers[0][1] / 1000.0);				//y of Marker Back
 		visionPose.pose.position.z = (Body[0].Markers[0][2] / 1000.0);				//z of Marker Back
		visionPose.pose.orientation.x = vQuaternion[0].x();
		visionPose.pose.orientation.y = vQuaternion[0].y();
		visionPose.pose.orientation.z = vQuaternion[0].z();
		visionPose.pose.orientation.w = vQuaternion[0].w();
    visionPosPub.publish(visionPose);

    // Orientation quaternion
    tf2::Quaternion q(vQuaternion[0].x(), vQuaternion[0].y(), vQuaternion[0].z(), vQuaternion[0].w());
    // 3x3 Rotation matrix from quaternion
    tf2::Matrix3x3 m(q);
    // Roll Pitch and Yaw from rotation matrix
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    #define RAD2DEG 57.295779513
    // Output the measure
    ROS_INFO("Rigid body 1 pose in world coordinate system: X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.9f Y: %.2f",
            (Body[0].Markers[0][0] / 1000.0), (Body[0].Markers[0][1] / 1000.0), (Body[0].Markers[0][2] / 1000.0),
            roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
	}
	/****************if data is invalid****************/
	else
		printf("Rigid body 1 data lost\n");

  if(FrameOfData->nBodies > 1)
  {
    	/****************if data is valid****************/
    if(Body[1].Markers[0][0] < 9999999.0)
    {
      /***********publish***********/
      geometry_msgs::PoseStamped visionPose2;
      visionPose2.header.stamp =  currentTime;
      visionPose2.header.frame_id = "world";			
      visionPose2.pose.position.x = (Body[1].Markers[0][0] / 1000.0);				//x of Marker Back 	    					
      visionPose2.pose.position.y = (Body[1].Markers[0][1] / 1000.0);				//y of Marker Back
      visionPose2.pose.position.z = (Body[1].Markers[0][2] / 1000.0);				//z of Marker Back
      visionPose2.pose.orientation.x = vQuaternion[1].x();
      visionPose2.pose.orientation.y = vQuaternion[1].y();
      visionPose2.pose.orientation.z = vQuaternion[1].z();
      visionPose2.pose.orientation.w = vQuaternion[1].w();
      visionPosPub2.publish(visionPose2);

      // Orientation quaternion
      tf2::Quaternion q(vQuaternion[1].x(), vQuaternion[1].y(), vQuaternion[1].z(), vQuaternion[1].w());
      // 3x3 Rotation matrix from quaternion
      tf2::Matrix3x3 m(q);
      // Roll Pitch and Yaw from rotation matrix
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      #define RAD2DEG 57.295779513
      // Output the measure
      ROS_INFO("Rigid body 2 pose in world coordinate system: X: %.2f Y: %.2f Z: %.2f - R: %.4f P: %.4f Y: %.4f",
              (Body[1].Markers[0][0] / 1000.0), (Body[1].Markers[0][1] / 1000.0), (Body[1].Markers[0][2] / 1000.0),
              roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
      ROS_INFO("Rigid body 2 quaternion in world coordinate system: q.x: %.2f q.y: %.2f q.z %.2f q.w %.2f",
                vQuaternion[1].x(), vQuaternion[1].y(), vQuaternion[1].z(), vQuaternion[1].w());
    }
    /****************if data is invalid****************/
    else
      printf("Rigid body 2 data lost\n");

    if((Body[0].Markers[0][0] < 9999999.0) && (Body[1].Markers[0][0] < 9999999.0))
    {
      // std::cout << "Body[0] and Body[1] data is valid!" << std::endl;

      // 刚体2在刚体1下位姿的变换矩阵
      Eigen::Isometry3d transformationMatrix21 = vTransformationMatrix[1] * vTransformationMatrix[0].inverse();
      Eigen::Matrix3d rotationMatrix21 = transformationMatrix21.matrix().block<3,3>(0,0);
      Eigen::Vector3d translationVector21 = transformationMatrix21.matrix().topRightCorner(3, 1);
      Eigen::Quaterniond quaternion21(rotationMatrix21);
      quaternion21.normalize();
      // std::cout << transformationMatrix21.matrix() << std::endl;  
      // std::cout << rotationMatrix21 << std::endl;
      // std::cout << translationVector21.transpose() << std::endl;


      /***********publish***********/
      geometry_msgs::PoseStamped visionPose21;
      visionPose21.header.stamp =  currentTime;
      visionPose21.header.frame_id = "local";			
      visionPose21.pose.position.x = translationVector21[0];	    					
      visionPose21.pose.position.y = translationVector21[1];
      visionPose21.pose.position.z = translationVector21[2];
      visionPose21.pose.orientation.x = quaternion21.x();
      visionPose21.pose.orientation.y = quaternion21.y();
      visionPose21.pose.orientation.z = quaternion21.z();
      visionPose21.pose.orientation.w = quaternion21.w();
      visionPosPub21.publish(visionPose21);

      // Orientation quaternion
      tf2::Quaternion q(quaternion21.x(), quaternion21.y(), quaternion21.z(), quaternion21.w());
      // 3x3 Rotation matrix from quaternion
      tf2::Matrix3x3 m(q);
      // Roll Pitch and Yaw from rotation matrix
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      #define RAD2DEG 57.295779513
      // Output the measure
      ROS_INFO("Rigid body 2 pose in rigid body 1 coordinate system: X: %.2f Y: %.2f Z: %.2f - R: %.2f P: %.9f Y: %.2f",
              translationVector21[0], translationVector21[1], translationVector21[2],
              roll * RAD2DEG, pitch * RAD2DEG, yaw * RAD2DEG);
    }
  }

#ifdef COMPILEDWITHC11
  std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
  std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

  double tCalculate= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

  vTimesCalculate.push_back(tCalculate);
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "nokov_ros_node");
  ros::NodeHandle nh;
  
  visionPosPub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose",10);
  visionPosPub2 = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose2",10);
  visionPosPub21 = nh.advertise<geometry_msgs::PoseStamped>("/mavros/vision_pose/pose21",10);
  
  int retval = RC_Okay;								// state message
  unsigned char SDK_Version[4];				// version
  sBodyDefs* pBodyDefs = NULL;							

  printf("Usage: ClientTest <Me> <Cortex>\n");
  printf("	Me = My machine name or its IP address\n");
  printf("	Cortex = Cortex' machine name or its IP Address\n");
  
  Cortex_SetVerbosityLevel(VL_Info);						
  Cortex_GetSdkVersion(SDK_Version);
  printf("Using SDK Version %d.%d.%d\n", SDK_Version[1], SDK_Version[2], SDK_Version[3]);

  Cortex_SetErrorMsgHandlerFunc(MyErrorMsgHandler);
  Cortex_SetDataHandlerFunc(MyDataHandler);					// call the MyDataHandler function

  printf("****** Cortex_Initialize ******\n");
  if (argc == 1) {
    retval = Cortex_Initialize("", NULL);
  } else if (argc == 2) {
    retval = Cortex_Initialize(argv[1], NULL);
  } else if (argc == 3) {
    retval = Cortex_Initialize(argv[1], argv[2]);
  }

  if (retval != RC_Okay) {
    printf("Error: Unable to initialize ethernet communication\n");
    retval = Cortex_Exit();
    return 1;
  }

  printf("****** Cortex_GetBodyDefs ******\n");
  pBodyDefs = Cortex_GetBodyDefs();

  if (pBodyDefs == NULL) {
    printf("Failed to get body defs\n");
  } else {
    printf("Got body defs\n");
    
    printf("nMarkers:%d,nDofs:%d\n",pBodyDefs->BodyDefs[0].nMarkers,pBodyDefs->BodyDefs[0].nDofs);
    
    Cortex_FreeBodyDefs(pBodyDefs);
    pBodyDefs = NULL;
  }

  void *pResponse;
  int nBytes;
  retval = Cortex_Request("GetContextFrameRate", &pResponse, &nBytes);
  if (retval != RC_Okay)
    printf("ERROR, GetContextFrameRate\n");

  float *contextFrameRate = (float*) pResponse;

  printf("ContextFrameRate = %3.1f Hz\n", *contextFrameRate);

  printf("*** Starting live mode ***\n");
  retval = Cortex_Request("LiveMode", &pResponse, &nBytes);

  ros::Rate loop_rate(30);
	while(ros::ok())
	{
		ros::spinOnce();
    loop_rate.sleep();
	}

  retval = Cortex_Request("Pause", &pResponse, &nBytes);
  printf("*** Paused live mode ***\n");

  printf("****** Cortex_Exit ******\n");
  retval = Cortex_Exit();

  // Calculating time statistics
  std::sort(vTimesCalculate.begin(), vTimesCalculate.end());
  double totaltime = 0.0;
  for(int ni = 0; ni < vTimesCalculate.size(); ni++)
  {
    totaltime += vTimesCalculate[ni];
  }
  std::cout << "-------" << std::endl << std::endl;
  std::cout << "received data number: " << vTimesCalculate.size() << std::endl;
  std::cout << "median calculating time: " << vTimesCalculate[vTimesCalculate.size() / 2] << std::endl;
  std::cout << "mean calculating time: " << totaltime / vTimesCalculate.size() << std::endl;

  return 0;
}
