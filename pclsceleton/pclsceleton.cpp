#include <iostream>
#include <string>
#include <sstream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/openni_grabber.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/octree/octree.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/common/time.h>
#include "oscpack/ip/UdpSocket.h"
#include "oscpack/osc/OscOutboundPacketStream.h"
#include <pcl/filters/passthrough.h>
#include <pcl/io/openni_grabber.h>
#include "Stroke.h"
#include <fstream>
#include <vector>
#include <math.h>
#include <time.h>

using namespace std;

char *ADDRESS = "127.0.0.1";
int PORT = 7110;

#define OUTPUT_BUFFER_SIZE 1024
char osc_buffer[OUTPUT_BUFFER_SIZE];
UdpTransmitSocket *transmitSocket;

//char oscPositionAddressBuff[24][64];
//char oscOrientationAddressBuff[24][64];

//int userID;
//float jointOrients[9];
char tmp[50];
float jointCoords[4]; //Fix for MacOSX's crazy gcc (was overwriting userGenerator with the last coord apparently...)

//float posConfidence;
//float orientConfidence;
//Multipliers for coordinate system. This is useful if you use software like animata,
//that needs OSC messages to use an arbitrary coordinate system.
double mult_x = 1;
double mult_y = 1;
double mult_z = 1;

//int handID;
//float handCoords[3];
//bool haveHand = false;

//Offsets for coordinate system. This is useful if you use software like animata,
//that needs OSC messages to use an arbitrary coordinate system.
double off_x = 0.0;
double off_y = 0.0;
double off_z = 0.0;

bool kitchenMode = true;
//bool mirrorMode = false;
//bool kitechenMode = false;
//bool handMode = false;
//bool play = false;
//bool record = false;
//bool sendRot = false;
//bool filter = false;
//bool raw = false;
//bool preview = false;
//bool filterLowConfidence = false;
//bool realworld = false;
//bool useRealTimeClock = true;
//bool sendOrient = false;
//bool handTime = false;
int nDimensions = 3;

char outputFileStr[1024];
//bool outputFileOpen = false;
//std::ofstream outputFile;

//void (*oscFunc)(lo_bundle*, char*, int) = NULL;

//xn::Context context;
//xn::DepthGenerator depth;
//xn::DepthMetaData depthMD;
//xn::HandsGenerator handsGenerator;
xn::UserGenerator userGenerator;
//xn::GestureGenerator gestureGenerator;
//lo_address addr;

XnChar g_strPose[20] = "";

void checkRetVal(XnStatus nRetVal) {
  if (nRetVal != XN_STATUS_OK) {
    printf("There was a problem initializing kinect... Make sure you have\
connected both usb and power cables and that the driver and OpenNI framework\
are correctly installed.\n\n");
    exit(1);
  }
}


// Callback: New user was detected
void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
  printf("New User %d\n", nId);
  userGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);

  osc::OutboundPacketStream p( osc_buffer, OUTPUT_BUFFER_SIZE );
  p << osc::BeginBundleImmediate;
  p << osc::BeginMessage( "/new_user" );
  p << (int)nId;
  p << osc::EndMessage;
  p << osc::EndBundle;
  transmitSocket->Send(p.Data(), p.Size());
}



// Callback: An existing user was lost
void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator& generator, XnUserID nId, void* pCookie) {
  printf("Lost user %d\n", nId);

  if (kitchenMode) return;

  osc::OutboundPacketStream p( osc_buffer, OUTPUT_BUFFER_SIZE );
  p << osc::BeginBundleImmediate;
  p << osc::BeginMessage( "/lost_user" );
  p << (int)nId;
  p << osc::EndMessage;
  p << osc::EndBundle;
  transmitSocket->Send(p.Data(), p.Size());
}

// Callback: Detected a pose
void XN_CALLBACK_TYPE UserPose_PoseDetected(xn::PoseDetectionCapability& capability, const XnChar* strPose, XnUserID nId, void* pCookie) {
  printf("Pose %s detected for user %d\n", strPose, nId);
  userGenerator.GetPoseDetectionCap().StopPoseDetection(nId);
  userGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}



// Callback: Started calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability& capability, XnUserID nId, void* pCookie) {
  printf("Calibration started for user %d\n", nId);
}



// Callback: Finished calibration
void XN_CALLBACK_TYPE UserCalibration_CalibrationEnd(xn::SkeletonCapability& capability, XnUserID nId, XnBool bSuccess, void* pCookie) {
  if (bSuccess) {
    printf("Calibration complete, start tracking user %d\n", nId);
    userGenerator.GetSkeletonCap().StartTracking(nId);

    if (kitchenMode) return;

    osc::OutboundPacketStream p( osc_buffer, OUTPUT_BUFFER_SIZE );
    p << osc::BeginBundleImmediate;
    p << osc::BeginMessage( "/new_skel" );
    p << (int)nId;
    p << osc::EndMessage;
    p << osc::EndBundle;
    transmitSocket->Send(p.Data(), p.Size());
  }
  else {
    printf("Calibration failed for user %d\n", nId);
    userGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
  }
}



int jointPos(XnUserID player, XnSkeletonJoint eJoint) {
  XnSkeletonJointPosition joint;
  userGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint, joint);

  if (joint.fConfidence < 0.5)
    return -1;

  jointCoords[0] = player;
  jointCoords[1] = off_x + (mult_x * (1280 - joint.position.X) / 2560); //Normalize coords to 0..1 interval
  jointCoords[2] = off_y + (mult_y * (1280 - joint.position.Y) / 2560); //Normalize coords to 0..1 interval
  jointCoords[3] = off_z + (mult_z * joint.position.Z * 7.8125 / 10000); //Normalize coords to 0..7.8125 interval
  //uncomment if you want native coordinates.	
 // 	jointCoords[1] = joint.position.X;
 // 	jointCoords[2] = joint.position.Y;
 // 	jointCoords[3] = joint.position.Z;

  return 0;
}



void genOscMsg(osc::OutboundPacketStream *p, char *name) {
  *p << osc::BeginMessage( "/joint" );
  *p << name;
  if (!kitchenMode)
    *p << (int)jointCoords[0];
  for (int i = 1; i < nDimensions+1; i++)
    *p << jointCoords[i];
  *p << osc::EndMessage;
}



/*void sendOSC(const xn::DepthMetaData& dmd)
{
  XnUserID aUsers[2];
  XnUInt16 nUsers = 2;
  userGenerator.GetUsers(aUsers, nUsers);
  for (int i = 0; i < nUsers; ++i)
    {
      if (userGenerator.GetSkeletonCap().IsTracking(aUsers[i])) {
	osc::OutboundPacketStream p( osc_buffer, OUTPUT_BUFFER_SIZE );
	p << osc::BeginBundleImmediate;

	if (jointPos(aUsers[i], XN_SKEL_HEAD) == 0) {
	  genOscMsg(&p, "head");
	}
	if (jointPos(aUsers[i], XN_SKEL_NECK) == 0) {
	  genOscMsg(&p, "neck");
	}
	if (jointPos(aUsers[i], XN_SKEL_LEFT_COLLAR) == 0) {
	  genOscMsg(&p, "l_collar");
	}
	if (jointPos(aUsers[i], XN_SKEL_LEFT_SHOULDER) == 0) {
	  genOscMsg(&p, "l_shoulder");
	}
	if (jointPos(aUsers[i], XN_SKEL_LEFT_ELBOW) == 0) {
	  genOscMsg(&p, "l_elbow");
	}
	if (jointPos(aUsers[i], XN_SKEL_LEFT_WRIST) == 0) {
	  genOscMsg(&p, "l_wrist");
	}
	if (jointPos(aUsers[i], XN_SKEL_LEFT_HAND) == 0) {
	  genOscMsg(&p, "l_hand");
	}
	if (jointPos(aUsers[i], XN_SKEL_LEFT_FINGERTIP) == 0) {
	  genOscMsg(&p, "l_fingertip");
	}
	if (jointPos(aUsers[i], XN_SKEL_RIGHT_COLLAR) == 0) {
	  genOscMsg(&p, "r_collar");
	}
	if (jointPos(aUsers[i], XN_SKEL_RIGHT_SHOULDER) == 0) {
	  genOscMsg(&p, "r_shoulder");
	}
	if (jointPos(aUsers[i], XN_SKEL_RIGHT_ELBOW) == 0) {
	  genOscMsg(&p, "r_elbow");
	}
	if (jointPos(aUsers[i], XN_SKEL_RIGHT_WRIST) == 0) {
	  genOscMsg(&p, "r_wrist");
	}
	if (jointPos(aUsers[i], XN_SKEL_RIGHT_HAND) == 0) {
	  genOscMsg(&p, "r_hand");
	}
	if (jointPos(aUsers[i], XN_SKEL_RIGHT_FINGERTIP) == 0) {
	  genOscMsg(&p, "r_fingertip");
	}
	if (jointPos(aUsers[i], XN_SKEL_TORSO) == 0) {
	  genOscMsg(&p, "torso");
	}
	if (jointPos(aUsers[i], XN_SKEL_WAIST) == 0) {
	  genOscMsg(&p, "waist");
	}
	if (jointPos(aUsers[i], XN_SKEL_LEFT_HIP) == 0) {
	  genOscMsg(&p, "l_hip");
	}
	if (jointPos(aUsers[i], XN_SKEL_LEFT_KNEE) == 0) {
	  genOscMsg(&p, "l_knee");
	}
	if (jointPos(aUsers[i], XN_SKEL_LEFT_ANKLE) == 0) {
	  genOscMsg(&p, "l_ankle");
	}
	if (jointPos(aUsers[i], XN_SKEL_LEFT_FOOT) == 0) {
	  genOscMsg(&p, "l_foot");
	}
	if (jointPos(aUsers[i], XN_SKEL_RIGHT_HIP) == 0) {
	  genOscMsg(&p, "r_hip");
	}
	if (jointPos(aUsers[i], XN_SKEL_RIGHT_KNEE) == 0) {
	  genOscMsg(&p, "r_knee");
	}
	if (jointPos(aUsers[i], XN_SKEL_RIGHT_ANKLE) == 0) {
	  genOscMsg(&p, "r_ankle");
	}
	if (jointPos(aUsers[i], XN_SKEL_RIGHT_FOOT) == 0) {
	  genOscMsg(&p, "r_foot");
	}

	p << osc::EndBundle;
	transmitSocket->Send(p.Data(), p.Size());
      }
    }
}
*/

void viewerAnnotation (pcl::visualization::PCLVisualizer& viewer) {
// viewer.addCoordinateSystem (0.1);    
// XnSkeletonJointPosition joint;
//
 

  //text annotation of joints? 
/*  static unsigned count = 0;
  std::stringstream ss;
  ss << "Hand?: " << count++;
  viewer.removeShape ("text", 0);  
   if (joint.fConfidence < 0.5)
 {
  viewer.addText (ss.str(), joint.position.X, joint.position.Y, "text", 0);
//  viewer.setBackgroundColor (0.0 ,1.0,0.0);
  /*pcl::PointXYZ o;
  o.x = 1.0;
  o.y = 0;
  o.z = 0;
//viewer.addSphere(o,0.25, "sphere" , 0);

 // viewer.addText3D (ss.str(), pclPoint, 2.0, 0.0, 255.0, 0.0, "text", 0);	
  */
//}
	
	
  //	pcl::PointXYZ p1, p2;
//	p1.x=0.0; p1.y=0.0; p1.z=0.0;
//	p2.x=1.0; p2.y=1.0; p2.z=1.0;
//	viewer.addCoordinateSystem(0.025);
//	viewer.addLine(p1, p2, 0.0, 255.0, 0.0,"line1", 0); 
  	
  viewer.removeShape ("headneck", 0);  
  viewer.removeShape ("necktorso", 0);  
  viewer.removeShape ("torsowaist", 0);  	
  viewer.removeShape ("neckrshoulder", 0);  		
  viewer.removeShape ("rshoulderrelbow", 0);  		
  viewer.removeShape ("relbowrhand", 0);  		
  viewer.removeShape ("rhandrfinger", 0);  			
  viewer.removeShape ("lhandlfinger", 0);
  viewer.removeShape ("necklshoulder", 0);
  viewer.removeShape ("lshoulderlelbow", 0);  		
  viewer.removeShape ("lelbowlhand", 0);  			
 // viewer.removeShape ("waistlhip", 0);
 // viewer.removeShape ("waistrhip", 0);
   viewer.removeShape ("rshoulderrhip", 0);
    viewer.removeShape ("lshoulderlhip", 0);
 viewer.removeShape ("rhiprknee", 0);
  viewer.removeShape ("lhiplknee", 0);
 viewer.removeShape ("lhiprhip", 0);
 viewer.removeShape ("lkneelfoot", 0);
  viewer.removeShape ("rkneerfoot", 0);

   vector<NodeData> jointPoints; 


  //add the skeleton outline	
  if (userGenerator.GetSkeletonCap().IsTracking(1)) {    
    XnSkeletonJointPosition joint;

    userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_HEAD, joint);		
    pcl::PointXYZRGBA pclHeadPoint;
    //pcl::PointXYZ o
    pclHeadPoint.x=joint.position.X/1000;
    pclHeadPoint.y=-1*joint.position.Y/1000;
    pclHeadPoint.z=joint.position.Z/1000;
    printf("user %d: head at (%6.2f, %6.2f , %6.2f)\n" , 1, pclHeadPoint.x,
 							  pclHeadPoint.y,
							  pclHeadPoint.z);
    //o.x = 1.0;
    //o.y = 0.0;
    //o.z = 0.0;
  //  viewer.addSphere (pclHeadPoint ,0.05 , "sphere" ,0);
		
    userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_NECK, joint);
    pcl::PointXYZRGBA pclNeckPoint;
    pclNeckPoint.x=joint.position.X/1000;
    pclNeckPoint.y=-1*joint.position.Y/1000;
    pclNeckPoint.z=joint.position.Z/1000;				
    NodeData NeckPoint(joint.position.X, joint.position.Y, joint.position.Z, Neither, Neck, time(NULL));
    cout << "User 1 :" << NeckPoint.getSide() << " " << NeckPoint.getLimb() << " X: " << NeckPoint.X() << " Y: " << NeckPoint.Y() << " Z: " <<  NeckPoint.Z() << "\n"; 
   jointPoints.push_back(NeckPoint);  
      printf("user %d: neck at (%6.2f, %6.2f , %6.2f)\n" , 1, pclNeckPoint.x,
                                                          pclNeckPoint.y,
                                                          pclNeckPoint.z);
    // viewer.addSphere (pclNeckPoint ,0.01 , "sphere" ,0);
    
    userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_WAIST, joint);
    pcl::PointXYZRGBA pclWaistPoint;
    pclWaistPoint.x=joint.position.X/1000;
    pclWaistPoint.y=-1*joint.position.Y/1000;
    pclWaistPoint.z=joint.position.Z/1000;	
    NodeData WaistPoint(joint.position.X, joint.position.Y, joint.position.Z, Neither, Waist, time(NULL));
    cout << "User 1 :" << WaistPoint.getSide() << " " << WaistPoint.getLimb() << " X: " << WaistPoint.X() << " Y: " << WaistPoint.Y() << " Z: " <<  WaistPoint.Z() << "\n"; 
   jointPoints.push_back(WaistPoint);  
 
      printf("user %d: waist at (%6.2f, %6.2f , %6.2f)\n" , 1, pclWaistPoint.x,
                                                          pclWaistPoint.y,
                                                          pclWaistPoint.z);

    //viewer.addSphere (pclWaistPoint ,0.01 , "sphere" ,0);		
   
    userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_TORSO, joint);
    pcl::PointXYZRGBA pclTorsoPoint;
    pclTorsoPoint.x=joint.position.X/1000;
    pclTorsoPoint.y=-1*joint.position.Y/1000;
    pclTorsoPoint.z=joint.position.Z/1000;				
    NodeData TorsoPoint(joint.position.X, joint.position.Y, joint.position.Z, Neither, Torso, time(NULL));
    jointPoints.push_back(TorsoPoint);  
 
      printf("user %d: torso at (%6.2f, %6.2f , %6.2f)\n" , 1, pclTorsoPoint.x,
                                                          pclTorsoPoint.y,
                                                          pclTorsoPoint.z);
    

    userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_LEFT_SHOULDER, joint);		
    pcl::PointXYZRGBA pclLShoulderPoint;
    pclLShoulderPoint.x=joint.position.X/1000;
    pclLShoulderPoint.y=-1*joint.position.Y/1000;
    pclLShoulderPoint.z=joint.position.Z/1000;				
    //viewer.addSphere (pclLShoulderPoint ,0.01 , "sphere" ,0);
    NodeData LShouldPoint(joint.position.X, joint.position.Y, joint.position.Z, Left, Shoulder, time(NULL));
    jointPoints.push_back(LShouldPoint); 
      printf("user %d: LeftShoulder at (%6.2f, %6.2f , %6.2f)\n" , 1, pclLShoulderPoint.x,
                                                          pclLShoulderPoint.y,
                                                          pclLShoulderPoint.z);

    
   userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_RIGHT_SHOULDER, joint);		
    pcl::PointXYZRGBA pclRShoulderPoint;
    pclRShoulderPoint.x=joint.position.X/1000;
    pclRShoulderPoint.y=-1*joint.position.Y/1000;
    pclRShoulderPoint.z=joint.position.Z/1000;				
    NodeData RShouldPoint(joint.position.X, joint.position.Y, joint.position.Z, Right, Shoulder, time(NULL));
    jointPoints.push_back(RShouldPoint); 
      printf("user %d:Right shoulder at (%6.2f, %6.2f , %6.2f)\n" , 1, pclRShoulderPoint.x,
                                                          pclRShoulderPoint.y,
                                                          pclRShoulderPoint.z);
    

    userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_RIGHT_ELBOW, joint);
    pcl::PointXYZRGBA pclRElbowPoint;
    pclRElbowPoint.x=joint.position.X/1000;
    pclRElbowPoint.y=-1*joint.position.Y/1000;
    pclRElbowPoint.z=joint.position.Z/1000;
    NodeData RElbowPoint(joint.position.X, joint.position.Y, joint.position.Z, Right, Elbow, time(NULL));
    jointPoints.push_back(RElbowPoint); 
 
      printf("user %d: Right Elbow at (%6.2f, %6.2f , %6.2f)\n" , 1, pclRElbowPoint.x,
                                                          pclRElbowPoint.y,
                                                          pclRElbowPoint.z);
   
 		
    userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_RIGHT_HAND, joint);
    pcl::PointXYZRGBA pclRHandPoint;
    pclRHandPoint.x=joint.position.X/1000;
    pclRHandPoint.y=-1*joint.position.Y/1000;
    pclRHandPoint.z=joint.position.Z/1000 ;
    NodeData RHandPoint(joint.position.X, joint.position.Y, joint.position.Z, Right, Hand, time(NULL));
    jointPoints.push_back(RHandPoint); 
      printf("user %d: Right Hand at (%6.2f, %6.2f , %6.2f)\n" , 1, pclRHandPoint.x,
                                                          pclRHandPoint.y,
                                                          pclRHandPoint.z);
    

    userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_RIGHT_FINGERTIP, joint);
    pcl::PointXYZRGBA pclRFingerPoint;
    pclRFingerPoint.x=joint.position.X/1000;
    pclRFingerPoint.y=-1*joint.position.Y/1000;
    pclRFingerPoint.z=joint.position.Z/1000 ;
//    NodeData RFingerPoint(joint.position.X, joint.position.Y, joint.position.Z, Right, Finger, time(NULL));
//    jointPoints.push_back(RHandPoint); 
 
      printf("user %d: Right Fingertip at (%6.2f, %6.2f , %6.2f)\n" , 1, pclRFingerPoint.x,
                                                          pclRFingerPoint.y,
                                                          pclRFingerPoint.z);


    userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_LEFT_FINGERTIP, joint);
    pcl::PointXYZRGBA pclLFingerPoint;
    pclLFingerPoint.x=joint.position.X/1000;
    pclLFingerPoint.y=-1*joint.position.Y/1000;
    pclLFingerPoint.z=joint.position.Z/1000 ;

     printf("user %d: Left Fingertip at (%6.2f, %6.2f , %6.2f)\n" , 1, pclLFingerPoint.x,
                                                          pclLFingerPoint.y,
                                                          pclLFingerPoint.z);



    userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_LEFT_ELBOW, joint);
    pcl::PointXYZRGBA pclLElbowPoint;
    pclLElbowPoint.x=joint.position.X/1000;
    pclLElbowPoint.y=-1*joint.position.Y/1000;
    pclLElbowPoint.z=joint.position.Z/1000;
    NodeData LElbowPoint(joint.position.X, joint.position.Y, joint.position.Z, Left, Elbow, time(NULL));
    jointPoints.push_back(LElbowPoint); 
 
      printf("user %d: Left Elbow at (%6.2f, %6.2f , %6.2f)\n" , 1, pclLElbowPoint.x,
                                                          pclLElbowPoint.y,
                                                          pclLElbowPoint.z);

 
		
    userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_LEFT_HAND, joint);
    pcl::PointXYZRGBA pclLHandPoint;
    pclLHandPoint.x=joint.position.X/1000;
    pclLHandPoint.y=-1*joint.position.Y/1000;
    pclLHandPoint.z=joint.position.Z/1000 ;
    NodeData LHandPoint(joint.position.X, joint.position.Y, joint.position.Z, Left, Hand, time(NULL));
    jointPoints.push_back(LHandPoint); 
      printf("user %d: Left Hand at (%6.2f, %6.2f , %6.2f)\n" , 1, pclLHandPoint.x,
                                                          pclLHandPoint.y,
                                                          pclLHandPoint.z);


    //	cout << "ZPos: "  << pclLHandPoint.z << endl;

    userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_WAIST, joint);
    pcl::PointXYZRGBA pclLWaistPoint;
    pclLWaistPoint.x=joint.position.X/1000;
    pclLWaistPoint.y=-1*joint.position.Y/1000;
    pclLWaistPoint.z=joint.position.Z/1000 ;
    NodeData LWaistPoint(joint.position.X, joint.position.Y, joint.position.Z, Left, Waist, time(NULL));
    jointPoints.push_back(LWaistPoint); 
      printf("user %d: Left Waist at (%6.2f, %6.2f , %6.2f)\n" , 1, pclLWaistPoint.x,
                                                          pclLWaistPoint.y,
                                                          pclLWaistPoint.z);



    userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_LEFT_HIP, joint);
    pcl::PointXYZRGBA pclLHipPoint;
    pclLHipPoint.x=joint.position.X/1000;
    pclLHipPoint.y=-1*joint.position.Y/1000;
    pclLHipPoint.z=joint.position.Z/1000 ;
    NodeData LHipPoint(joint.position.X, joint.position.Y, joint.position.Z, Left, Hip, time(NULL));
    jointPoints.push_back(LHipPoint); 
      printf("user %d: Left Hip at (%6.2f, %6.2f , %6.2f)\n" , 1, pclLHipPoint.x,
                                                          pclLHipPoint.y,
                                                          pclLHipPoint.z);

    userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_LEFT_KNEE, joint);
    pcl::PointXYZRGBA pclLKneePoint;
    pclLKneePoint.x=joint.position.X/1000;
    pclLKneePoint.y=-1*joint.position.Y/1000;
    pclLKneePoint.z=joint.position.Z/1000 ;
    NodeData LKneePoint(joint.position.X, joint.position.Y, joint.position.Z, Left, Knee, time(NULL));
    jointPoints.push_back(LKneePoint); 
      printf("user %d: Left Knee at (%6.2f, %6.2f , %6.2f)\n" , 1, pclLKneePoint.x,
                                                          pclLKneePoint.y,
                                                          pclLKneePoint.z);



    userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_LEFT_ANKLE, joint);
    pcl::PointXYZRGBA pclLAnklePoint;
    pclLAnklePoint.x=joint.position.X/1000;
    pclLAnklePoint.y=-1*joint.position.Y/1000;
    pclLAnklePoint.z=joint.position.Z/1000 ;
    NodeData LAnklePoint(joint.position.X, joint.position.Y, joint.position.Z, Left, Ankle, time(NULL));
    jointPoints.push_back(LAnklePoint); 
     printf("user %d: Left Ankle at (%6.2f, %6.2f , %6.2f)\n" , 1, pclLAnklePoint.x,
                                                          pclLAnklePoint.y,
                                                          pclLAnklePoint.z);



    userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_LEFT_FOOT, joint);
    pcl::PointXYZRGBA pclLFootPoint;
    pclLFootPoint.x=joint.position.X/1000;
    pclLFootPoint.y=-1*joint.position.Y/1000;
    pclLFootPoint.z=joint.position.Z/1000 ;
    NodeData LFootPoint(joint.position.X, joint.position.Y, joint.position.Z, Left, Foot, time(NULL));
    jointPoints.push_back(LFootPoint); 
      printf("user %d: Left Foot at (%6.2f, %6.2f , %6.2f)\n" , 1, pclLFootPoint.x,
                                                          pclLFootPoint.y,
                                                          pclLFootPoint.z);



    userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_RIGHT_HIP, joint);
    pcl::PointXYZRGBA pclRHipPoint;
    //time_t timer;
    pclRHipPoint.x=joint.position.X/1000;
    pclRHipPoint.y=-1*joint.position.Y/1000;
    pclRHipPoint.z=joint.position.Z/1000 ;
    NodeData RHipPoint(joint.position.X, joint.position.Y, joint.position.Z, Right, Hip, time(NULL));
    jointPoints.push_back(RHipPoint);
    cout << "User 1 :" << RHipPoint.getSide() << " " << RHipPoint.getLimb() << " X: " << RHipPoint.X() << " Y: " << RHipPoint.Y() << " Z: " <<  RHipPoint.Z() << "\n";  
      //printf("user %d: %d %d  at (%6.2f, %6.2f , %6.2f)\n" , 1, HipPoint.getSide(), HipPoint.getLimb(), HipPoint.X(),
      //                                                    HipPoint.Y(),
      //                                                    HipPoint.Z());



    userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_RIGHT_KNEE, joint);
    pcl::PointXYZRGBA pclRKneePoint;
    pclRKneePoint.x=joint.position.X/1000;
    pclRKneePoint.y=-1*joint.position.Y/1000;
    pclRKneePoint.z=joint.position.Z/1000 ;
    NodeData RKneePoint(joint.position.X, joint.position.Y, joint.position.Z, Right, Knee, time(NULL));
    jointPoints.push_back(RKneePoint); 
      printf("user %d: Right Knee at (%6.2f, %6.2f , %6.2f)\n" , 1, pclRKneePoint.x,
                                                          pclRKneePoint.y,
                                                          pclRKneePoint.z);
    

    userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_RIGHT_ANKLE, joint);
    pcl::PointXYZRGBA pclRAnklePoint;
    pclRAnklePoint.x=joint.position.X/1000;
    pclRAnklePoint.y=-1*joint.position.Y/1000;
    pclRAnklePoint.z=joint.position.Z/1000 ;
    NodeData RAnklePoint(joint.position.X, joint.position.Y, joint.position.Z, Right, Ankle, time(NULL));
    jointPoints.push_back(RAnklePoint); 
      printf("user %d: Right Ankle at (%6.2f, %6.2f , %6.2f)\n" , 1, pclRAnklePoint.x,
                                                          pclRAnklePoint.y,
                                                          pclRAnklePoint.z);


    userGenerator.GetSkeletonCap().GetSkeletonJointPosition(1, XN_SKEL_RIGHT_FOOT, joint);
    pcl::PointXYZRGBA pclRFootPoint;
    pclRFootPoint.x=joint.position.X/1000;
    pclRFootPoint.y=-1*joint.position.Y/1000;
    pclRFootPoint.z=joint.position.Z/1000 ;
    NodeData RFootPoint(joint.position.X, joint.position.Y, joint.position.Z, Right, Foot, time(NULL));
    jointPoints.push_back(RFootPoint); 
      printf("user %d: Right Foot at (%6.2f, %6.2f , %6.2f)\n" , 1, pclRFootPoint.x,
                                                          pclRFootPoint.y,
                                                          pclRFootPoint.z);

    vector<int> Angles = fillAngles(Right, Arm, jointPoints);
    stringstream ss;
    ss << Angles[0]; 
    viewer.addText (ss.str(), 0, 0, "text", 0);
    viewer.addLine(pclHeadPoint, pclNeckPoint, 255,0,0, "headneck", 0); 
    viewer.addLine(pclNeckPoint, pclTorsoPoint, 255,0,0, "necktorso", 0); 
   // viewer.addLine(pclTorsoPoint, pclWaistPoint, 255,0, 0, "torsowaist", 0); 
    viewer.addLine(pclNeckPoint, pclRShoulderPoint, 255,0,0, "neckrshoulder", 0); 
    viewer.addLine(pclNeckPoint, pclLShoulderPoint, 255,0, 0, "necklshoulder", 0);
    viewer.addLine(pclRShoulderPoint, pclRElbowPoint, 255, 0, 0, "rshoulderrelbow", 0);
    viewer.addLine(pclLShoulderPoint, pclLElbowPoint, 255, 0, 0, "lshoulderlelbow", 0);
     viewer.addLine(pclLShoulderPoint, pclLHipPoint, 255, 0, 0, "lshoulderlhip" , 0);
     viewer.addLine(pclRShoulderPoint, pclRHipPoint, 255, 0, 0, "rshoulderrhip", 0);
     viewer.addLine(pclRElbowPoint, pclRHandPoint, 255, 0 , 0, "relbowrhand", 0);
    //		viewer.addLine(pclRHandPoint,pclRFingerPoint, 0.0, 255.0, 0.0,"rhandrfinger", 0); 
    viewer.addLine(pclLElbowPoint, pclLHandPoint, 255, 0, 0, "lelbowlhand", 0);
    //      viewer.addLine(pclLHandPoint,pclLFingerPoint, 0.0, 255.0, 0.0,"lhandlfinger", 0);
   // viewer.addLine(pclWaistPoint, pclLHipPoint, 255, 0, 0, "waistlhip", 0); 
   // viewer.addLine(pclWaistPoint, pclRHipPoint, 255, 0, 0, "waistrhip", 0); 
    viewer.addLine(pclRHipPoint, pclRKneePoint, 255, 0, 0, "rhiprknee", 0);
    viewer.addLine(pclLHipPoint, pclLKneePoint, 255, 0, 0, "lhiplknee", 0);
    viewer.addLine(pclLHipPoint, pclRHipPoint, 255, 0, 0, "lhiprhip", 0);
    viewer.addLine(pclLKneePoint, pclLFootPoint, 255, 0, 0, "lkneelfoot", 0);
    viewer.addLine(pclRKneePoint, pclRFootPoint, 255, 0, 0, "rkneerfoot", 0);
  }
	
}		

class SimpleOpenNIViewer
{
public:
  SimpleOpenNIViewer () : viewer ("PCL OpenNI Viewer") {}


  void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
  {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>);     
    pcl::PassThrough<pcl::PointXYZRGBA> pass;	
    if (!viewer.wasStopped()){
     // pass.setInputCloud (cloud);
      pass.setFilterFieldName ("z");
      pass.setFilterLimits (0.0, 3.0);
      pass.setFilterLimitsNegative (true);
      pass.filter (*cloud_filtered);
    }
    viewer.showCloud (cloud_filtered);
    viewer.runOnVisualizationThread (viewerAnnotation);
  }

  void run ()
  {
    pcl::Grabber* interface = new pcl::OpenNIGrabber();
    boost::function<void (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f =boost::bind (&SimpleOpenNIViewer::cloud_cb_, this, _1);
    interface->registerCallback (f);
    interface->start ();
    //openni  code.       
    xn::Context context;
    xn::DepthGenerator depth;

    context.Init();
    depth.Create(context);
    XnMapOutputMode mapMode;
    //uncomment on slower boxen for lower res		
    mapMode.nXRes = XN_VGA_X_RES;
    mapMode.nYRes = XN_VGA_Y_RES;
   // mapMode.nXRes = XN_SVGA_X_RES;
   // mapMode.nYRes = XN_SVGA_Y_RES;
    mapMode.nFPS = 30;
    depth.SetMapOutputMode(mapMode);
    XnStatus nRetVal = XN_STATUS_OK;
    nRetVal = context.FindExistingNode(XN_NODE_TYPE_USER, userGenerator);
    if (nRetVal != XN_STATUS_OK)
      nRetVal = userGenerator.Create(context);		

 XnCallbackHandle hUserCallbacks, hCalibrationCallbacks, hPoseCallbacks;
    checkRetVal(userGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks));
    checkRetVal(userGenerator.GetSkeletonCap().RegisterCalibrationCallbacks(UserCalibration_CalibrationStart, UserCalibration_CalibrationEnd, NULL, hCalibrationCallbacks));
    checkRetVal(userGenerator.GetPoseDetectionCap().RegisterToPoseCallbacks(UserPose_PoseDetected, NULL, NULL, hPoseCallbacks));
    checkRetVal(userGenerator.GetSkeletonCap().GetCalibrationPose(g_strPose));
    checkRetVal(userGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL));	

    context.StartGeneratingAll();
    transmitSocket = new UdpTransmitSocket(IpEndpointName(ADDRESS, PORT));
    cout << "openni/osceleton portion done..pcl portion starting\n" ;
    while (!viewer.wasStopped())
      {
	// Read next available data
	// boost::this_thread::sleep (boost::posix_time::seconds (1));
	context.WaitAnyUpdateAll();
	// Process the data
	xn::DepthMetaData depthMD;
        depth.GetMetaData(depthMD);
               
      }

    interface->stop ();
    context.Release();
  }

  pcl::visualization::CloudViewer viewer;
 //pcl::visualization::PCLVisualizer viewer;
 // viewer.addCoordinateSystem(1.0,0);	 
};

int main ()
{

  SimpleOpenNIViewer v;
  v.run ();
  /*Therapy therapyType = Stroke;
	Side side = Right;
	Type strokeType = MotorArm;

	int score = 99;

	score = getScore( therapyType, side, strokeType);

	cout << "Score = " << score << endl;
	//cout << "Press enter when ready....";
	getchar();
*/
  return 0;
}

