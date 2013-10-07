#include "OpenNIContext.h"
#include <string>
#define SAMPLE_XML_PATH "../HandGestureRecognition/config/SamplesConfig.xml"
void XN_CALLBACK_TYPE myCalibrationInProgress(xn::SkeletonCapability&, XnUserID id, XnCalibrationStatus calibrationError, void*);
using namespace std;
inline XnStatus CHECK_RC(XnStatus nRetVal,string what)
{
	if (nRetVal != XN_STATUS_OK)
		printf("%s failed: %s\n", what.c_str(), xnGetStatusString(nRetVal));
	return nRetVal;
}

void XN_CALLBACK_TYPE User_NewUser(xn::UserGenerator &g_UserGenerator, XnUserID nId, void *)
{
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	printf("%d New User %d\n", epochTime, nId);
	// New user found
	g_UserGenerator.GetSkeletonCap().RequestCalibration(nId, TRUE);
}

void XN_CALLBACK_TYPE User_LostUser(xn::UserGenerator &g_UserGenerator, XnUserID nId, void *)
{
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	printf("%d Lost user %d\n", epochTime, nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationStart(xn::SkeletonCapability &, XnUserID nId, void *)
{
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	printf("%d Calibration started for user %d\n", epochTime, nId);
}

void XN_CALLBACK_TYPE UserCalibration_CalibrationComplete(xn::SkeletonCapability &skeletonCapability, XnUserID nId, XnCalibrationStatus eStatus, void *)
{
	XnUInt32 epochTime = 0;
	xnOSGetEpochTime(&epochTime);
	if (eStatus == XN_CALIBRATION_STATUS_OK)
	{
		printf("%d Calibration complete, start tracking user %d\n", epochTime, nId);
		skeletonCapability.StartTracking(nId);
	}
	else
	{
		printf("%d Calibration failed for user %d\n", epochTime, nId);
		if(eStatus==XN_CALIBRATION_STATUS_MANUAL_ABORT)
		{
			printf("Manual abort occured, stop attempting to calibrate!");
			return;
		}
		skeletonCapability.RequestCalibration(nId, TRUE);
	}
}

OpenNIContext::OpenNIContext(int argc, char **argv)
{
	XnStatus nRetVal = XN_STATUS_OK;
	if (argc > 1)
	{
		nRetVal = context.Init();
		CHECK_RC(nRetVal, "Init");
		nRetVal = context.OpenFileRecording(argv[1], player);
		if (nRetVal != XN_STATUS_OK)
		{
			printf("Can't open recording %s: %s\n", argv[1], xnGetStatusString(nRetVal));
			exit(1);
		}
	}
	else
	{
		xn::EnumerationErrors errors;
		nRetVal = context.InitFromXmlFile(SAMPLE_XML_PATH, scriptNode, &errors);
		if (nRetVal == XN_STATUS_NO_NODE_PRESENT)
		{
			XnChar strError[1024];
			errors.ToString(strError, 1024);
			printf("%s\n", strError);
			exit (nRetVal);
		}
		else if (nRetVal != XN_STATUS_OK)
		{
			printf("Open failed: %s\n", xnGetStatusString(nRetVal));
			exit (nRetVal);
		}
	}

	nRetVal = context.FindExistingNode(XN_NODE_TYPE_DEPTH, depthGenerator);
	context.FindExistingNode(XN_NODE_TYPE_IMAGE, imageGenerator);
	if (nRetVal != XN_STATUS_OK)
		printf("No depth generator found. Using a default one...");

	nRetVal = context.FindExistingNode(XN_NODE_TYPE_USER, userGenerator);
	if (nRetVal != XN_STATUS_OK)
	{
		nRetVal = userGenerator.Create(context);
		CHECK_RC(nRetVal, "Find user generator");
	}

	XnCallbackHandle hUserCallbacks, hCalibrationStart, hCalibrationComplete, hCalibrationInProgress;
	if (!userGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
	{
		printf("Supplied user generator doesn't support skeleton\n");
		exit (1);
	}
	nRetVal = userGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
	CHECK_RC(nRetVal, "Register to user callbacks");
	nRetVal = userGenerator.GetSkeletonCap().RegisterToCalibrationStart(UserCalibration_CalibrationStart, NULL, hCalibrationStart);
	CHECK_RC(nRetVal, "Register to calibration start");
	nRetVal = userGenerator.GetSkeletonCap().RegisterToCalibrationComplete(UserCalibration_CalibrationComplete, NULL, hCalibrationComplete);
	CHECK_RC(nRetVal, "Register to calibration complete");

	userGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	nRetVal = userGenerator.GetSkeletonCap().RegisterToCalibrationInProgress(myCalibrationInProgress, NULL, hCalibrationInProgress);
	CHECK_RC(nRetVal, "Register to calibration in progress");

	nRetVal = context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");
	sceneDrawer = new SceneDrawer(userGenerator, depthGenerator);
}

OpenNIContext::~OpenNIContext()
{
	scriptNode.Release();
	depthGenerator.Release();
	userGenerator.Release();
	imageGenerator.Release();
	player.Release();
	context.Release();
	delete sceneDrawer;
}

void OpenNIContext::display()
{
	depthGenerator.GetAlternativeViewPointCap().SetViewPoint(imageGenerator);
	xn::SceneMetaData sceneMD;
	xn::DepthMetaData depthMD;
	// Process the data
	depthGenerator.GetMetaData(depthMD);
	userGenerator.GetUserPixels(0, sceneMD);
	xn::ImageMetaData ImageMD;
	imageGenerator.GetMetaData(ImageMD);
	sceneDrawer->drawDepthMap(depthMD, sceneMD);
}

void OpenNIContext::update()
{
	// Read next available data
	context.WaitOneUpdateAll(userGenerator);
}

void OpenNIContext::getImageMap(cv::Mat &image)
{
	xn::ImageMetaData ImageMD;
	imageGenerator.GetMetaData(ImageMD);
	const XnUInt8* img = ImageMD.Data();
	cv::Mat imgBuf(480,640,CV_8UC3,(unsigned short*)img);
	cv::cvtColor( imgBuf, image, CV_RGB2BGR );
}

void OpenNIContext::getHandsPositions(std::vector<std::pair<std::pair<int, int>, std::pair<int, int> > > &positions)
{
	XnUserID aUsers[15];
	XnUInt16 nUsers = 15;
	userGenerator.GetUsers(aUsers, nUsers);
	XnSkeletonJoint hand[] = {XN_SKEL_LEFT_HAND, XN_SKEL_RIGHT_HAND};
	for (int i = 0; i < nUsers; ++i)
		if (userGenerator.GetSkeletonCap().IsTracking(aUsers[i]))
		{
			pair<pair<int, int>, pair<int, int> > position;
			for (int j = 0; j < 2; j++)
				if (userGenerator.GetSkeletonCap().IsJointActive(hand[j]))
				{
					XnSkeletonJointPosition joint;
					userGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], hand[j], joint);
					if (joint.fConfidence >= 0.5)
					{
						XnPoint3D pt;
						pt = joint.position;
						depthGenerator.ConvertRealWorldToProjective(1, &pt, &pt);
						if (!j)
						{
							position.first.first = pt.Y;
							position.first.second = pt.X;
						}
						else
						{
							position.second.first = pt.Y;
							position.second.second = pt.X;
						}
					}
				}
			positions.push_back(position);
		}
}
