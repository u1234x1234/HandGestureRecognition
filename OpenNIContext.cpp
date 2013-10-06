#include "OpenNIContext.h"
#include <string>
#define SAMPLE_XML_PATH "../HandGestureRecognition/config/SamplesConfig.xml"
void XN_CALLBACK_TYPE MyCalibrationInProgress(xn::SkeletonCapability&, XnUserID id, XnCalibrationStatus calibrationError, void*);
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
		nRetVal = g_Context.Init();
		CHECK_RC(nRetVal, "Init");
		nRetVal = g_Context.OpenFileRecording(argv[1], g_Player);
		if (nRetVal != XN_STATUS_OK)
		{
			printf("Can't open recording %s: %s\n", argv[1], xnGetStatusString(nRetVal));
			exit(1);
		}
	}
	else
	{
		xn::EnumerationErrors errors;
		nRetVal = g_Context.InitFromXmlFile(SAMPLE_XML_PATH, g_scriptNode, &errors);
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

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_DEPTH, g_DepthGenerator);
	g_Context.FindExistingNode(XN_NODE_TYPE_IMAGE, g_ImageGenerator);
	if (nRetVal != XN_STATUS_OK)
		printf("No depth generator found. Using a default one...");

	nRetVal = g_Context.FindExistingNode(XN_NODE_TYPE_USER, g_UserGenerator);
	if (nRetVal != XN_STATUS_OK)
	{
		nRetVal = g_UserGenerator.Create(g_Context);
		CHECK_RC(nRetVal, "Find user generator");
	}

	XnCallbackHandle hUserCallbacks, hCalibrationStart, hCalibrationComplete, hCalibrationInProgress;
	if (!g_UserGenerator.IsCapabilitySupported(XN_CAPABILITY_SKELETON))
	{
		printf("Supplied user generator doesn't support skeleton\n");
		exit (1);
	}
	nRetVal = g_UserGenerator.RegisterUserCallbacks(User_NewUser, User_LostUser, NULL, hUserCallbacks);
	CHECK_RC(nRetVal, "Register to user callbacks");
	nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationStart(UserCalibration_CalibrationStart, NULL, hCalibrationStart);
	CHECK_RC(nRetVal, "Register to calibration start");
	nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationComplete(UserCalibration_CalibrationComplete, NULL, hCalibrationComplete);
	CHECK_RC(nRetVal, "Register to calibration complete");

	g_UserGenerator.GetSkeletonCap().SetSkeletonProfile(XN_SKEL_PROFILE_ALL);

	nRetVal = g_UserGenerator.GetSkeletonCap().RegisterToCalibrationInProgress(MyCalibrationInProgress, NULL, hCalibrationInProgress);
	CHECK_RC(nRetVal, "Register to calibration in progress");

	nRetVal = g_Context.StartGeneratingAll();
	CHECK_RC(nRetVal, "StartGenerating");
	sceneDrawer = new SceneDrawer(g_UserGenerator, g_DepthGenerator);
}

OpenNIContext::~OpenNIContext()
{
	g_scriptNode.Release();
	g_DepthGenerator.Release();
	g_UserGenerator.Release();
	g_ImageGenerator.Release();
	g_Player.Release();
	g_Context.Release();
	delete sceneDrawer;
}

void OpenNIContext::display()
{
	g_DepthGenerator.GetAlternativeViewPointCap().SetViewPoint(g_ImageGenerator);
	xn::SceneMetaData sceneMD;
	xn::DepthMetaData depthMD;
	// Read next available data
	g_Context.WaitOneUpdateAll(g_UserGenerator);
	// Process the data
	g_DepthGenerator.GetMetaData(depthMD);
	g_UserGenerator.GetUserPixels(0, sceneMD);
	xn::ImageMetaData ImageMD;
	g_ImageGenerator.GetMetaData(ImageMD);
	sceneDrawer->DrawDepthMap(depthMD, sceneMD);
}
