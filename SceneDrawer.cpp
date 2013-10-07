#include "SceneDrawer.h"
#include <map>

using namespace cv;

std::map<XnUInt32, std::pair<XnCalibrationStatus, XnPoseDetectionStatus> > errors;
void XN_CALLBACK_TYPE myCalibrationInProgress(xn::SkeletonCapability& /*capability*/, XnUserID id, XnCalibrationStatus calibrationError, void* /*pCookie*/)
{
	errors[id].first = calibrationError;
}

SceneDrawer::SceneDrawer(const xn::UserGenerator &userGenerator, const xn::DepthGenerator &depthGenerator)
	:userGenerator(userGenerator), depthGenerator(depthGenerator)
{
	drawBackground = true;
	drawSkeleton = true;
	printID = true;
	printState = true;
	printFrameID = true;
	markJoints = true;
}

void SceneDrawer::drawDepthMap(const xn::DepthMetaData &dmd, const xn::SceneMetaData &smd)
{
	static unsigned char* pDepthTexBuf = new unsigned char[Width * Height*3];

	unsigned char* pDestImage = pDepthTexBuf;

	depthMapCreating(pDestImage, dmd, smd);

	imgBuf = Mat(Height, Width, CV_8UC3, pDepthTexBuf);
	XnUserID aUsers[15];
	XnUInt16 nUsers = 15;
	userGenerator.GetUsers(aUsers, nUsers);
	for (int i = 0; i < nUsers; ++i)
	{
		if (printID)
			printId(aUsers[i]);
		if (drawSkeleton && userGenerator.GetSkeletonCap().IsTracking(aUsers[i]))
		{
			if (markJoints)
				drawAllJoints(aUsers[i]);
			drawAllLimbs(aUsers[i]);
		}
	}
	imshow("qwe", imgBuf);
	waitKey( 20 );
}

bool SceneDrawer::drawLimb(XnUserID player, XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2)
{
	if (!userGenerator.GetSkeletonCap().IsTracking(player))
	{
		printf("not tracked!\n");
		return true;
	}

	if (!userGenerator.GetSkeletonCap().IsJointActive(eJoint1) ||
			!userGenerator.GetSkeletonCap().IsJointActive(eJoint2))
	{
		return false;
	}

	XnSkeletonJointPosition joint1, joint2;
	userGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint1, joint1);
	userGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint2, joint2);

	if (joint1.fConfidence < 0.5 || joint2.fConfidence < 0.5)
	{
		return true;
	}

	XnPoint3D pt[2];
	pt[0] = joint1.position;
	pt[1] = joint2.position;

	depthGenerator.ConvertRealWorldToProjective(2, pt, pt);
	line(imgBuf, Point(pt[0].X, pt[0].Y), Point(pt[1].X, pt[1].Y), Scalar(255, 255, 0));
	return true;
}

void SceneDrawer::drawCircle(float x, float y, float radius)
{
	circle(imgBuf, Point(x, y), radius, Scalar(255, 255, 0));
}

void SceneDrawer::drawJoint(XnUserID player, XnSkeletonJoint eJoint)
{
	if (!userGenerator.GetSkeletonCap().IsTracking(player))
	{
		printf("not tracked!\n");
		return;
	}

	if (!userGenerator.GetSkeletonCap().IsJointActive(eJoint))
	{
		return;
	}

	XnSkeletonJointPosition joint;
	userGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint, joint);

	if (joint.fConfidence < 0.5)
	{
		return;
	}

	XnPoint3D pt;
	pt = joint.position;

	depthGenerator.ConvertRealWorldToProjective(1, &pt, &pt);

	drawCircle(pt.X, pt.Y, 2);
}

void SceneDrawer::printId(XnUserID id)
{
	char strLabel[50] = "";
	XnPoint3D com;
	userGenerator.GetCoM(id, com);
	depthGenerator.ConvertRealWorldToProjective(1, &com, &com);

	XnUInt32 nDummy = 0;

	xnOSMemSet(strLabel, 0, sizeof(strLabel));
	if (!printState)
	{
		// Tracking
		xnOSStrFormat(strLabel, sizeof(strLabel), &nDummy, "%d", id);
	}
	else if (userGenerator.GetSkeletonCap().IsTracking(id))
	{
		// Tracking
		xnOSStrFormat(strLabel, sizeof(strLabel), &nDummy, "%d - Tracking", id);
	}
	else if (userGenerator.GetSkeletonCap().IsCalibrating(id))
	{
		// Calibrating
		xnOSStrFormat(strLabel, sizeof(strLabel), &nDummy, "%d - Calibrating [%s]", id, getCalibrationErrorString(errors[id].first));
	}

	putText(imgBuf, strLabel, Point(com.X, com.Y), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
}

void SceneDrawer::drawAllJoints(XnUserID id)
{
	drawJoint(id, XN_SKEL_HEAD);
	drawJoint(id, XN_SKEL_NECK);
	drawJoint(id, XN_SKEL_TORSO);
	drawJoint(id, XN_SKEL_WAIST);

	drawJoint(id, XN_SKEL_LEFT_COLLAR);
	drawJoint(id, XN_SKEL_LEFT_SHOULDER);
	drawJoint(id, XN_SKEL_LEFT_ELBOW);
	drawJoint(id, XN_SKEL_LEFT_WRIST);
	drawJoint(id, XN_SKEL_LEFT_HAND);
	drawJoint(id, XN_SKEL_LEFT_FINGERTIP);

	drawJoint(id, XN_SKEL_RIGHT_COLLAR);
	drawJoint(id, XN_SKEL_RIGHT_SHOULDER);
	drawJoint(id, XN_SKEL_RIGHT_ELBOW);
	drawJoint(id, XN_SKEL_RIGHT_WRIST);
	drawJoint(id, XN_SKEL_RIGHT_HAND);
	drawJoint(id, XN_SKEL_RIGHT_FINGERTIP);

	drawJoint(id, XN_SKEL_LEFT_HIP);
	drawJoint(id, XN_SKEL_LEFT_KNEE);
	drawJoint(id, XN_SKEL_LEFT_ANKLE);
	drawJoint(id, XN_SKEL_LEFT_FOOT);

	drawJoint(id, XN_SKEL_RIGHT_HIP);
	drawJoint(id, XN_SKEL_RIGHT_KNEE);
	drawJoint(id, XN_SKEL_RIGHT_ANKLE);
	drawJoint(id, XN_SKEL_RIGHT_FOOT);
}

void SceneDrawer::drawAllLimbs(XnUserID id)
{
	drawLimb(id, XN_SKEL_HEAD, XN_SKEL_NECK);

	drawLimb(id, XN_SKEL_NECK, XN_SKEL_LEFT_SHOULDER);
	drawLimb(id, XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_ELBOW);
	if (!drawLimb(id, XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_WRIST))
	{
		drawLimb(id, XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_HAND);
	}
	else
	{
		drawLimb(id, XN_SKEL_LEFT_WRIST, XN_SKEL_LEFT_HAND);
		drawLimb(id, XN_SKEL_LEFT_HAND, XN_SKEL_LEFT_FINGERTIP);
	}


	drawLimb(id, XN_SKEL_NECK, XN_SKEL_RIGHT_SHOULDER);
	drawLimb(id, XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW);
	if (!drawLimb(id, XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_WRIST))
	{
		drawLimb(id, XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_HAND);
	}
	else
	{
		drawLimb(id, XN_SKEL_RIGHT_WRIST, XN_SKEL_RIGHT_HAND);
		drawLimb(id, XN_SKEL_RIGHT_HAND, XN_SKEL_RIGHT_FINGERTIP);
	}

	drawLimb(id, XN_SKEL_LEFT_SHOULDER, XN_SKEL_TORSO);
	drawLimb(id, XN_SKEL_RIGHT_SHOULDER, XN_SKEL_TORSO);

	drawLimb(id, XN_SKEL_TORSO, XN_SKEL_LEFT_HIP);
	drawLimb(id, XN_SKEL_LEFT_HIP, XN_SKEL_LEFT_KNEE);
	drawLimb(id, XN_SKEL_LEFT_KNEE, XN_SKEL_LEFT_FOOT);

	drawLimb(id, XN_SKEL_TORSO, XN_SKEL_RIGHT_HIP);
	drawLimb(id, XN_SKEL_RIGHT_HIP, XN_SKEL_RIGHT_KNEE);
	drawLimb(id, XN_SKEL_RIGHT_KNEE, XN_SKEL_RIGHT_FOOT);

	drawLimb(id, XN_SKEL_LEFT_HIP, XN_SKEL_RIGHT_HIP);
}

void SceneDrawer::depthMapCreating(unsigned char *pDestImage, const xn::DepthMetaData &dmd, const xn::SceneMetaData &smd)
{
	unsigned int nNumberOfPoints = 0;
	XnUInt16 g_nXRes = dmd.XRes();
	XnUInt16 g_nYRes = dmd.YRes();
	const XnDepthPixel* pDepth = dmd.Data();
	const XnLabel* pLabels = smd.Data();
	unsigned int nValue = 0;
	static unsigned int nZRes = dmd.ZRes();
	static float* pDepthHist = (float*)malloc(nZRes* sizeof(float));

	// Calculate the accumulative histogram
	memset(pDepthHist, 0, nZRes*sizeof(float));
	for (int nY=0; nY<g_nYRes; nY++)
		for (int nX=0; nX<g_nXRes; nX++)
		{
			nValue = *pDepth;
			if (nValue != 0)
			{
				pDepthHist[nValue]++;
				nNumberOfPoints++;
			}
			pDepth++;
		}

	for (int i=1; i<nZRes; i++)
		pDepthHist[i] += pDepthHist[i-1];
	if (nNumberOfPoints)
		for (int i=1; i<nZRes; i++)
			pDepthHist[i] = (unsigned int)(256 * (1.0f - (pDepthHist[i] / nNumberOfPoints)));

	pDepth = dmd.Data();
	// Prepare the texture map
	for (int nY=0; nY<g_nYRes; nY++)
	{
		for (int nX=0; nX < g_nXRes; nX++)
		{
			pDestImage[0] = 0;
			pDestImage[1] = 0;
			pDestImage[2] = 0;
			if (drawBackground || *pLabels != 0)
			{
				nValue = *pDepth;
				XnLabel label = *pLabels;
				XnUInt32 nColorID = label % nColors;
				if (label == 0)
					nColorID = nColors;

				if (nValue != 0)
				{
					pDestImage[0] = pDepthHist[nValue] * Colors[nColorID][0];
					pDestImage[1] = pDepthHist[nValue] * Colors[nColorID][1];
					pDestImage[2] = pDepthHist[nValue] * Colors[nColorID][2];
				}
			}
			pDepth++;
			pLabels++;
			pDestImage+=3;
		}
		pDestImage += (Width - g_nXRes) *3;
	}
}

const XnChar *SceneDrawer::getCalibrationErrorString(XnCalibrationStatus error)
{
	switch (error)
	{
	case XN_CALIBRATION_STATUS_OK:
		return "OK";
	case XN_CALIBRATION_STATUS_NO_USER:
		return "NoUser";
	case XN_CALIBRATION_STATUS_ARM:
		return "Arm";
	case XN_CALIBRATION_STATUS_LEG:
		return "Leg";
	case XN_CALIBRATION_STATUS_HEAD:
		return "Head";
	case XN_CALIBRATION_STATUS_TORSO:
		return "Torso";
	case XN_CALIBRATION_STATUS_TOP_FOV:
		return "Top FOV";
	case XN_CALIBRATION_STATUS_SIDE_FOV:
		return "Side FOV";
	default:
		return "Unknown";
	}
}
