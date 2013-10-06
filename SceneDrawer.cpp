#include "SceneDrawer.h"
#include <map>

using namespace cv;

std::map<XnUInt32, std::pair<XnCalibrationStatus, XnPoseDetectionStatus> > m_Errors;
void XN_CALLBACK_TYPE MyCalibrationInProgress(xn::SkeletonCapability& /*capability*/, XnUserID id, XnCalibrationStatus calibrationError, void* /*pCookie*/)
{
	m_Errors[id].first = calibrationError;
}

SceneDrawer::SceneDrawer(const xn::UserGenerator &userGenerator, const xn::DepthGenerator &depthGenerator)
	:userGenerator(userGenerator), depthGenerator(depthGenerator)
{
	g_bDrawBackground = true;
	g_bDrawPixels = true;
	g_bDrawSkeleton = true;
	g_bPrintID = true;
	g_bPrintState = true;
	g_bPrintFrameID = true;
	g_bMarkJoints = true;
}

void SceneDrawer::DrawDepthMap(const xn::DepthMetaData &dmd, const xn::SceneMetaData &smd)
{
	static int texWidth = 640, texHeight = 480;
	static unsigned char* pDepthTexBuf = new unsigned char[texWidth*texHeight*3];

	unsigned int nValue = 0;
	unsigned int nHistValue = 0;
	unsigned int nIndex = 0;
	unsigned int nX = 0;
	unsigned int nY = 0;
	unsigned int nNumberOfPoints = 0;
	XnUInt16 g_nXRes = dmd.XRes();
	XnUInt16 g_nYRes = dmd.YRes();

	unsigned char* pDestImage = pDepthTexBuf;

	const XnDepthPixel* pDepth = dmd.Data();
	const XnLabel* pLabels = smd.Data();

	static unsigned int nZRes = dmd.ZRes();
	static float* pDepthHist = (float*)malloc(nZRes* sizeof(float));

	// Calculate the accumulative histogram
	memset(pDepthHist, 0, nZRes*sizeof(float));
	for (nY=0; nY<g_nYRes; nY++)
	{
		for (nX=0; nX<g_nXRes; nX++)
		{
			nValue = *pDepth;

			if (nValue != 0)
			{
				pDepthHist[nValue]++;
				nNumberOfPoints++;
			}

			pDepth++;
		}
	}

	for (nIndex=1; nIndex<nZRes; nIndex++)
	{
		pDepthHist[nIndex] += pDepthHist[nIndex-1];
	}
	if (nNumberOfPoints)
	{
		for (nIndex=1; nIndex<nZRes; nIndex++)
		{
			pDepthHist[nIndex] = (unsigned int)(256 * (1.0f - (pDepthHist[nIndex] / nNumberOfPoints)));
		}
	}

	pDepth = dmd.Data();
	if (g_bDrawPixels)
	{
		XnUInt32 nIndex = 0;
		// Prepare the texture map
		for (nY=0; nY<g_nYRes; nY++)
		{
			for (nX=0; nX < g_nXRes; nX++, nIndex++)
			{

				pDestImage[0] = 0;
				pDestImage[1] = 0;
				pDestImage[2] = 0;
				if (g_bDrawBackground || *pLabels != 0)
				{
					nValue = *pDepth;
					XnLabel label = *pLabels;
					XnUInt32 nColorID = label % nColors;
					if (label == 0)
					{
						nColorID = nColors;
					}

					if (nValue != 0)
					{
						nHistValue = pDepthHist[nValue];

						pDestImage[0] = nHistValue * Colors[nColorID][0];
						pDestImage[1] = nHistValue * Colors[nColorID][1];
						pDestImage[2] = nHistValue * Colors[nColorID][2];
					}
				}

				pDepth++;
				pLabels++;
				pDestImage+=3;
			}

			pDestImage += (texWidth - g_nXRes) *3;
		}
	}
	else
	{
		xnOSMemSet(pDepthTexBuf, 0, 3*2*g_nXRes*g_nYRes);
	}

	ImgBuf = Mat(texHeight, texWidth, CV_8UC3, pDepthTexBuf);
	char strLabel[50] = "";
	XnUserID aUsers[15];
	XnUInt16 nUsers = 15;
	userGenerator.GetUsers(aUsers, nUsers);
	for (int i = 0; i < nUsers; ++i)
	{
		if (g_bPrintID)
		{
			XnPoint3D com;
			userGenerator.GetCoM(aUsers[i], com);
			depthGenerator.ConvertRealWorldToProjective(1, &com, &com);

			XnUInt32 nDummy = 0;

			xnOSMemSet(strLabel, 0, sizeof(strLabel));
			if (!g_bPrintState)
			{
				// Tracking
				xnOSStrFormat(strLabel, sizeof(strLabel), &nDummy, "%d", aUsers[i]);
			}
			else if (userGenerator.GetSkeletonCap().IsTracking(aUsers[i]))
			{
				// Tracking
				xnOSStrFormat(strLabel, sizeof(strLabel), &nDummy, "%d - Tracking", aUsers[i]);
			}
			else if (userGenerator.GetSkeletonCap().IsCalibrating(aUsers[i]))
			{
				// Calibrating
				xnOSStrFormat(strLabel, sizeof(strLabel), &nDummy, "%d - Calibrating [%s]", aUsers[i], GetCalibrationErrorString(m_Errors[aUsers[i]].first));
			}

			putText(ImgBuf, strLabel, Point(com.X, com.Y), FONT_HERSHEY_COMPLEX_SMALL, 0.8, cvScalar(200,200,250), 1, CV_AA);
		}
		if (g_bDrawSkeleton && userGenerator.GetSkeletonCap().IsTracking(aUsers[i]))
		{
			// Draw Joints
			if (g_bMarkJoints)
			{
				// Try to draw all joints
				DrawJoint(aUsers[i], XN_SKEL_HEAD);
				DrawJoint(aUsers[i], XN_SKEL_NECK);
				DrawJoint(aUsers[i], XN_SKEL_TORSO);
				DrawJoint(aUsers[i], XN_SKEL_WAIST);

				DrawJoint(aUsers[i], XN_SKEL_LEFT_COLLAR);
				DrawJoint(aUsers[i], XN_SKEL_LEFT_SHOULDER);
				DrawJoint(aUsers[i], XN_SKEL_LEFT_ELBOW);
				DrawJoint(aUsers[i], XN_SKEL_LEFT_WRIST);
				DrawJoint(aUsers[i], XN_SKEL_LEFT_HAND);
				DrawJoint(aUsers[i], XN_SKEL_LEFT_FINGERTIP);

				DrawJoint(aUsers[i], XN_SKEL_RIGHT_COLLAR);
				DrawJoint(aUsers[i], XN_SKEL_RIGHT_SHOULDER);
				DrawJoint(aUsers[i], XN_SKEL_RIGHT_ELBOW);
				DrawJoint(aUsers[i], XN_SKEL_RIGHT_WRIST);
				DrawJoint(aUsers[i], XN_SKEL_RIGHT_HAND);
				DrawJoint(aUsers[i], XN_SKEL_RIGHT_FINGERTIP);

				DrawJoint(aUsers[i], XN_SKEL_LEFT_HIP);
				DrawJoint(aUsers[i], XN_SKEL_LEFT_KNEE);
				DrawJoint(aUsers[i], XN_SKEL_LEFT_ANKLE);
				DrawJoint(aUsers[i], XN_SKEL_LEFT_FOOT);

				DrawJoint(aUsers[i], XN_SKEL_RIGHT_HIP);
				DrawJoint(aUsers[i], XN_SKEL_RIGHT_KNEE);
				DrawJoint(aUsers[i], XN_SKEL_RIGHT_ANKLE);
				DrawJoint(aUsers[i], XN_SKEL_RIGHT_FOOT);
			}

			// Draw Limbs
			DrawLimb(aUsers[i], XN_SKEL_HEAD, XN_SKEL_NECK);

			DrawLimb(aUsers[i], XN_SKEL_NECK, XN_SKEL_LEFT_SHOULDER);
			DrawLimb(aUsers[i], XN_SKEL_LEFT_SHOULDER, XN_SKEL_LEFT_ELBOW);
			if (!DrawLimb(aUsers[i], XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_WRIST))
			{
				DrawLimb(aUsers[i], XN_SKEL_LEFT_ELBOW, XN_SKEL_LEFT_HAND);
			}
			else
			{
				DrawLimb(aUsers[i], XN_SKEL_LEFT_WRIST, XN_SKEL_LEFT_HAND);
				DrawLimb(aUsers[i], XN_SKEL_LEFT_HAND, XN_SKEL_LEFT_FINGERTIP);
			}


			DrawLimb(aUsers[i], XN_SKEL_NECK, XN_SKEL_RIGHT_SHOULDER);
			DrawLimb(aUsers[i], XN_SKEL_RIGHT_SHOULDER, XN_SKEL_RIGHT_ELBOW);
			if (!DrawLimb(aUsers[i], XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_WRIST))
			{
				DrawLimb(aUsers[i], XN_SKEL_RIGHT_ELBOW, XN_SKEL_RIGHT_HAND);
			}
			else
			{
				DrawLimb(aUsers[i], XN_SKEL_RIGHT_WRIST, XN_SKEL_RIGHT_HAND);
				DrawLimb(aUsers[i], XN_SKEL_RIGHT_HAND, XN_SKEL_RIGHT_FINGERTIP);
			}

			DrawLimb(aUsers[i], XN_SKEL_LEFT_SHOULDER, XN_SKEL_TORSO);
			DrawLimb(aUsers[i], XN_SKEL_RIGHT_SHOULDER, XN_SKEL_TORSO);

			DrawLimb(aUsers[i], XN_SKEL_TORSO, XN_SKEL_LEFT_HIP);
			DrawLimb(aUsers[i], XN_SKEL_LEFT_HIP, XN_SKEL_LEFT_KNEE);
			DrawLimb(aUsers[i], XN_SKEL_LEFT_KNEE, XN_SKEL_LEFT_FOOT);

			DrawLimb(aUsers[i], XN_SKEL_TORSO, XN_SKEL_RIGHT_HIP);
			DrawLimb(aUsers[i], XN_SKEL_RIGHT_HIP, XN_SKEL_RIGHT_KNEE);
			DrawLimb(aUsers[i], XN_SKEL_RIGHT_KNEE, XN_SKEL_RIGHT_FOOT);

			DrawLimb(aUsers[i], XN_SKEL_LEFT_HIP, XN_SKEL_RIGHT_HIP);
		}
	}
	imshow("qwe", ImgBuf);
	waitKey( 20 );
}

bool SceneDrawer::DrawLimb(XnUserID player, XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2)
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
	line(ImgBuf, Point(pt[0].X, pt[0].Y), Point(pt[1].X, pt[1].Y), Scalar(255, 255, 0));
	return true;
}

void SceneDrawer::drawCircle(float x, float y, float radius)
{
	circle(ImgBuf, Point(x, y), radius, Scalar(255, 255, 0));
}

void SceneDrawer::DrawJoint(XnUserID player, XnSkeletonJoint eJoint)
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

const XnChar *SceneDrawer::GetCalibrationErrorString(XnCalibrationStatus error)
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
