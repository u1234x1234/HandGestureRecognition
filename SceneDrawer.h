#ifndef SCENEDRAWER_H_
#define SCENEDRAWER_H_

#include <XnCppWrapper.h>
#include <opencv2/opencv.hpp>

class SceneDrawer{
public:
	SceneDrawer(const xn::UserGenerator&, const xn::DepthGenerator&);
	void DrawDepthMap(const xn::DepthMetaData& dmd, const xn::SceneMetaData& smd);
private:
	bool DrawLimb(XnUserID player, XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2);
	void drawCircle(float x, float y, float radius);
	void DrawJoint(XnUserID player, XnSkeletonJoint eJoint);
	const XnChar* GetCalibrationErrorString(XnCalibrationStatus error);
	const xn::UserGenerator userGenerator;
	const xn::DepthGenerator depthGenerator;
	XnBool g_bDrawBackground;
	XnBool g_bDrawPixels;
	XnBool g_bDrawSkeleton;
	XnBool g_bPrintID;
	XnBool g_bPrintState;
	XnBool g_bPrintFrameID;
	XnBool g_bMarkJoints;
	cv::Mat ImgBuf;
	const XnFloat Colors[11][3] =
	{
		{0,1,1},
		{0,0,1},
		{0,1,0},
		{1,1,0},
		{1,0,0},
		{1,.5,0},
		{.5,1,0},
		{0,.5,1},
		{.5,0,1},
		{1,1,.5},
		{1,1,1}
	};
	const XnUInt32 nColors = 10;
};

#endif
