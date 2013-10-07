#ifndef SCENEDRAWER_H_
#define SCENEDRAWER_H_

#include <XnCppWrapper.h>
#include <opencv2/opencv.hpp>

class SceneDrawer{
public:
	SceneDrawer(const xn::UserGenerator&, const xn::DepthGenerator&);
	void drawDepthMap(const xn::DepthMetaData& dmd, const xn::SceneMetaData& smd);
private:
	bool drawLimb(XnUserID player, XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2);
	void drawCircle(float x, float y, float radius);
	void drawJoint(XnUserID player, XnSkeletonJoint eJoint);
	void printId(XnUserID);
	void drawAllJoints(XnUserID);
	void drawAllLimbs(XnUserID);
	void depthMapCreating(unsigned char*, const xn::DepthMetaData&, const xn::SceneMetaData&);
	const XnChar* getCalibrationErrorString(XnCalibrationStatus error);
	const xn::UserGenerator userGenerator;
	const xn::DepthGenerator depthGenerator;
	XnBool drawBackground;
	XnBool drawSkeleton;
	XnBool printID;
	XnBool printState;
	XnBool printFrameID;
	XnBool markJoints;
	cv::Mat imgBuf;
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
	static const int Width = 640, Height = 480;
};

#endif
