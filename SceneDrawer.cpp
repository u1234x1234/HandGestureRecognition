/****************************************************************************
*                                                                           *
*  OpenNI 1.x Alpha                                                         *
*  Copyright (C) 2011 PrimeSense Ltd.                                       *
*                                                                           *
*  This file is part of OpenNI.                                             *
*                                                                           *
*  OpenNI is free software: you can redistribute it and/or modify           *
*  it under the terms of the GNU Lesser General Public License as published *
*  by the Free Software Foundation, either version 3 of the License, or     *
*  (at your option) any later version.                                      *
*                                                                           *
*  OpenNI is distributed in the hope that it will be useful,                *
*  but WITHOUT ANY WARRANTY; without even the implied warranty of           *
*  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the             *
*  GNU Lesser General Public License for more details.                      *
*                                                                           *
*  You should have received a copy of the GNU Lesser General Public License *
*  along with OpenNI. If not, see <http://www.gnu.org/licenses/>.           *
*                                                                           *
****************************************************************************/
//---------------------------------------------------------------------------
// Includes
//---------------------------------------------------------------------------
#include <math.h>
#include <queue>
#include <deque>
#include <set>
#include "SceneDrawer.h"
#include <opencv2/opencv.hpp>
#include <fstream>
#include <time.h>
#ifndef USE_GLES
#if (XN_PLATFORM == XN_PLATFORM_MACOSX)
	#include <GLUT/glut.h>
#else
	#include <GL/glut.h>
#endif
#else
	#include "opengles.h"
#endif
using namespace std;
using namespace cv;
extern ofstream out;
std::ostringstream fileName;
int nFiles, cnt;
extern CvSVM svmRight;
extern CvSVM svmLeft;
extern CvSVM svmTake;
extern CvSVM svmATake;
extern CvSVM svmiTake;
extern CvSVM svmiATake;

int rTake, lTake;
int ix, iy;
extern xn::UserGenerator g_UserGenerator;
extern xn::DepthGenerator g_DepthGenerator;

extern XnBool g_bDrawBackground;
extern XnBool g_bDrawPixels;
extern XnBool g_bDrawSkeleton;
extern XnBool g_bPrintID;
extern XnBool g_bPrintState;

extern XnBool g_bPrintFrameID;
extern XnBool g_bMarkJoints;
HOGDescriptor hog(
		 Size(64,64), //winSize
		 Size(16,16), //blocksize
		 Size(8,8), //blockStride,
		 Size(8,8), //cellSize,
		 9,  //nbins,
		 0,  //derivAper,
		 -1,  //winSigma,
		 0,  //histogramNormType,
		 0.2,  //L2HysThresh,
		 0  //gammal correction,
				 //nlevels=64
		 );
deque<vector<float> > de;
deque<vector<float> > de2;
#include <map>
std::map<XnUInt32, std::pair<XnCalibrationStatus, XnPoseDetectionStatus> > m_Errors;
void XN_CALLBACK_TYPE MyCalibrationInProgress(xn::SkeletonCapability& /*capability*/, XnUserID id, XnCalibrationStatus calibrationError, void* /*pCookie*/)
{
	m_Errors[id].first = calibrationError;
}
void XN_CALLBACK_TYPE MyPoseInProgress(xn::PoseDetectionCapability& /*capability*/, const XnChar* /*strPose*/, XnUserID id, XnPoseDetectionStatus poseError, void* /*pCookie*/)
{
	m_Errors[id].second = poseError;
}

unsigned int getClosestPowerOfTwo(unsigned int n)
{
	unsigned int m = 2;
	while(m < n) m<<=1;

	return m;
}
GLuint initTexture(void** buf, int& width, int& height)
{
	GLuint texID = 0;
	glGenTextures(1,&texID);

	width = getClosestPowerOfTwo(width);
	height = getClosestPowerOfTwo(height); 
	*buf = new unsigned char[width*height*4];
	glBindTexture(GL_TEXTURE_2D,texID);

	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

	return texID;
}

GLfloat texcoords[8];
void DrawRectangle(float topLeftX, float topLeftY, float bottomRightX, float bottomRightY)
{
	GLfloat verts[8] = {	topLeftX, topLeftY,
		topLeftX, bottomRightY,
		bottomRightX, bottomRightY,
		bottomRightX, topLeftY
	};
	glVertexPointer(2, GL_FLOAT, 0, verts);
	glDrawArrays(GL_TRIANGLE_FAN, 0, 4);

	//TODO: Maybe glFinish needed here instead - if there's some bad graphics crap
	glFlush();
}
void DrawTexture(float topLeftX, float topLeftY, float bottomRightX, float bottomRightY)
{
	glEnableClientState(GL_TEXTURE_COORD_ARRAY);
	glTexCoordPointer(2, GL_FLOAT, 0, texcoords);

	DrawRectangle(topLeftX, topLeftY, bottomRightX, bottomRightY);

	glDisableClientState(GL_TEXTURE_COORD_ARRAY);
}

XnFloat Colors[][3] =
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
XnUInt32 nColors = 10;
#ifndef USE_GLES
void glPrintString(void *font, char *str)
{
	int i,l = (int)strlen(str);

	for(i=0; i<l; i++)
	{
		glutBitmapCharacter(font,*str++);
	}
}
#endif
bool DrawLimb(XnUserID player, XnSkeletonJoint eJoint1, XnSkeletonJoint eJoint2)
{
	if (!g_UserGenerator.GetSkeletonCap().IsTracking(player))
	{
		printf("not tracked!\n");
		return true;
	}

	if (!g_UserGenerator.GetSkeletonCap().IsJointActive(eJoint1) ||
		!g_UserGenerator.GetSkeletonCap().IsJointActive(eJoint2))
	{
		return false;
	}

	XnSkeletonJointPosition joint1, joint2;
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint1, joint1);
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint2, joint2);

	if (joint1.fConfidence < 0.5 || joint2.fConfidence < 0.5)
	{
		return true;
	}

	XnPoint3D pt[2];
	pt[0] = joint1.position;
	pt[1] = joint2.position;

	g_DepthGenerator.ConvertRealWorldToProjective(2, pt, pt);
#ifndef USE_GLES
	glVertex3i(pt[0].X, pt[0].Y, 0);
	glVertex3i(pt[1].X, pt[1].Y, 0);
#else
	GLfloat verts[4] = {pt[0].X, pt[0].Y, pt[1].X, pt[1].Y};
	glVertexPointer(2, GL_FLOAT, 0, verts);
	glDrawArrays(GL_LINES, 0, 2);
	glFlush();
#endif

	return true;
}

static const float DEG2RAD = 3.14159/180;
 
void drawCircle(float x, float y, float radius)
{
   glBegin(GL_TRIANGLE_FAN);
 
   for (int i=0; i < 360; i++)
   {
      float degInRad = i*DEG2RAD;
      glVertex2f(x + cos(degInRad)*radius, y + sin(degInRad)*radius);
   }
 
   glEnd();
}
void DrawJoint(XnUserID player, XnSkeletonJoint eJoint)
{
	if (!g_UserGenerator.GetSkeletonCap().IsTracking(player))
	{
		printf("not tracked!\n");
		return;
	}

	if (!g_UserGenerator.GetSkeletonCap().IsJointActive(eJoint))
	{
		return;
	}

	XnSkeletonJointPosition joint;
	g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(player, eJoint, joint);

	if (joint.fConfidence < 0.5)
	{
		return;
	}

	XnPoint3D pt;
	pt = joint.position;

	g_DepthGenerator.ConvertRealWorldToProjective(1, &pt, &pt);

	drawCircle(pt.X, pt.Y, 2);
}

const XnChar* GetCalibrationErrorString(XnCalibrationStatus error)
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
	case XN_CALIBRATION_STATUS_POSE:
		return "Pose";
	default:
		return "Unknown";
	}
}
const XnChar* GetPoseErrorString(XnPoseDetectionStatus error)
{
	switch (error)
	{
	case XN_POSE_DETECTION_STATUS_OK:
		return "OK";
	case XN_POSE_DETECTION_STATUS_NO_USER:
		return "NoUser";
	case XN_POSE_DETECTION_STATUS_TOP_FOV:
		return "Top FOV";
	case XN_POSE_DETECTION_STATUS_SIDE_FOV:
		return "Side FOV";
	case XN_POSE_DETECTION_STATUS_ERROR:
		return "General error";
	default:
		return "Unknown";
	}
}
void handSeparation2(const Mat &depth, int y, int x, const Mat& image)
{
	ix = x;
	iy = y;

	queue<pair<int,int> > q;
	Mat flags = Mat::zeros(480, 640, CV_8UC1);
	q.push(make_pair(y, x));
	int dx[] = {1, -1, 0, 0}, dy[]={0, 0, 1, -1};
	int xx, yy;
	int gx1 = x - 10, gx2 = x + 10, gy1 = y - 10, gy2 = y + 10;
	if (gx1 < 0) gx1 = 0;
	if (gy1 < 0) gy1 = 0;
	if (gx2 >= 640) gx1 = 640 - 1;
	if (gy2 >= 480) gy1 = 480 - 1;
	const int inf  = (int)1e8;
	int tx = x, ty = y, mn = inf;
	for (int i = gy1; i <= gy2; i++)
		for (int j = gx1; j <= gx2; j++)
			if (depth.at<unsigned short>(i, j) < mn)
			{
				if (depth.at<unsigned short>(i, j) < 500)continue;
				ty = i;
				tx = j;
				mn = depth.at<unsigned short>(i, j);
			}
	if (mn < 500 || mn > 4000)return;
	x = tx;
	y = ty;
	unsigned short cnDist = depth.at<unsigned short>(y, x);
	unsigned short threshold = 100;
	flags.at<uchar>(y, x) = 1;
	int xmax = -1, xmin = inf, ymax = -1, ymin = inf;
	while (!q.empty())
	{
		pair<int,int> t = q.front();
		q.pop();
		xmax = max(xmax, t.second);
		xmin = min(xmin, t.second);

		ymax = max(ymax, t.first);
		ymin = min(ymin, t.first);
		for (int i = 0; i < 4; i++)
		{
			xx = t.second + dx[i];
			yy = t.first + dy[i];
			if (yy < 0 || xx < 0 || xx >= 640 || yy >= 480)continue;
			if (flags.at<uchar>(yy, xx))continue;
			if (abs(depth.at<unsigned short>(yy, xx) - cnDist) > threshold) continue;
			q.push(make_pair(yy, xx));
			flags.at<uchar>(yy, xx) = 1;
		}
	}
	if (xmin == inf || ymin == inf || xmax == -1 || ymax == -1){return;}
	if ( xmax <= xmin || ymax <= ymin ) {return;}
	Mat resImage;
	depth.copyTo(resImage, flags);
//	Mat resImage2 = resImage(Range(ymin, ymax), Range(xmin, xmax));

	//resImage.convertTo(resImage, CV_8UC1, 3);
	//cv::resize(resImage, resImage, Size(100, 100), 0, 0, INTER_CUBIC);
	//cv::resize(resImage, resImage, Size(), 1.5, 1.5, INTER_CUBIC);
//	blur(resImage, resImage, Size(3,3));
	resImage = resImage(Range(ymin, ymax), Range(xmin, xmax));
	resImage *= 100;
	resize(resImage, resImage, Size(64, 64));
//	imshow("4341", resImage);

	fileName.str("");
	fileName << ++nFiles;
	//cv::imwrite("samples/"+fileName.str()+".png", resImage );

	imwrite("g.png", resImage);
	Mat im = imread("g.png", CV_LOAD_IMAGE_GRAYSCALE);
	vector<float> tmp;
	hog.compute(im, tmp, Size(8,8), Size(0,0));
	if (de2.size() >= 10)de2.pop_back();
	de2.push_front(tmp);
	if (de2.size() >= 10)
	{
		vector<float> tests;
		for (int i = 0; i < 10; i++)
			tests.insert(tests.end(), de2[9 - i].begin(), de2[9 - i].end());

		Mat ts(1, tests.size(), CV_32FC1);
		for (int i = 0; i < tests.size(); i++)
			ts.at<float>(0,i)=tests[i];

		if (svmiTake.predict(ts) == 1)
		{
			printf("+++++++++++++++++++ L  take   +++++++++++++++++++++++++++++++++++++\n");
			lTake = 1;
			de2.clear();
		}
		if (svmiATake.predict(ts) == 1)
		{
			printf("////////////////// L  Atake   /////////////////////////////////////\n");
			lTake = 0;
			rTake = 0;
			de2.clear();
		}
		//printf("no gesture\n");
	}
	waitKey(10);
}
vector<int> rr;
clock_t init, final;
double diffclock( clock_t clock1, clock_t clock2 ) {

		double diffticks = clock1 - clock2;
		double diffms    = diffticks / ( CLOCKS_PER_SEC / 1000 );

		return diffms;
	}
void handSeparation(const Mat &depth, int y, int x, const Mat& image)
{
	Mat g;
	image.copyTo(g);
	circle(g, Point(x, y), 10, Scalar(100, 0, 200), 2);
	circle(g, Point(ix, iy), 10, Scalar(100, 150, 200), 2);
	if (lTake && rTake) rectangle(g, Rect(Point(x, y), Point(ix, iy)), Scalar(0, 0, 255), 3);
	final = clock();
	//cout << diffclock(init, final);
	if (fabs(diffclock(init, final)) <= 300)
	for (int i = 0 ; i < rr.size(); i++)
	{
		ostringstream ot;
		ot << rr[i] << ".jpg";
		Mat ff = imread("../HandGestureRecognition/images/" + ot.str());
		resize(ff, ff, Size(50, 100));
		ff.copyTo(g(Rect(i*50 + 200, 0, ff.cols, ff.rows)));
	}
	else rr.clear();
	imshow("qw", g);
	queue<pair<int,int> > q;
	Mat flags = Mat::zeros(480, 640, CV_8UC1);
	q.push(make_pair(y, x));
	int dx[] = {1, -1, 0, 0}, dy[]={0, 0, 1, -1};
	int xx, yy;
	int gx1 = x - 10, gx2 = x + 10, gy1 = y - 10, gy2 = y + 10;
	if (gx1 < 0) gx1 = 0;
	if (gy1 < 0) gy1 = 0;
	if (gx2 >= 640) gx1 = 640 - 1;
	if (gy2 >= 480) gy1 = 480 - 1;
	const int inf  = (int)1e8;
	int tx = x, ty = y, mn = inf;
	for (int i = gy1; i <= gy2; i++)
		for (int j = gx1; j <= gx2; j++)
			if (depth.at<unsigned short>(i, j) < mn)
			{
				if (depth.at<unsigned short>(i, j) < 500)continue;
				ty = i;
				tx = j;
				mn = depth.at<unsigned short>(i, j);
			}
	if (mn < 500 || mn > 4000)return;
	x = tx;
	y = ty;
	unsigned short cnDist = depth.at<unsigned short>(y, x);
	unsigned short threshold = 100;
	flags.at<uchar>(y, x) = 1;
	int xmax = -1, xmin = inf, ymax = -1, ymin = inf;
	while (!q.empty())
	{
		pair<int,int> t = q.front();
		q.pop();
		xmax = max(xmax, t.second);
		xmin = min(xmin, t.second);

		ymax = max(ymax, t.first);
		ymin = min(ymin, t.first);
		for (int i = 0; i < 4; i++)
		{
			xx = t.second + dx[i];
			yy = t.first + dy[i];
			if (yy < 0 || xx < 0 || xx >= 640 || yy >= 480)continue;
			if (flags.at<uchar>(yy, xx))continue;
			if (abs(depth.at<unsigned short>(yy, xx) - cnDist) > threshold) continue;
			q.push(make_pair(yy, xx));
			flags.at<uchar>(yy, xx) = 1;
		}
	}
	if (xmin == inf || ymin == inf || xmax == -1 || ymax == -1){return;}
	if ( xmax <= xmin || ymax <= ymin ) {return;}
	Mat resImage;
	depth.copyTo(resImage, flags);
//	Mat resImage2 = resImage(Range(ymin, ymax), Range(xmin, xmax));

	//resImage.convertTo(resImage, CV_8UC1, 3);
	//cv::resize(resImage, resImage, Size(100, 100), 0, 0, INTER_CUBIC);
	//cv::resize(resImage, resImage, Size(), 1.5, 1.5, INTER_CUBIC);
//	blur(resImage, resImage, Size(3,3));
	resImage = resImage(Range(ymin, ymax), Range(xmin, xmax));
	resImage *= 100;
	resize(resImage, resImage, Size(64, 64));
	//imshow("434", resImage);

	fileName.str("");
	fileName << ++nFiles;
//	cv::imwrite("samples/"+fileName.str()+".png", resImage );

	imwrite("g.png", resImage);
	Mat im = imread("g.png", CV_LOAD_IMAGE_GRAYSCALE);
	vector<float> tmp;
	hog.compute(im, tmp, Size(8,8), Size(0,0));
	if (de.size() >= 10)de.pop_back();
	de.push_front(tmp);
	if (de.size() >= 10)
	{
		vector<float> tests;
		for (int i = 0; i < 10; i++)
			tests.insert(tests.end(), de[9 - i].begin(), de[9 - i].end());

		Mat ts(1, tests.size(), CV_32FC1);
		for (int i = 0; i < tests.size(); i++)
			ts.at<float>(0,i)=tests[i];

		float responseR = svmRight.predict(ts);
		if (responseR == 1)
		{
			printf("----------------- right ---------------\n");
			Mat z = imread("../HandGestureRecognition/images/1.jpg");
			imshow("res", z);
			//waitKey(500);
			rTake = 0;
			rr.clear();
			rr.push_back(1);
			init=clock();
			de.clear();
		}
		if (svmLeft.predict(ts) == 1)
		{
			printf("----------------- left ---------------\n");
			Mat z = imread("../HandGestureRecognition/images/2.jpg");
			imshow("res", z);
			//waitKey(500);
			rTake = 0;
			de.clear();
			rr.clear();
			rr.push_back(2);
			init=clock();
		}
		if (svmTake.predict(ts) == 1)
		{
			printf("+++++++++++++++++++   take   ++++++++++++++++\n");
			Mat z = imread("../HandGestureRecognition/images/3.jpg");
			imshow("res", z);
			//waitKey(500);
			rTake = 1;
			de.clear();
			rr.clear();
			rr.push_back(3);
			init=clock();
		}
		if (svmATake.predict(ts) == 1)
		{
			printf("//////////////////   Atake   /////////////////\n");
			Mat z = imread("../HandGestureRecognition/images/4.jpg");
			imshow("res", z);
			//waitKey(500);
			rTake = 0;
			lTake = 0;
			rr.clear();
			rr.push_back(4);
			de.clear();
			init=clock();
		}
		destroyWindow("res");
		//printf("no gesture\n");
	}
	waitKey(10);
}
void DrawDepthMap(const xn::DepthMetaData& dmd, const xn::SceneMetaData& smd, const xn::ImageMetaData& imd)
{
	static bool bInitialized = false;	
	static GLuint depthTexID;
	static unsigned char* pDepthTexBuf;
	static int texWidth, texHeight;

	float topLeftX;
	float topLeftY;
	float bottomRightY;
	float bottomRightX;
	float texXpos;
	float texYpos;

	if(!bInitialized)
	{
		texWidth =  getClosestPowerOfTwo(dmd.XRes());
		texHeight = getClosestPowerOfTwo(dmd.YRes());

//		printf("Initializing depth texture: width = %d, height = %d\n", texWidth, texHeight);
		depthTexID = initTexture((void**)&pDepthTexBuf,texWidth, texHeight) ;

//		printf("Initialized depth texture: width = %d, height = %d\n", texWidth, texHeight);
		bInitialized = true;

		topLeftX = dmd.XRes();
		topLeftY = 0;
		bottomRightY = dmd.YRes();
		bottomRightX = 0;
		texXpos =(float)dmd.XRes()/texWidth;
		texYpos  =(float)dmd.YRes()/texHeight;

		memset(texcoords, 0, 8*sizeof(float));
		texcoords[0] = texXpos, texcoords[1] = texYpos, texcoords[2] = texXpos, texcoords[7] = texYpos;
	}

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

	glBindTexture(GL_TEXTURE_2D, depthTexID);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, texWidth, texHeight, 0, GL_RGB, GL_UNSIGNED_BYTE, pDepthTexBuf);

	// Display the OpenGL texture map
	glColor4f(0.75,0.75,0.75,1);

	glEnable(GL_TEXTURE_2D);
	DrawTexture(dmd.XRes(),dmd.YRes(),0,0);	
	glDisable(GL_TEXTURE_2D);

	char strLabel[50] = "";
	XnUserID aUsers[15];
	XnUInt16 nUsers = 15;
	g_UserGenerator.GetUsers(aUsers, nUsers);
	for (int i = 0; i < nUsers; ++i)
	{
#ifndef USE_GLES
		if (g_bPrintID)
		{
			XnPoint3D com;
			g_UserGenerator.GetCoM(aUsers[i], com);
			g_DepthGenerator.ConvertRealWorldToProjective(1, &com, &com);

			XnUInt32 nDummy = 0;

			xnOSMemSet(strLabel, 0, sizeof(strLabel));
			if (!g_bPrintState)
			{
				// Tracking
				xnOSStrFormat(strLabel, sizeof(strLabel), &nDummy, "%d", aUsers[i]);
			}
			else if (g_UserGenerator.GetSkeletonCap().IsTracking(aUsers[i]))
			{
				// Tracking
				xnOSStrFormat(strLabel, sizeof(strLabel), &nDummy, "%d - Tracking", aUsers[i]);
			}
			else if (g_UserGenerator.GetSkeletonCap().IsCalibrating(aUsers[i]))
			{
				// Calibrating
				xnOSStrFormat(strLabel, sizeof(strLabel), &nDummy, "%d - Calibrating [%s]", aUsers[i], GetCalibrationErrorString(m_Errors[aUsers[i]].first));
			}
			else
			{
				// Nothing
				xnOSStrFormat(strLabel, sizeof(strLabel), &nDummy, "%d - Looking for pose [%s]", aUsers[i], GetPoseErrorString(m_Errors[aUsers[i]].second));
			}


			glColor4f(1-Colors[i%nColors][0], 1-Colors[i%nColors][1], 1-Colors[i%nColors][2], 1);

			glRasterPos2i(com.X, com.Y);
			glPrintString(GLUT_BITMAP_HELVETICA_18, strLabel);
		}
#endif
		if (g_bDrawSkeleton && g_UserGenerator.GetSkeletonCap().IsTracking(aUsers[i]))
		{
			glColor4f(1-Colors[aUsers[i]%nColors][0], 1-Colors[aUsers[i]%nColors][1], 1-Colors[aUsers[i]%nColors][2], 1);
			// getting information from depthmap

			if (g_UserGenerator.GetSkeletonCap().IsTracking(aUsers[i]) &&  g_UserGenerator.GetSkeletonCap().IsJointActive(XN_SKEL_LEFT_HAND))
			{

				XnSkeletonJointPosition joint;
				g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_LEFT_HAND, joint);
				if (joint.fConfidence >= 0.5)
				{

					XnPoint3D pt;
					pt = joint.position;
					g_DepthGenerator.ConvertRealWorldToProjective(1, &pt, &pt);

					Mat DepthBuf( dmd.YRes(), dmd.XRes(), CV_16UC1 );
					const XnDepthPixel* pDepthMap = dmd.Data();
					memcpy( DepthBuf.data, pDepthMap, dmd.YRes()*dmd.XRes()*sizeof(XnDepthPixel) );

					const XnUInt8* g_Img =imd.Data();
					Mat ImgBuf(480,640,CV_8UC3,(unsigned short*)g_Img);
					Mat ImgBuf2;
					cvtColor(ImgBuf,ImgBuf2,CV_RGB2BGR);
					//imshow("456", ImgBuf2);

					handSeparation2(DepthBuf, pt.Y, pt.X, ImgBuf2);


					//HOGDescriptor::

				}
			}

			if (g_UserGenerator.GetSkeletonCap().IsTracking(aUsers[i]) &&  g_UserGenerator.GetSkeletonCap().IsJointActive(XN_SKEL_RIGHT_HAND))
			{

				XnSkeletonJointPosition joint;
				g_UserGenerator.GetSkeletonCap().GetSkeletonJointPosition(aUsers[i], XN_SKEL_RIGHT_HAND, joint);
				if (joint.fConfidence >= 0.5)
				{
					/*
					cv::Mat asd = cv::Mat::zeros(100, 100, CV_32F);
					cv::putText(asd, "0" + cnt++, cv::Point(10,10), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, cv::Scalar(255, 0, 0));
					if (cnt == 9)cnt = 0;
					imshow("12", asd);
					cv::waitKey(100);
					*/
					XnPoint3D pt;
					pt = joint.position;
					g_DepthGenerator.ConvertRealWorldToProjective(1, &pt, &pt);

					Mat DepthBuf( dmd.YRes(), dmd.XRes(), CV_16UC1 );
					const XnDepthPixel* pDepthMap = dmd.Data();
					memcpy( DepthBuf.data, pDepthMap, dmd.YRes()*dmd.XRes()*sizeof(XnDepthPixel) );

					/*unsigned short aa = DepthBuf.at<unsigned short>(pt.Y, pt.X);
					ostringstream ttt;
					ttt << aa << ' ';
					ttt << pt.X << ' ' << pt.Y;
					Mat asd = Mat::zeros(300, 900, 1);
					putText(asd, ttt.str(), Point(70,70), FONT_HERSHEY_SCRIPT_SIMPLEX, 2.0, Scalar(255, 100), 3);
					imshow("124", asd);
					waitKey(10);*/
					const XnUInt8* g_Img =imd.Data();
					Mat ImgBuf(480,640,CV_8UC3,(unsigned short*)g_Img);
					Mat ImgBuf2;
					cvtColor(ImgBuf,ImgBuf2,CV_RGB2BGR);
					//imshow("456", ImgBuf2);

					handSeparation(DepthBuf, pt.Y, pt.X, ImgBuf2);


					//HOGDescriptor::

				}
			}

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

#ifndef USE_GLES
			glBegin(GL_LINES);
#endif

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
#ifndef USE_GLES
			glEnd();
#endif
		}
	}

	if (g_bPrintFrameID)
	{
		static XnChar strFrameID[80];
		xnOSMemSet(strFrameID, 0, 80);
		XnUInt32 nDummy = 0;
		xnOSStrFormat(strFrameID, sizeof(strFrameID), &nDummy, "%d", dmd.FrameID());

		glColor4f(1, 0, 0, 1);

		glRasterPos2i(10, 10);

		glPrintString(GLUT_BITMAP_HELVETICA_18, strFrameID);
	}
}
