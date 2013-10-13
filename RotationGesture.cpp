#include "RotationGesture.h"
#include <cstdio>
using namespace std;
using namespace cv;

RotationGesture::RotationGesture()
{
	dir = 0;
	volume = 50;
	char tm[100];
	sprintf(tm, "amixer set Master %d", volume);
	popen(tm, "r");
}

void RotationGesture::addFrame(Mat &src)
{
	depthMap = src;
}

void RotationGesture::predict(int &result)
{
	vector<pair<int, int> > pos;
	findFingers(pos);
	Mat asd = Mat::zeros(depthMap.size(), CV_8UC3);
	int cnt = 0;
	int centralizedX = 0;
	int centralizedY = 0;
	for (int i = 0; i < pos.size(); i++)
	{
		int x = pos[i].second;
		int y = pos[i].first;
		circle(asd, Point( x, y ), 2, Scalar(100,100,250));
		if (y < asd.rows / 2)cnt++;
		centralizedX += x;
		centralizedY += y;
		if (hypot(x - asd.cols/2, y - asd.rows/2) < min(asd.cols/8, asd.rows/8) && dir)
		{
			angles.clear();
			dir = 0;
			timeOfLastGesture = clock();
		}
	}
	centralizedX /= pos.size();
	centralizedY /= pos.size();
	double alpha = atan2(centralizedY - asd.rows/2, centralizedX - asd.cols/2);
	line(asd, Point( centralizedX, centralizedY ), Point(asd.cols/2, asd.rows/2), Scalar(250,50, 20, 200));

	angles.push_back(alpha);
	if (angles.size() > 3)
	{
		if ((clock() - timeOfLastGesture)/CLOCKS_PER_SEC < 1 && dir != 2 && angles[angles.size() - 1] > angles[angles.size() - 2]
				//		&&  angles[angles.size() - 2] > angles[angles.size() - 3]
				)
		{
			volume += 1;
			char tm[100];
			sprintf(tm, "amixer set Master %d", volume);
			popen(tm, "r");
			dir = 1;
		}
	}
	if (angles.size() > 4)
	{
		if ((clock() - timeOfLastGesture)/CLOCKS_PER_SEC < 1 && dir != 1 && angles[angles.size() - 1] < angles[angles.size() - 2]
						&&  angles[angles.size() - 2] < angles[angles.size() - 3]
				)
		{
			volume -= 1;
			char tm[100];
			sprintf(tm, "amixer set Master %d", volume);
			popen(tm, "r");
			dir = 2;
		}
		angles.pop_front();
	}

	fflush(stdout);
	resize(asd, asd, Size(100, 100));
	imshow("1", asd);
	waitKey(10);
}

void RotationGesture::findFingers(std::vector<std::pair<int, int> > &fingerPositions)
{
	ushort minDist = USHRT_MAX;
	for (int i = 0; i < depthMap.rows; i++)
		for (int j = 0; j < depthMap.cols; j++)
		{
			ushort pixel = depthMap.at<ushort>(i, j);
			if (pixel)
				minDist = min(minDist, pixel);
		}
	for (int i = 0; i < depthMap.rows; i++)
		for (int j = 0; j < depthMap.cols; j++)
		{
			ushort pixel = depthMap.at<ushort>(i, j);
			if (pixel && pixel < minDist + fingersThreshhold)
				fingerPositions.push_back(make_pair(i, j));
		}
}
