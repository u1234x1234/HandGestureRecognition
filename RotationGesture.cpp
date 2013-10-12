#include "RotationGesture.h"
#include <cstdio>
using namespace std;
using namespace cv;

RotationGesture::RotationGesture(cv::Mat &src)
	:depthMap(src)
{
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
			if (pixel && pixel < minDist + 30)
				fingerPositions.push_back(make_pair(i, j));
		}
	//for (int i = 0; i < fingerPositions.size(); i++)
	//	circle(depthMap, Point(fingerPositions[i].first, fingerPositions[i].second), 2, Scalar(200,200,200));
}
