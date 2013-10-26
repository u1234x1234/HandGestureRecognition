#include "RotationGesture.h"
#include <cstdio>
#include <time.h>
using namespace std;
using namespace cv;

RotationGesture::RotationGesture()
{
	cnt = 0;
	dir = 0;
	volume = 50;
	char tm[100];
	sprintf(tm, "amixer set Master %d", volume);
//	popen(tm, "r");
}

void RotationGesture::addFrame(Mat &src)
{
	depthMap = src;
}

void RotationGesture::predict(int &result)
{
	vector<pair<int, int> > pos;
	findFingers(pos);
	//resize(depthMap, depthMap, Size(100, 100));
	//normalize(depthMap, depthMap, 0, 255, NORM_MINMAX, CV_8UC3);
	//imshow("12", depthMap);
	Mat asd = Mat::zeros(depthMap.size(), CV_8UC3);

	int centralizedX = 0;
	int centralizedY = 0;
	for (int i = 0; i < pos.size(); i++)
	{
		int x = pos[i].second;
		int y = pos[i].first;
		circle(asd, Point( x, y ), 1, Scalar(100,100,250));
		centralizedX += x;
		centralizedY += y;
		if ( hypot(x - asd.cols/2, y - asd.rows/2) < min(asd.cols/12, asd.rows/12) )
		{
			angles.clear();
			dir = 0;
			cnt = 0;
			return;
		}
	}
	centralizedX /= pos.size();
	centralizedY /= pos.size();
	double alpha = atan2(centralizedY - asd.rows/2, centralizedX - asd.cols/2);
	line(asd, Point( centralizedX, centralizedY ), Point(asd.cols/2, asd.rows/2), Scalar(250,50, 20, 200));
	angles.push_back(alpha);
	//printf("%d\n", -(int)(alpha * 180.0 / 3.14));
	resize(asd, asd, Size(200, 200), 2, 2, INTER_CUBIC);

	cnt++;
	if (cnt > 10){
		if (fix == 1000)
		{
			fix = -(int)(alpha * 180.0 / 3.14);
		//	printf("%d\n", fix);
			fflush(stdout);
		}
	}
	else
		fix = 1000;
	if (fix != 1000)
	{
		int g = -(int)(alpha * 180.0 / 3.14);
		if (abs(g - fix) < 5)return;
		if (g < fix)
		{
			volume += 3;
			char tm[100];
			sprintf(tm, "amixer set Master %d", volume);
			popen(tm, "r");
			printf("+\n");
		}
		else
		{
			volume -= 3;
			char tm[100];
			sprintf(tm, "amixer set Master %d", volume);
			popen(tm, "r");
			printf("-\n");
		}
		fix = g;
		imshow("1", asd);
		waitKey(10);
		return;
	}
	else
		return;
	////////////////////////////////////////////
//	Mat points( pos.size(), 1, CV_32FC2), labels;
//	int clusterCount = 5;
//	Mat centers( clusterCount, 1, points.type());
//	for (int i = 0; i < pos.size(); i++)
//		points.at<Point2f>(i) = Point2f(pos[i].second, pos[i].first);
//	kmeans(points, clusterCount, labels,
//		   TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0),
//		   3, KMEANS_PP_CENTERS, centers);
/*
	Mat centers2( 2, 1, points.type()), labels2;
	kmeans(centers, 2, labels2,
		   TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 10, 1.0),
		   3, KMEANS_PP_CENTERS, centers2);
	int cl[2];
	for( int  i = 0; i < pos.size(); i++ )
	{
		int clusterIdx = labels2.at<int>(i);
		cl[clusterIdx]++;
	}
	int h;
	if (cl[0] > cl[1])
		h = 0;
	else
		h = 1;
	circle( asd, Point(centers2.at<float>(h, 0), centers2.at<float>(h, 1)), 5, Scalar(0, 150, 0), 5);
	circle( asd, Point(centers2.at<float>(h^1, 0), centers2.at<float>(h^1, 1)), 5, Scalar(150, 150, 0), 5);
*/
	vector<Point> p(pos.size());
	for (int i=0;i<pos.size();i++)
		p[i] = Point(pos[i].second, pos[i].first);
	vector<Point> ind;
	convexHull(p, ind);
	for ( int i = 1; i < ind.size(); i++ )
		line(asd, ind[i], ind[i - 1], Scalar(200, 250, 10), 1);
//	for( int i = 0; i < pos.size(); i++ )
//	{
//		int clusterIdx = labels.at<int>(i);
//		Point ipt = points.at<Point2f>(i);
//		circle( asd, ipt, 2, colorTab[clusterIdx], CV_FILLED, CV_AA );
//	}

	////////////////////////////////////////////
	if (cnt < 5) return;
	if (angles.size() > 3)
	{
		if ( angles[angles.size() - 1] > angles[angles.size() - 2]
						&&  angles[angles.size() - 2] > angles[angles.size() - 3]
				//&& angles[angles.size() - 2] < angles[0]
				)
		{
			volume += 1;
			char tm[100];
			sprintf(tm, "amixer set Master %d", volume);
	//		popen(tm, "r");
			printf("+ %d\n", cnt);
			fflush(stdout);
			dir = 1;
			timeOfLastGesture = clock();
			angles.pop_front();
			return;
		}
	}
	if (angles.size() > 3)
	{
		if ( angles[angles.size() - 1] < angles[angles.size() - 2]
						&&  angles[angles.size() - 2] < angles[angles.size() - 3]
			 )
		{
			volume -= 1;
			char tm[100];
			sprintf(tm, "amixer set Master %d", volume);
		//	popen(tm, "r");
			printf("- %d\n", cnt);
			fflush(stdout);
			dir = 2;
			timeOfLastGesture = clock();
			return;
		}
		angles.pop_front();
	}
	dir = 0;
}

void RotationGesture::clear()
{
	angles.clear();
	d.clear();
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
