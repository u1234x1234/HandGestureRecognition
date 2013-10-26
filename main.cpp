#include <opencv2/opencv.hpp>
#include <OpenNIContext.h>
#include "HandSeparator.h"
#include "RotationGesture.h"
#include "SceneDrawer.h"

using namespace std;
using namespace cv;
CvSVM svmRight;
CvSVM svmLeft;
CvSVM svmTake;
CvSVM svmATake;
CvSVM svmiTake;
CvSVM svmiATake;

int main(int argc, char **argv)
{
	svmRight.load("../HandGestureRecognition/config/right.xml");
	svmLeft.load("../HandGestureRecognition/config/left.xml");
	svmTake.load("../HandGestureRecognition/config/take.xml");
	svmATake.load("../HandGestureRecognition/config/atake.xml");
	svmiTake.load("../HandGestureRecognition/config/itake.xml");
	svmiATake.load("../HandGestureRecognition/config/iatake.xml");

	OpenNIContext *context = new OpenNIContext(argc, argv);
	RotationGesture rg;
	while (1)
	{
		context->update();
	//	context->display();
		Mat a;
		context->getImageMap(a);
		Mat b;
		context->getDepthMap(b);
		vector<pair<pair<int, int>, pair<int,int> > > p;
		context->getHandsPositions(p);
		vector<pair<int, int> > headsPositions;
		context->getHeadsPositions(headsPositions);
		for (int i = 0; i < headsPositions.size(); i++)
		{
			char strLabel[10];
			sprintf(strLabel, "%d", i + 1);
			putText(a, strLabel, Point(headsPositions[i].second-20, headsPositions[i].first+20), FONT_HERSHEY_SIMPLEX, 2, Scalar(250, 0, 100), 3);
			circle(a, Point(headsPositions[i].second, headsPositions[i].first), 30, Scalar(250, 0, 100), 3);
		}
		for (int i = 0; i < p.size(); i++)
		{
			Mat c;
			circle(a, Point(p[i].second.second, p[i].second.first), 15, Scalar(250, 0, 100), 3);
			HandSeparator asd(b, a, p[i].second.first, p[i].second.second, Point(headsPositions[i].second, headsPositions[i].first));
			int y1, y2, x1, x2;
			if (asd.separate(y1, y2, x1, x2))
			{
				y1 -= 10;
				x1 -= 10;

				y2 += 10;
				x2 += 10;

				x1 = max(x1, 0);
				y1 = max(y1, 0);
				y2 = min(y2, a.rows - 1);
				x2 = min(x2, a.cols - 1);
				Mat f = a(Range(y1, y2), Range(x1, x2));
				imshow("12", f);
				c = b(Range(y1, y2), Range(x1, x2));
				rg.addFrame(c);
				rg.predict(i);
			}
			else
			{
				rg.clear();
				Mat e = Mat::zeros(100, 100, CV_8UC3);
				imshow("1", e);
			}
			imshow("hand", a);
		}
		waitKey(10);
	}
}
