#include <opencv2/opencv.hpp>
#include <OpenNIContext.h>
#include "HandSeparator.h"
#include "RotationGesture.h"

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
		//context->display();
		Mat a;
		context->getImageMap(a);
		Mat b;
		context->getDepthMap(b);
		vector<pair<pair<int, int>, pair<int,int> > > p;
		context->getHandsPositions(p);
		for (int i = 0; i < p.size(); i++)
		{
			Mat c;
			HandSeparator asd(b, a, p[i].second.first, p[i].second.second);
			int y1, y2, x1, x2;
			if (asd.separate(y1, y2, x1, x2))
			{
				c = b(Range(y1, y2), Range(x1, x2));
				rg.addFrame(c);
				rg.predict(i);
			}
			imshow("hand", a);
		}
		waitKey(10);
	}
}
