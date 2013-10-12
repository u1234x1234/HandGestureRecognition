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
			//circle(a, Point(p[i].second.second, p[i].second.first), 30, Scalar(250, 50, 50), 5 );
			Mat c;
			HandSeparator asd(b, a, p[i].second.first, p[i].second.second);
			int y1, y2, x1, x2;
			if (asd.separate(y1, y2, x1, x2))
			{
				c = b(Range(y1, y2), Range(x1, x2));
				//imshow("hand", c);
				RotationGesture rg(c);
				vector<pair<int, int> > pos;
				rg.findFingers(pos);
				Mat asd = Mat::zeros(x2-x1, y2-y1, CV_8UC3);
				for (int i = 0; i < pos.size(); i++)
					circle(asd, Point( pos[i].second,  pos[i].first), 2, Scalar(100,100,250));
				imshow("1", asd);
			}
			imshow("hand", a);
		}
		//imshow("image", b);
		waitKey(10);
	}
}
