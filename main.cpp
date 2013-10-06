#include <opencv2/opencv.hpp>
#include <fstream>
#include <OpenNIContext.h>
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
		context->display();
	}
}
