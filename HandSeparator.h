#ifndef HANDSEPARATOR_H
#define HANDSEPARATOR_H

#include <opencv2/opencv.hpp>

class HandSeparator
{
public:
	HandSeparator(const cv::Mat &depth, const cv::Mat &image, int y, int x);
	bool separate(int &yMin, int &yMax, int &xMin, int &xMax);
	bool separate(cv::Mat &dest);
private:
	cv::Mat depth;
	cv::Mat image;
	int handPositionX;
	int handPositionY;
	cv::Mat resImage;
};

#endif // HANDSEPARATOR_H
