#ifndef HANDSEPARATOR_H
#define HANDSEPARATOR_H

#include <opencv2/opencv.hpp>

class HandSeparator
{
public:
	HandSeparator(const cv::Mat &depth, const cv::Mat &image, int y, int x);
	bool separate(cv::Mat &result);
private:
	cv::Mat depth;
	cv::Mat image;
	int handPositionX;
	int handPositionY;
};

#endif // HANDSEPARATOR_H
