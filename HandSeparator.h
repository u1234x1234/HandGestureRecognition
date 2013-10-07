#ifndef HANDSEPARATOR_H
#define HANDSEPARATOR_H

#include <opencv2/opencv.hpp>

class HandSeparator
{
public:
	HandSeparator(const cv::Mat &, const cv::Mat&, int y, int x);
	void separate(cv::Mat &result);
private:
	cv::Mat depth;
	cv::Mat image;
	int handPositionX;
	int handPositionY;
};

#endif // HANDSEPARATOR_H
