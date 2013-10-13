#ifndef ROTATIONGESTURE_H
#define ROTATIONGESTURE_H

#include <opencv2/opencv.hpp>
#include <vector>
#include <ctime>
#include <deque>

class RotationGesture
{
public:
	RotationGesture();
	void addFrame(cv::Mat &src);
	void predict(int &result);
private:
	void findFingers(std::vector<std::pair<int, int> > &fingerPositions);
	cv::Mat depthMap;
	std::deque<int> d;
	std::deque<double> angles;
	clock_t timeOfLastGesture;
	const int fingersThreshhold = 30;
	int volume;
	int dir;
};

#endif // ROTATIONGESTURE_H
