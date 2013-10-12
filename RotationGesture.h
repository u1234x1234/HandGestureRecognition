#ifndef ROTATIONGESTURE_H
#define ROTATIONGESTURE_H

#include <opencv2/opencv.hpp>
#include <vector>

class RotationGesture
{
public:
	RotationGesture(cv::Mat &src);
	void findFingers(std::vector<std::pair<int, int> > &fingerPositions);
private:
	cv::Mat depthMap;
};

#endif // ROTATIONGESTURE_H
