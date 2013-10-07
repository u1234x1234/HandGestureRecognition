#ifndef OPENNICONTEXT_H
#define OPENNICONTEXT_H

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnPropNames.h>
#include "SceneDrawer.h"
#include <vector>

class OpenNIContext
{
public:
	OpenNIContext(int, char**);
	~OpenNIContext();
	void display (void);
	void update();
	void getImageMap(cv::Mat &image);
	void getDepthMap(cv::Mat &depth);
	void getHandsPositions(std::vector<std::pair<std::pair<int, int>, std::pair<int, int> > > &); // vector of users hands, left - first, right - second.
private:
	SceneDrawer *sceneDrawer;
	xn::Context context;
	xn::ScriptNode scriptNode;
	xn::DepthGenerator depthGenerator;
	xn::ImageGenerator imageGenerator;
	xn::UserGenerator userGenerator;
	xn::Player player;
};

#endif // OPENNICONTEXT_H
