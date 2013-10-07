#ifndef OPENNICONTEXT_H
#define OPENNICONTEXT_H

#include <XnOpenNI.h>
#include <XnCodecIDs.h>
#include <XnPropNames.h>
#include "SceneDrawer.h"

class OpenNIContext
{
public:
	OpenNIContext(int, char**);
	~OpenNIContext();
	void display (void);
	void update();
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
