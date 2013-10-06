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
private:
	SceneDrawer *sceneDrawer;
	xn::Context g_Context;
	xn::ScriptNode g_scriptNode;
	xn::DepthGenerator g_DepthGenerator;
	xn::ImageGenerator g_ImageGenerator;
	xn::UserGenerator g_UserGenerator;
	xn::Player g_Player;
};

#endif // OPENNICONTEXT_H
