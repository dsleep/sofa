#ifndef SPEECHTOTEXT_H
#define SPEECHTOTEXT_H

#include <sofa/core/objectmodel/BaseObject.h>
#include <sofa/core/objectmodel/MouseEvent.h> 
#include "initSurfLabSpeechRecognition.h"

#include <SofaUserInteraction/Controller.h>
#include <sofa/core/behavior/BaseController.h>

#include <SofaBaseVisual/BaseCamera.h>

//test linking haptic
#include <sofa/core/objectmodel/HapticDeviceEvent.h>
#include <sofa/simulation/Node.h>
#include <sofa/simulation/Simulation.h>

namespace sofa
{

namespace simulation { class Node; }

namespace component
{
namespace visualModel { class OglModel; }

namespace controller{

using namespace sofa::defaulttype;
using core::objectmodel::Data;

class SOFA_SURFLABSPEECHRECOGNITION_API SpeechToText : public Controller
{
public:
	SOFA_CLASS(SpeechToText, Controller);

	SpeechToText();
	~SpeechToText();

	void recognize_speech();
	virtual void init();
	virtual void bwdInit();
	void handleEvent(core::objectmodel::Event *);

	void setSTTThreadCreated(bool b) { STTThreadCreated = b; }
	void setMoveMode(int m) { moveMode = m; }
	void setMoveCount(int m) { moveCount = m; }
	
	int confidence;
	helper::system::thread::CTime *thTimer;
	sofa::component::visualmodel::BaseCamera::SPtr currentCamera;
	int pos_x, pos_y, wheelDelta;
	int moveCount = 0;
	std::string base_path = "";
	sofa::simulation::Node::SPtr groot;
	bool enableSpeechRec = false;

	enum Mode { LEFT, RIGHT, UP, DOWN, SLIGHTLY_LEFT, SLIGHTLY_RIGHT, SLIGHTLY_UP, SLIGHTLY_DOWN,
		ROTATE_LEFT, ROTATE_RIGHT, ROTATE_UP, ROTATE_DOWN, ZOOM_IN, ZOOM_OUT, SWITCH, 
		SWITCH_TO_CAUTERIZER, SWITCH_TO_GRASPER, SWITCH_TO_SCISSOR, SWITCH_TO_DISSECTOR, 
		SWITCH_TO_BAG, SWITCH_TO_SCALPEL, SWITCH_TO_STAPLER, SWITCH_TO_CLAMP, SWITCH_TO_RETRACTOR,
		UNRECOGNIZED = 9999
	};

private:
	bool STTThreadCreated;
	int moveMode = -1;
};

} // namespace controller

} // namespace component

} // namespace sofa
#endif