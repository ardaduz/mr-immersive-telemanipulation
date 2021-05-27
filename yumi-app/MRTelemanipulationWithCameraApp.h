// IMPORTANT NOTICE !
// THE CODE HERE IS NOT COMPLETE ON PURPOSE FOR COPYRIGHT REASONS (ARDA DUZCEKER)

#pragma once

// determine whether use head robot or not
#define HEAD

#include <ZEDVRLib/ZEDVRApplication.h>
#include <MRTelemanipulationWithCameraLib/TeleoperationRobot.h>
#include <MRTelemanipulationWithCameraLib/MirrorPlane.h>

#include <ControlLib/RobotControlInterface.h>
#include <YuMiLib/YuMiControlInterface.h>
#include <MRTelemanipulationWithCameraLib/ROSCommunication.h>
#include <thread>


#ifdef HEAD
#include <GUILib/GLApplication.h>
#include <ZEDVRLib/ZEDVRApplication.h>
#include <string>
#include <map>
#include <deque>
#include <GUILib/TranslateWidget.h>
#include <RBSimLib/AbstractRBEngine.h>
#include <RBSimLib/WorldOracle.h>
#include <GUILib/GLWindow3D.h>
#include <GUILib/GLWindow3DWithMesh.h>
#include <GUILib/GLWindowContainer.h>
#include <ControlLib/IK_Solver.h>
#include <XsensMTwLib/XsensMTwObject.h>
#include "DxlMotorGroupInterface.h"
#include "IK_VRViewArmJointLimitsObjective.h"
#include "IK_VRViewArmMyObjective.h"

using namespace	std;

class DXLMap {
public:
	//this is the id of the motor...
	int id = -1;
	double multiplier = 1;
	HingeJoint* j;

	DXLMap(int dxl_id, HingeJoint* j, double multiplier = 1) {
		this->id = dxl_id;
		this->j = j;
		this->multiplier = multiplier;
	}
};

#endif



class MRTelemanipulationWithCameraApp : public ZEDVRApplication {
private:
	// ROS communication interface
	ROSCommunication rosCom;

	//--- Simulated Robot
	TeleoperationRobot teleopRobot, teleopRobot_mirror;
	MirrorPlane mirrorPlane;
	double EE_shift = 0.05;


	//--- Physical Robot
	RobotControlInterface* rci = nullptr;
	bool syncPhysicalRobot = false;
	double desiredRobotFrameRate = 8.0;
	bool applySocketLock = false;
	Timer simuGripperTimer, yumiComTimer;

	YuMiConstants::PATHZONE currentZone = YuMiConstants::z200;
	YuMiConstants::STOPPOINTDATA currentStopPoint = YuMiConstants::inpos100;


	//--- VR
	bool syncWithVRController = false;
	bool applyMirrorMode = false;


	//--- Gripper
	YuMiControlInterface* yci = nullptr;
	bool gripInLeftBlocked = true;
	bool gripOutLeftBlocked = false;

	bool gripInRightBlocked = true;
	bool gripOutRightBlocked = false;

	bool closeLeftGripper = false;
	bool closeRightGripper = false;


	//--- Threads
	std::thread processThread;
	bool runProcessThread = false;


	//--- Buttons
	bool blockMenuButton = true;
	const unsigned int maxBlockCounter = 5;
	unsigned int menuButtonBlockCounterL = 0;
	unsigned int menuButtonBlockCounterR = 0;

	// items below are for the head (cemara) robot arm


public:
	MRTelemanipulationWithCameraApp();
	~MRTelemanipulationWithCameraApp(void);

	void process();
	void applyProcessThread();
	void updateSimuGripper();
	void communicateWithPhysicalRobot();

	void initializeTeleopRobots();

	void restart();

	void drawScene();
	void drawAuxiliarySceneInfo();

	bool onKeyEvent(int key, int action, int mods);
	bool onCharacterPressedEvent(int key, int mods);
	bool onMouseButtonEvent(int button, int action, int mods, double xPos, double yPos);
	bool onMouseMoveEvent(double xPos, double yPos);
	bool onMouseWheelScrollEvent(double xOffset, double yOffset);

	bool processCommandLine(const std::string& cmdLine);

	void setupMenu();
	void resetCamera();

	void updateEndEffectorsWithVRControllers();
	void updateMirror();

	void leftController_trackPad(double xValue, double yValue);
	void leftController_trackPadButton();
	void leftController_menuButton();
	void leftController_systemButton();
	void leftController_trigger(double axisValue);
	void leftController_grip();

	void rightController_trackPad(double xValue, double yValue);
	void rightController_trackPadButton();
	void rightController_menuButton();
	void rightController_systemButton();
	void rightController_trigger(double axisValue);
	void rightController_grip();

	#ifdef HEAD
private:
	Robot* robot = NULL;
	AbstractRBEngine* rbEngine = NULL;
	RobotState startState = RobotState(14);
	bool showMesh = false;
	bool showMOI = false;
	bool showRotationAxes = false;
	bool showCDPs = false;
	bool showSkeleton = true;
	bool showActualRobot = false;
	bool drawDebuggingTools = false;
	double dt;

	RigidBody* head = NULL;
	P3D defaultTarget;

	// IMPORTANT NOTICE !
	// THE REST OF THE CODE IS MISSING ON PURPOSE FOR COPYRIGHT REASONS (ARDA DUZCEKER)	
};
