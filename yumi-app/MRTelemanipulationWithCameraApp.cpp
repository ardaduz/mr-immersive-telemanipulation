// IMPORTANT NOTICE !
// THE CODE HERE IS NOT COMPLETE ON PURPOSE FOR COPYRIGHT REASONS (ARDA DUZCEKER)


#include "MRTelemanipulationWithCameraApp.h"

#include <GUILib/GLUtils.h>
#include <GUILib/GLTrackingCamera.h>

#ifdef HEAD
#include <GUILib/GLMesh.h>
#include <GUILib/GLContentManager.h>
#include <MathLib/MathLib.h>
#include <RBSimLib/ODERBEngine.h>
#include <ControlLib/SimpleLimb.h>
#include <ControlLib/IK_SphereCDPObjective.h>
#define PORT "COM3" // Set the correct COM port here
#endif

MRTelemanipulationWithCameraApp::MRTelemanipulationWithCameraApp() {
	bgColorR = bgColorG = bgColorB = bgColorA = 1; //Makes background white
	setWindowTitle("Mixed Reality Telemanipulation App");

	// Added by Arda Duzceker to enable simulation to visualize robot meshes
	// Behavioral correctness not guaranteed
	setup = true;

	desiredFrameRate = 100;
	showGroundPlane = false;
	useTracker = false;

	initializeTeleopRobots();

	delete rci;
	rci = new YuMiControlInterface(teleopRobot.robot);

	delete yci;
	yci = new YuMiControlInterface(teleopRobot_mirror.robot);

	#ifdef HEAD
	char* fName = "../data/rbs/VRViewArmRobot.rbs";
	Logger::consolePrint("Loading file\'%s\'...\n", fName);
	loadRobot(fName);

	motorBounds = {
		make_pair(1000, 3096),
		make_pair(1000, 3096),
		make_pair(1000, 3096),
		make_pair(1100, 2800),
		make_pair(1100, 2800),
		make_pair(1000, 3096),
		make_pair(1000, 3096),
		make_pair(1000, 3096)
	};

	tracker = new XsensMTwObject();
	#endif

	setupMenu();
	resetCamera();
	rosCom.Init(&teleopRobot, dynamic_cast<YuMiControlInterface*>(rci), &vrTargetOrientation, EE_shift);

	Logger::consoleOutput.clear();
}


MRTelemanipulationWithCameraApp::~MRTelemanipulationWithCameraApp(void) {
	delete rci;
	delete yci;
	rosCom.Stop();

	#ifdef HEAD
	delete rbEngine;
	delete robot;
	tracker->closeConnection();
	torquesOff();
	delete tracker;
	#endif
}


void MRTelemanipulationWithCameraApp::restart() {
	teleopRobot.reset();
	teleopRobot_mirror.reset();
	#ifdef HEAD
	loadFile("../data/rbs/VRViewArmRobot.rbs");
	#endif
}


void MRTelemanipulationWithCameraApp::process() {
	if (setup) {
		//updateSimuGripper();

		if (syncWithVRController)	mainMenu->refresh();

		if (mrIsEnabled && syncWithVRController)	updateEndEffectorsWithVRControllers();

		if (applyMirrorMode)	updateMirror();

		if (syncPhysicalRobot)	communicateWithPhysicalRobot();

		#ifdef HEAD
		processHeadRobot();
		subMenu->refresh();
		#endif
	}
}




void MRTelemanipulationWithCameraApp::applyProcessThread() {
	while (runProcessThread) {
		if (applyMirrorMode) {
			teleopRobot_mirror.updateEndEffectorTargets(EE_shift, Eigen::Matrix3d::Identity() * Eigen::AngleAxisd(teleopRobot_mirror.robot->getHeadingAngle(), Eigen::Vector3d::UnitZ()));
			teleopRobot_mirror.ikSolver->solve();
		}

		teleopRobot.updateEndEffectorTargets(EE_shift, Eigen::Matrix3d::Identity());
		teleopRobot.ikSolver->solve();
	}
}


void MRTelemanipulationWithCameraApp::updateSimuGripper() {
	if (closeLeftGripper) {
		if (applyMirrorMode) {
			yci->leftABBGripper.runGripIn = true;
			yci->leftABBGripper.runGripOut = false;

			dynamic_cast<YuMiControlInterface*>(rci)->rightABBGripper.runGripIn = true;
			dynamic_cast<YuMiControlInterface*>(rci)->rightABBGripper.runGripOut = false;
		}
		else {
			dynamic_cast<YuMiControlInterface*>(rci)->leftABBGripper.runGripIn = true;
			dynamic_cast<YuMiControlInterface*>(rci)->leftABBGripper.runGripOut = false;
		}

		if (syncPhysicalRobot && !gripInLeftBlocked) {
			gripInLeftBlocked = true;
			gripOutLeftBlocked = false;
			rci->grip("left");
		}
	}
	else {
		if (applyMirrorMode) {
			yci->leftABBGripper.runGripIn = false;
			yci->leftABBGripper.runGripOut = true;

			dynamic_cast<YuMiControlInterface*>(rci)->rightABBGripper.runGripIn = false;
			dynamic_cast<YuMiControlInterface*>(rci)->rightABBGripper.runGripOut = true;
		}
		else {
			dynamic_cast<YuMiControlInterface*>(rci)->leftABBGripper.runGripIn = false;
			dynamic_cast<YuMiControlInterface*>(rci)->leftABBGripper.runGripOut = true;
		}

		if (syncPhysicalRobot && !gripOutLeftBlocked) {
			gripInLeftBlocked = false;
			gripOutLeftBlocked = true;
			rci->grip("left");
		}
	}

	if (closeRightGripper) {
		if (applyMirrorMode) {
			yci->rightABBGripper.runGripIn = true;
			yci->rightABBGripper.runGripOut = false;

			dynamic_cast<YuMiControlInterface*>(rci)->leftABBGripper.runGripIn = true;
			dynamic_cast<YuMiControlInterface*>(rci)->leftABBGripper.runGripOut = false;
		}
		else {
			dynamic_cast<YuMiControlInterface*>(rci)->rightABBGripper.runGripIn = true;
			dynamic_cast<YuMiControlInterface*>(rci)->rightABBGripper.runGripOut = false;
		}

		if (syncPhysicalRobot && !gripInRightBlocked) {
			gripInRightBlocked = true;
			gripOutRightBlocked = false;
			rci->grip("right");
		}
	}
	else {
		if (applyMirrorMode) {
			yci->rightABBGripper.runGripIn = false;
			yci->rightABBGripper.runGripOut = true;

			dynamic_cast<YuMiControlInterface*>(rci)->leftABBGripper.runGripIn = false;
			dynamic_cast<YuMiControlInterface*>(rci)->leftABBGripper.runGripOut = true;
		}
		else {
			dynamic_cast<YuMiControlInterface*>(rci)->rightABBGripper.runGripIn = false;
			dynamic_cast<YuMiControlInterface*>(rci)->rightABBGripper.runGripOut = true;
		}

		if (syncPhysicalRobot && !gripOutRightBlocked) {
			gripInRightBlocked = false;
			gripOutRightBlocked = true;
			rci->grip("right");
		}
	}

	dynamic_cast<YuMiControlInterface*>(rci)->updateSimuGrippers(1.0/simuGripperTimer.timeEllapsed());
	if(applyMirrorMode)	yci->updateSimuGrippers(1.0 / simuGripperTimer.timeEllapsed());
	simuGripperTimer.restart();
}


void MRTelemanipulationWithCameraApp::communicateWithPhysicalRobot() {
	if (!applySocketLock) {
		dynamic_cast<YuMiControlInterface*>(rci)->sendSocketResponse = false;
		if (yumiComTimer.timeEllapsed() >= 1.0 / desiredRobotFrameRate) {
			if (rci && syncPhysicalRobot)	rci->syncPhysicalRobotWithSimRobot(1.0 / desiredRobotFrameRate);
			yumiComTimer.restart();
		}
	}
	else {
		dynamic_cast<YuMiControlInterface*>(rci)->sendSocketResponse = true;
		if (rci && syncPhysicalRobot)	rci->syncPhysicalRobotWithSimRobot(yumiComTimer.timeEllapsed());
		yumiComTimer.restart();
	}
}


void MRTelemanipulationWithCameraApp::initializeTeleopRobots() {
	teleopRobot.init(useTracker);
	teleopRobot_mirror.init(useTracker);

	teleopRobot.updateRobotRootState(P3D(1.0, 1.0, 0.0), -PI / 2.0);
	teleopRobot_mirror.updateRobotRootState(P3D(1.0, 1.0, 1.9), PI / 2.0);

	teleopRobot.setupGripperAndElbows(Eigen::Matrix3d::Identity()*Eigen::AngleAxisd(-PI / 2.0, Eigen::Vector3d::UnitX()));
	teleopRobot_mirror.setupGripperAndElbows(Eigen::Matrix3d::Identity()*Eigen::AngleAxisd(-PI / 2.0, Eigen::Vector3d::UnitX()));
	updateMirror();
}


void MRTelemanipulationWithCameraApp::drawScene() {
	if (setup) {
		teleopRobot.drawRobot();
		if (applyMirrorMode) {
			teleopRobot_mirror.drawRobot();
		}

		#ifdef HEAD
		drawSceneHeadRobot();
		#endif
	}
}


void MRTelemanipulationWithCameraApp::drawAuxiliarySceneInfo() {
	teleopRobot.drawGripperAndElbows();
	if (applyMirrorMode) {
		teleopRobot_mirror.drawGripperAndElbows();
	}
}


bool MRTelemanipulationWithCameraApp::processCommandLine(const std::string& cmdLine) {
	if (GLApplication::processCommandLine(cmdLine)) return true;

	return false;
}

// IMPORTANT NOTICE !
// THE REST OF THE CODE IS MISSING ON PURPOSE FOR COPYRIGHT REASONS (ARDA DUZCEKER)
