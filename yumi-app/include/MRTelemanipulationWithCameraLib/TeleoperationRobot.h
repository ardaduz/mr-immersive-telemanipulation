#pragma once

#include <ControlLib/IK_Solver.h>
#include <ControlLib/RobotControlInterface.h>

#include <RBSimLib/AbstractRBEngine.h>

#include <YuMiLib/YuMiConstants.h>

#include "Elbow.h"
#include "Gripper.h"

#include <string>


class TeleoperationRobot {
public:
	std::string robotPath = "../data/rbs/yumi/yumi_simplified.rbs";
	std::string homeStatePath = "../data/rbs/yumi/yumiHomeState.rs";

	Robot* robot = nullptr;
	AbstractRBEngine* rbEngine = nullptr;
	IK_Solver* ikSolver = nullptr;

	bool showMesh = true;
	bool showMOI = false;
	bool showRotationAxes = false;
	bool showCDPs = false;
	bool showAbstractView = false;
	bool drawCollisionSpheres = false;

	enum ENDEFFECTORS {
		RIGHT_HAND_EE = 0,
		LEFT_HAND_EE = 1,
		RIGHT_ELBOW_EE = 2,
		LEFT_ELBOW_EE = 3
	};
	
	std::vector<std::string> EE_strings = std::vector<std::string>{ "link_7_r", "link_7_l", "link_4_r", "link_4_l" };

	Eigen::Matrix3d rightEE_localRotMat, leftEE_localRotMat, EE_globalRotMat; //Set in setupYuMiEndEffectors()

	Elbow leftElbow, rightElbow;
	Gripper leftGripper, rightGripper;
	bool useTracker;

public:
	TeleoperationRobot() {}
	~TeleoperationRobot();

	void init(bool useTracker);
	void initRobot();
	void reset();

	void drawRobot();
	void drawGripperAndElbows();
	void updateEndEffectorTargets(double EE_shift, const Eigen::Matrix3d& posRotShift);
	void updateRobotRootState(P3D offset, double heading);
	
	void setupYuMiEndEffectors();
	void setupYuMiCollisionSpheres();

	void setupGripperAndElbows(const Eigen::Matrix3d& EE_globalRotMat);

	bool onMouseMoveEvent(double xPos, double yPos);
	bool onMouseButtonEvent(int button, int action);
	bool onKeyEvent(int key, int action, int mods);
};
