//#include <VRTeleoperationLib/TeleoperationRobot.h>
#include <MRTelemanipulationWithCameraLib\TeleoperationRobot.h>

#include <GUILib/GLUtils.h>
#include <YuMiLib/IK_YuMiJointLimitsObjective.h>
#include <RBSimLib/ODERBEngine.h>

#include <ControlLib/IK_EndEffectorsObjective.h>
#include <ControlLib/IK_RobotStateRegularizer.h>
#include <ControlLib/IK_SphereCDPObjective.h>


TeleoperationRobot::~TeleoperationRobot(void) {
	delete rbEngine;
	delete robot;
	delete ikSolver;
}


void TeleoperationRobot::init(bool useTracker) {
	this->useTracker = useTracker;
	initRobot();
}


void TeleoperationRobot::initRobot() {
	delete robot;
	delete rbEngine;
	delete ikSolver;

	rbEngine = new ODERBEngine();
	rbEngine->loadRBsFromFile(robotPath.c_str());
	robot = new Robot(rbEngine->rbs[0]);
	setupSimpleRobotStructure(robot);
	ikSolver = new IK_Solver(robot, true);

	setupYuMiCollisionSpheres();

	std::vector<double> yumiJointLimits = { -2.940880, 2.940880,  -2.940880, 2.940880, -2.504547, 0.759218, -2.504547, 0.759218, -2.940880, 2.940880, -2.940880, 2.940880, -1.300000, 1.396263,
                                                -1.30000, 1.396263, -5.061455, 5.061455, -5.061455, 5.061455, -1.535897, 2.4085544, -1.535897, 2.4085544, -3.996804, 3.996804, -3.996804, 3.996804 };

	ikSolver->ikEnergyFunction->regularizer = 100; //Default: 100
	ikSolver->ikOptimizer->checkDerivatives = false;
	ikSolver->ikPlan->optimizeRootConfiguration = false;

	ikSolver->ikEnergyFunction->objectives.clear();
	ikSolver->ikEnergyFunction->objectives.push_back(new IK_EndEffectorsObjective(ikSolver->ikPlan, "EndEffectorTargets", 1000));
	ikSolver->ikEnergyFunction->objectives.push_back(new IK_RobotStateRegularizer(ikSolver->ikPlan, 6, ikSolver->ikPlan->gcRobotRepresentation->getDimensionCount() - 1, "JointAnglesRegularizer", 1));
	ikSolver->ikEnergyFunction->objectives.push_back(new IK_SphereCDPObjective(ikSolver->ikPlan, "SphereCollisionAvoidance", 10000));
	ikSolver->ikEnergyFunction->objectives.push_back(new IK_YuMiJointLimitsObjective(ikSolver->ikPlan, yumiJointLimits, "YuMiJointLimits", 1000.0));
	if (!useTracker) ikSolver->ikEnergyFunction->objectives.push_back(new IK_EndEffectorsObjective(ikSolver->ikPlan, true, "EndEffectorRegularizers", 10));

	RobotState rs(robot);
	rs.readFromFile(homeStatePath.c_str());
	robot->setState(&rs);
	ikSolver->ikPlan->setTargetIKStateFromRobot();

	setupYuMiEndEffectors();
}


void TeleoperationRobot::reset() {
	initRobot();
}


void TeleoperationRobot::drawRobot() {
	glColor3d(1, 1, 1);
	glDisable(GL_LIGHTING);

	//int flags = SHOW_ABSTRACT_VIEW | HIGHLIGHT_SELECTED;
	int flags = HIGHLIGHT_SELECTED;

	if (showAbstractView)
		flags |= SHOW_ABSTRACT_VIEW;

	if (showMesh)
		flags |= SHOW_MESH;
	if (showMOI) {
		flags |= SHOW_MOI_BOX;
		Logger::consolePrint("total mass: %lf\n", robot->mass);
	}
	if (showRotationAxes)
		flags |= SHOW_JOINTS;
	if (showCDPs)
		flags |= SHOW_CD_PRIMITIVES;

	glEnable(GL_LIGHTING);
	glPushMatrix();

	rbEngine->drawRBs(flags);

	if (drawCollisionSpheres) {
		for (IK_SphereCDP& sphere : ikSolver->ikPlan->sphereCDPs) {
			P3D globalPos = sphere.rb->getWorldCoordinates(sphere.localCoords);
			glColor3d(1.0, 0.0, 0.0);
			drawSphere(globalPos, sphere.radius);
		}
	}

	glPopMatrix();
}


void TeleoperationRobot::drawGripperAndElbows() {
	leftElbow.draw();
	rightElbow.draw();

	leftGripper.draw();
	rightGripper.draw();
}


void TeleoperationRobot::updateEndEffectorTargets(double EE_shift, const Eigen::Matrix3d& posRotShift) {
	if (useTracker) {
		ikSolver->ikPlan->endEffectors[RIGHT_ELBOW_EE].setTargetPosition(rightElbow.position);
		ikSolver->ikPlan->endEffectors[LEFT_ELBOW_EE].setTargetPosition(leftElbow.position);
	}
	else {
		ikSolver->ikPlan->EE_regularizers[RIGHT_HAND_EE].setTargetPosition(rightElbow.position); //Use hand for indexing...
		ikSolver->ikPlan->EE_regularizers[LEFT_HAND_EE].setTargetPosition(leftElbow.position);
	}

	Eigen::Vector3d positionShift = EE_globalRotMat * posRotShift * Eigen::Vector3d(EE_shift, 0.0, 0.0);

	ikSolver->ikPlan->endEffectors[LEFT_HAND_EE].setTargetPosition(leftGripper.position - P3D(leftGripper.rotation*positionShift));
	ikSolver->ikPlan->endEffectors[LEFT_HAND_EE].setTargetOrientation(leftGripper.rotation*EE_globalRotMat);

	ikSolver->ikPlan->endEffectors[RIGHT_HAND_EE].setTargetPosition(rightGripper.position - P3D(rightGripper.rotation*positionShift));
	ikSolver->ikPlan->endEffectors[RIGHT_HAND_EE].setTargetOrientation(rightGripper.rotation*EE_globalRotMat);
}


void TeleoperationRobot::updateRobotRootState(P3D offset, double heading) {
	for (int i = 0; i < robot->getRigidBodyCount(); i++) {
		RigidBody* rb = robot->getRigidBody(i);
		rb->setCMPosition(rb->getCMPosition() + offset);
	}
	robot->setHeading(heading);
}


void TeleoperationRobot::setupYuMiEndEffectors() {
	//Setup ikPlan
	ikSolver->ikPlan->endEffectors.clear();
	ikSolver->ikPlan->EE_regularizers.clear();

	ikSolver->ikPlan->endEffectors.push_back(IK_EndEffector());
	ikSolver->ikPlan->endEffectors.push_back(IK_EndEffector());

	ikSolver->ikPlan->endEffectors.at(RIGHT_HAND_EE).endEffectorRB = robot->getRBByName(EE_strings[RIGHT_HAND_EE].c_str());
	ikSolver->ikPlan->endEffectors.at(LEFT_HAND_EE).endEffectorRB = robot->getRBByName(EE_strings[LEFT_HAND_EE].c_str());

	if (useTracker) {
		ikSolver->ikPlan->endEffectors.push_back(IK_EndEffector());
		ikSolver->ikPlan->endEffectors.push_back(IK_EndEffector());

		ikSolver->ikPlan->endEffectors.at(RIGHT_ELBOW_EE).endEffectorRB = robot->getRBByName(EE_strings[RIGHT_ELBOW_EE].c_str());
		ikSolver->ikPlan->endEffectors.at(LEFT_ELBOW_EE).endEffectorRB = robot->getRBByName(EE_strings[LEFT_ELBOW_EE].c_str());
	}
	else {
		ikSolver->ikPlan->EE_regularizers.push_back(IK_EndEffector());
		ikSolver->ikPlan->EE_regularizers.push_back(IK_EndEffector());

		ikSolver->ikPlan->EE_regularizers.at(RIGHT_HAND_EE).endEffectorRB = robot->getRBByName(EE_strings[RIGHT_ELBOW_EE].c_str()); //Use hand for indexing...
		ikSolver->ikPlan->EE_regularizers.at(LEFT_HAND_EE).endEffectorRB = robot->getRBByName(EE_strings[LEFT_ELBOW_EE].c_str());
	}


	//---Setup orientations
	//double EE_shift = 0.125; //DEFAULT: 0.145
	//V3D rightEE_localTip = V3D(0.0200485, -0.0189025, -0.0217355).toUnit()*EE_shift;
	//V3D leftEE_localTip = V3D(0.017977, -0.0169495, 0.01949).toUnit()*EE_shift;

	V3D dir_1_r = V3D(0.627, 0.084, 0.344) - V3D(0.619, 0.076, 0.351);
	V3D dir_2_r = V3D(0.656349, 0.0844, 0.348425) - V3D(0.634457, 0.093577, 0.335758);
	dir_1_r.toUnit();
	dir_2_r.toUnit();
	V3D dir_3_r = dir_1_r.cross(dir_2_r);

	Eigen::Matrix3d dirMat_r;
	dirMat_r.col(0) = dir_1_r;
	dirMat_r.col(1) = dir_2_r;
	dirMat_r.col(2) = dir_3_r;

	rightEE_localRotMat = ikSolver->ikPlan->endEffectors.at(RIGHT_HAND_EE).endEffectorRB->meshTransformations[0].R * dirMat_r;
	ikSolver->ikPlan->endEffectors.at(RIGHT_HAND_EE).setLocalOrientation(rightEE_localRotMat);


	V3D dir_1_l = V3D(0.647, -0.075, 0.356) - V3D(0.639, -0.067, 0.363);
	V3D dir_2_l = V3D(0.656353, -0.084374, 0.3484) - V3D(0.634453, -0.093602, 0.335783);
	dir_1_l.toUnit();
	dir_2_l.toUnit();
	V3D dir_3_l = dir_1_l.cross(dir_2_l);

	Eigen::Matrix3d dirMat_l;
	dirMat_l.col(0) = dir_1_l;
	dirMat_l.col(1) = dir_2_l;
	dirMat_l.col(2) = dir_3_l;

	leftEE_localRotMat = ikSolver->ikPlan->endEffectors.at(LEFT_HAND_EE).endEffectorRB->meshTransformations[0].R * dirMat_l;
	ikSolver->ikPlan->endEffectors.at(LEFT_HAND_EE).setLocalOrientation(leftEE_localRotMat);
}


void TeleoperationRobot::setupYuMiCollisionSpheres() {
	ikSolver->ikPlan->sphereCDPs.clear();
	//for (unsigned int i = 0; i < robot->getRigidBodyCount(); i++) {
	//	ikSolver->ikPlan->sphereCDPs.push_back(IK_SphereCDP(robot->getRigidBody(i), P3D(0.0, 0.0, 0.0), 0.075));
	//}
	ikSolver->ikPlan->sphereCDPs.push_back(IK_SphereCDP(robot->getRBByName("link_1_r"), P3D(0.0, 0.0, 0.0), 0.075));
	ikSolver->ikPlan->sphereCDPs.push_back(IK_SphereCDP(robot->getRBByName("link_2_r"), P3D(0.0, 0.0, 0.0), 0.075));
	ikSolver->ikPlan->sphereCDPs.push_back(IK_SphereCDP(robot->getRBByName("link_3_r"), P3D(0.0, 0.0, 0.0), 0.075));
	ikSolver->ikPlan->sphereCDPs.push_back(IK_SphereCDP(robot->getRBByName("link_4_r"), P3D(0.0, 0.0, 0.0), 0.075));
	ikSolver->ikPlan->sphereCDPs.push_back(IK_SphereCDP(robot->getRBByName("link_5_r"), P3D(0.0, 0.0, 0.0), 0.075));
	//ikSolver->ikPlan->sphereCDPs.push_back(IK_SphereCDP(robot->getRBByName("link_6_r"), P3D(0.0, 0.0, 0.0), 0.075));
	//ikSolver->ikPlan->sphereCDPs.push_back(IK_SphereCDP(robot->getRBByName("link_7_r"), P3D(V3D(0.0200485, -0.0189025, -0.0217355).toUnit()*0.09), 0.075));

	ikSolver->ikPlan->sphereCDPs.push_back(IK_SphereCDP(robot->getRBByName("link_1_l"), P3D(0.0, 0.0, 0.0), 0.075));
	ikSolver->ikPlan->sphereCDPs.push_back(IK_SphereCDP(robot->getRBByName("link_2_l"), P3D(0.0, 0.0, 0.0), 0.075));
	ikSolver->ikPlan->sphereCDPs.push_back(IK_SphereCDP(robot->getRBByName("link_3_l"), P3D(0.0, 0.0, 0.0), 0.075));
	ikSolver->ikPlan->sphereCDPs.push_back(IK_SphereCDP(robot->getRBByName("link_4_l"), P3D(0.0, 0.0, 0.0), 0.075));
	ikSolver->ikPlan->sphereCDPs.push_back(IK_SphereCDP(robot->getRBByName("link_5_l"), P3D(0.0, 0.0, 0.0), 0.075));
	//ikSolver->ikPlan->sphereCDPs.push_back(IK_SphereCDP(robot->getRBByName("link_6_l"), P3D(0.0, 0.0, 0.0), 0.075));
	//ikSolver->ikPlan->sphereCDPs.push_back(IK_SphereCDP(robot->getRBByName("link_7_l"), P3D(V3D(0.017977, -0.0169495, 0.01949).toUnit()*0.09), 0.075));
}


void TeleoperationRobot::setupGripperAndElbows(const Eigen::Matrix3d& EE_globalRotMat) {
	this->EE_globalRotMat = EE_globalRotMat;

	P3D globalElbowShift(0.0, 0.0, -0.1);

	leftElbow.position = robot->getRBByName(EE_strings[LEFT_ELBOW_EE].c_str())->getWorldCoordinates(P3D(0.0, 0.0, 0.0)) + globalElbowShift;
	leftElbow.tWidget->pos = leftElbow.position;
	leftElbow.drawColor = P3D(0.0, 1.0, 0.0);

	rightElbow.position = robot->getRBByName(EE_strings[RIGHT_ELBOW_EE].c_str())->getWorldCoordinates(P3D(0.0, 0.0, 0.0)) + globalElbowShift;
	rightElbow.tWidget->pos = rightElbow.position;
	rightElbow.drawColor = P3D(0.0, 0.0, 1.0);

	leftGripper.position = ikSolver->ikPlan->endEffectors[LEFT_HAND_EE].endEffectorRB->getWorldCoordinates(P3D(0.0, 0.0, 0.0));
	leftGripper.tWidget->pos = leftGripper.position;
	leftGripper.rWidget->pos = leftGripper.position;
	leftGripper.drawColor = P3D(0.0, 1.0, 0.0);

	//Quaternion q_left = ikSolver->ikPlan->endEffectors[LEFT_HAND_EE].endEffectorRB->getOrientation();
	//leftGripper.rotation = q_left.getRotationMatrix() * leftEE_localRotMat * -EE_globalRotMat;
	//q_left.setRotationFrom(leftGripper.rotation);
	//leftGripper.rWidget->setOrientation(q_left);
	Quaternion q_left;
	q_left.setRotationFrom(Eigen::Matrix3d::Identity()*Eigen::AngleAxisd(robot->getHeadingAngle(), Eigen::Vector3d::UnitY()));
	leftGripper.rWidget->setOrientation(leftGripper.rWidget->getOrientation()*q_left);
	leftGripper.rotation = leftGripper.rWidget->getOrientation().getRotationMatrix();
	
	rightGripper.position = ikSolver->ikPlan->endEffectors[RIGHT_HAND_EE].endEffectorRB->getWorldCoordinates(P3D(0.0, 0.0, 0.0));
	rightGripper.tWidget->pos = rightGripper.position;
	rightGripper.rWidget->pos = rightGripper.position;
	rightGripper.drawColor = P3D(0.0, 0.0, 1.0);

	//Quaternion q_right = ikSolver->ikPlan->endEffectors[RIGHT_HAND_EE].endEffectorRB->getOrientation();
	//rightGripper.rotation = q_right.getRotationMatrix() * rightEE_localRotMat * -EE_globalRotMat;
	//q_right.setRotationFrom(rightGripper.rotation);
	//rightGripper.rWidget->setOrientation(q_right);
	Quaternion q_right;
	q_right.setRotationFrom(Eigen::Matrix3d::Identity()*Eigen::AngleAxisd(robot->getHeadingAngle(), Eigen::Vector3d::UnitY()));
	rightGripper.rWidget->setOrientation(rightGripper.rWidget->getOrientation()*q_right);
	rightGripper.rotation = rightGripper.rWidget->getOrientation().getRotationMatrix();
}


bool TeleoperationRobot::onMouseMoveEvent(double xPos, double yPos) {
	if (leftElbow.onMouseMoveEvent(xPos, yPos))		return true;
	if (rightElbow.onMouseMoveEvent(xPos, yPos))	return true;
	if (leftGripper.onMouseMoveEvent(xPos, yPos))	return true;
	if (rightGripper.onMouseMoveEvent(xPos, yPos))	return true;
	return false;
}


bool TeleoperationRobot::onMouseButtonEvent(int button, int action) {
	return false;
}


bool TeleoperationRobot::onKeyEvent(int key, int action, int mods) {
	if (key == GLFW_KEY_T && action == GLFW_PRESS) {
		leftElbow.tWidget->visible = true;
		rightElbow.tWidget->visible = true;

		leftGripper.tWidget->visible = true;
		leftGripper.rWidget->visible = true;
		rightGripper.tWidget->visible = true;
		rightGripper.rWidget->visible = true;
	}

	if (key == GLFW_KEY_T && action == GLFW_RELEASE) {
		leftElbow.tWidget->visible = false;
		rightElbow.tWidget->visible = false;

		leftGripper.tWidget->visible = false;
		leftGripper.rWidget->visible = false;
		rightGripper.tWidget->visible = false;
		rightGripper.rWidget->visible = false;
	}

	return false;
}
