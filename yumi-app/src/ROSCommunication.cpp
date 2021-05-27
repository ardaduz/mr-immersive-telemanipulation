#include <MRTelemanipulationWithCameraLib/ROSCommunication.h>
#include <cv_bridge/cv_bridge.h>

ROSCommunication::ROSCommunication() {}

ROSCommunication::~ROSCommunication() {}

/**
 *	Initializes the ROSCommunication to operate on the given TeleoperationRobot instance. Has to be called once before starting the communication thread.
 */
void ROSCommunication::Init(TeleoperationRobot* pRobot, YuMiControlInterface* pYumiInterface, Quaternion* robotHeadTargetOrientation, double EE_shift) {
	this->pSimulationRobot = pRobot;
	this->pYumiInterface = pYumiInterface;
	this->robotHeadTargetOrientation = robotHeadTargetOrientation;

	this->EE_shift = EE_shift;
	this->futureRobot.init(this->pSimulationRobot->useTracker);
	this->futureRobot.updateRobotRootState(P3D(1.0, 1.0, 0.0), -PI / 2.0);
	this->futureRobot.setupGripperAndElbows(Eigen::Matrix3d::Identity() * Eigen::AngleAxisd(-PI / 2.0, Eigen::Vector3d::UnitX()));

	this->meshPosePublisher = nodeHandle.advertise<geometry_msgs::PoseArray>("robot/mesh_pose", 1, true);
	this->globalPosePublisher = nodeHandle.advertise<geometry_msgs::PoseArray>("robot/global_pose", 1, true);
	this->trajectoryMeshPosesPublisher = nodeHandle.advertise<geometry_msgs::PoseArray>("robot/trajectory_poses", 1, false);
	this->trajectoryCheckerPublisher = nodeHandle.advertise<geometry_msgs::PoseArray>("robot/trajectory_checker", 1, false);

	this->leftEESubscriber = nodeHandle.subscribe("robot/left_ee_target", 1, &ROSCommunication::OnLeftEEGoalPoseReceived, this);
	this->rightEESubscriber = nodeHandle.subscribe("robot/right_ee_target", 1, &ROSCommunication::OnRightEEGoalPoseReceived, this);
	this->globalTrajectorySubscriber = nodeHandle.subscribe("robot/global_trajectory", 1, &ROSCommunication::OnTaskTrajectoryReceived, this);
	this->taskSentSubscriber = nodeHandle.subscribe("hololens/send_pressed", 1, &ROSCommunication::OnSendButtonClickedReceived, this);
	this->leftGripperSubscriber = nodeHandle.subscribe("robot/left_gripper_target_width", 1, &ROSCommunication::OnLeftGripperTargetWidthReceived, this);
	this->rightGripperSubscriber = nodeHandle.subscribe("robot/right_gripper_target_width", 1, &ROSCommunication::OnRightGripperTargetWidthReceived, this);
	this->headPoseSubscriber = nodeHandle.subscribe("hololens/head_pose", 1, &ROSCommunication::OnHeadPoseReceived, this);

	// CAMERA
	init_params.depth_mode = depthMode; // Use ULTRA depth mode
	init_params.coordinate_units = sl::UNIT_METER; // Unity uses meters + marker size given in meters
	init_params.camera_resolution = captureResolution;
	init_params.enable_right_side_measure = false;
	init_params.depth_stabilization = true;

	sl::ERROR_CODE zed_error = zed.open(init_params);
	if (zed_error == sl::SUCCESS)
	{
		isZEDConnected = true;

		runtime_params.sensing_mode = sl::SENSING_MODE_STANDARD;
		left_image_zed.alloc(zed.getResolution(), sl::MAT_TYPE_8U_C4);
		left_image_ocv = SlMat2CvMat(left_image_zed);
		leftCameraPublisher = nodeHandle.advertise<sensor_msgs::CompressedImage>("robot/left_camera", 1, true);

		if (rightCameraPublishingEnabled)
		{
			right_image_zed.alloc(zed.getResolution(), sl::MAT_TYPE_8U_C4);
			right_image_ocv = SlMat2CvMat(right_image_zed);
			rightCameraPublisher = nodeHandle.advertise<sensor_msgs::CompressedImage>("robot/right_camera", 1, true);
		}


		zed.disableSpatialMapping();
		zed.disableTracking();

		if (pointCloudPublishingEnabled)
		{
			pointCloudPublisher = nodeHandle.advertise<sensor_msgs::PointCloud2>("robot/point_cloud", 1, false);

			kalmanFilter.Initialize(targetPointCloudLoopDuration);
			kalmanFilter.isReset = true;

			const sl::Resolution cameraResolution = zed.getResolution();
			const sl::Resolution pointCloudResolution(cameraResolution.width / depthImageDownscaleBy,
				cameraResolution.height / depthImageDownscaleBy);
			point_cloud_zed.alloc(pointCloudResolution, sl::MAT_TYPE_32F_C4);
			point_cloud_ocv = SlMat2CvMat(point_cloud_zed);

			calibration_data = zed.getCameraInformation().calibration_parameters;
			cameraMatrix = cv::Mat::zeros(3, 3, CV_64FC1);
			cameraMatrix.at<double>(0, 0) = calibration_data.left_cam.fx;
			cameraMatrix.at<double>(1, 1) = calibration_data.left_cam.fy;
			cameraMatrix.at<double>(0, 2) = calibration_data.left_cam.cx;
			cameraMatrix.at<double>(1, 2) = calibration_data.left_cam.cy;
			cameraMatrix.at<double>(2, 2) = 1.0;
		}
	}
	else
	{
		cout << "ZED could not be connected!" << endl;
	}

	if (this->pSimulationRobot != nullptr &&
		this->pSimulationRobot->rbEngine != nullptr &&
		this->pYumiInterface != nullptr) {
		this->isRosInitialized = true;
	}
	else {
		std::cerr << "Failed to initialize!" << std::endl;
		this->isRosInitialized = false;
	}
}

/**
 *	Starts the threads which publishes and receives all ROS messages.
 */
void ROSCommunication::Start() {
	if (isRosInitialized && !isControlThreadRunning) {
		isControlThreadRunning = true;
		controlThread = std::thread(&ROSCommunication::ControlLoop, this);
		controlThread.detach();
		std::cout << "ROSCommunication control thread started." << std::endl;
		Logger::consolePrint("ROSCommunication control thread started.");
	}

	if (isRosInitialized && !isCameraThreadRunning && (leftCameraPublishingEnabled || rightCameraPublishingEnabled)) {
		isCameraThreadRunning = true;
		cameraThread = std::thread(&ROSCommunication::CameraLoop, this);
		cameraThread.detach();
		std::cout << "ROSCommunication camera thread started." << std::endl;
		Logger::consolePrint("ROSCommunication camera thread started.");
	}

	if (isRosInitialized && !isPointCloudThreadRunning && pointCloudPublishingEnabled) {
		isPointCloudThreadRunning = true;
		pointCloudThread = std::thread(&ROSCommunication::PointCloudLoop, this);
		pointCloudThread.detach();
		std::cout << "ROSCommunication point cloud thread started." << std::endl;
		Logger::consolePrint("ROSCommunication point cloud thread started.");
	}
}

void ROSCommunication::ControlLoop() {
	int seq = 0;
	std::chrono::time_point<std::chrono::high_resolution_clock> frameStartTime;

	while (isControlThreadRunning && ros::ok()) {
		frameStartTime = this->timer.now();

		// Receive any pending incoming messages
		ros::spinOnce();

		// Publish the current poses of the main simulation robot
		geometry_msgs::PoseArray meshPoseMsg = GenerateMeshPoseArrayMsg(*this->pSimulationRobot, seq);
		geometry_msgs::PoseArray globalPoseMsg = GenerateGlobalPoseArrayMsg(*this->pSimulationRobot, seq, true);
		meshPosePublisher.publish(meshPoseMsg);
		globalPosePublisher.publish(globalPoseMsg);

		if (newTrajectoryReceived) {
			geometry_msgs::PoseArray trajectoryMeshPoses;
			geometry_msgs::PoseArray trajectoryEeGlobalPoses;
			GenerateTrajectoryPoseArrays(trajectoryMeshPoses, trajectoryEeGlobalPoses);
			trajectoryMeshPosesPublisher.publish(trajectoryMeshPoses);
			trajectoryCheckerPublisher.publish(trajectoryEeGlobalPoses);
		}

		// Execute the task frame by frame
		if (taskNeedsExecution)
		{
			TryExecutingTask();
		}

		// Execute the camera arm movement frame by frame (if interpolated)
		if (headPoseNeedsExecution)
		{
			if (currentHeadPoseExecutionIndex < headTargetOrientations.size())
			{
				robotHeadTargetOrientation->s = headTargetOrientations[currentHeadPoseExecutionIndex].s;
				robotHeadTargetOrientation->v[0] = headTargetOrientations[currentHeadPoseExecutionIndex].v[0];
				robotHeadTargetOrientation->v[1] = headTargetOrientations[currentHeadPoseExecutionIndex].v[1];
				robotHeadTargetOrientation->v[2] = headTargetOrientations[currentHeadPoseExecutionIndex].v[2];
				currentHeadPoseExecutionIndex++;
			}
			else
			{
				headPoseNeedsExecution = false;
				currentHeadPoseExecutionIndex = 0;
			}
		}

		seq++;
		SleepRemainingDuration(frameStartTime, targetControlLoopDuration);
	}
}

void ROSCommunication::CameraLoop() {
	int seq = 0;
	std::chrono::time_point<std::chrono::high_resolution_clock> frameStartTime;

	while (isCameraThreadRunning && ros::ok()) {
		frameStartTime = this->timer.now();

		if (isZEDConnected && (leftCameraPublishingEnabled || rightCameraPublishingEnabled))
		{
			PublishCameraFrameMsg(seq);
		}
		seq++;
		SleepRemainingDuration(frameStartTime, targetCameraLoopDuration);
	}
}


void ROSCommunication::PointCloudLoop() {
	int seq = 0;
	std::chrono::time_point<std::chrono::high_resolution_clock> frameStartTime;

	while (isPointCloudThreadRunning && ros::ok()) {
		frameStartTime = this->timer.now();

		if (isZEDConnected && pointCloudPublishingEnabled)
		{
			Publish3DPointCloud(seq);
		}
		seq++;
		SleepRemainingDuration(frameStartTime, targetPointCloudLoopDuration);
	}
}



/**
 *	Stops the threads.
 */
void ROSCommunication::Stop() {
	if (isControlThreadRunning) {
		isControlThreadRunning = false;
		controlThread.~thread();
		std::cout << "ROSCommunication control thread terminated." << std::endl;
		Logger::consolePrint("ROSCommunication control thread terminated.");
	}

	if (isCameraThreadRunning) {
		isCameraThreadRunning = false;
		cameraThread.~thread();
		std::cout << "ROSCommunication camera thread terminated." << std::endl;
		Logger::consolePrint("ROSCommunication camera thread terminated.");
	}

	if (isPointCloudThreadRunning) {
		isPointCloudThreadRunning = false;
		pointCloudThread.~thread();
		std::cout << "ROSCommunication point cloud thread terminated." << std::endl;
		Logger::consolePrint("ROSCommunication point cloud thread terminated.");
	}
}

/**
 *	Callback method for received target pose of the left end-effector
 */
void ROSCommunication::OnLeftEEGoalPoseReceived(const geometry_msgs::PoseConstPtr& msg) {
	this->pSimulationRobot->leftGripper.position = P3D(msg->position.x, msg->position.y, msg->position.z);
	Eigen::Quaterniond q = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
	this->pSimulationRobot->leftGripper.rotation = q.normalized().toRotationMatrix() * Eigen::AngleAxisd(PI / 2.0, Eigen::Vector3d::UnitY());
}

/**
 *	Callback method for received target pose of the right end-effector
 */
void ROSCommunication::OnRightEEGoalPoseReceived(const geometry_msgs::PoseConstPtr& msg) {
	this->pSimulationRobot->rightGripper.position = P3D(msg->position.x, msg->position.y, msg->position.z);
	Eigen::Quaterniond q = Eigen::Quaterniond(msg->orientation.w, msg->orientation.x, msg->orientation.y, msg->orientation.z);
	this->pSimulationRobot->rightGripper.rotation = q.normalized().toRotationMatrix() * Eigen::AngleAxisd(PI / 2.0, Eigen::Vector3d::UnitY());
}


/**
  *	Callback method for left gripper target width messages. gripper width is given as percentage, i.e. in value range [0, 1]
  */
void ROSCommunication::OnLeftGripperTargetWidthReceived(const std_msgs::StringConstPtr& msg) {
	this->pYumiInterface->setFingerDistance("left", std::stof(msg->data));
}

/**
  *	Callback method for right gripper target width messages. gripper width is given as percentage, i.e. in value range [0, 1]
  */
void ROSCommunication::OnRightGripperTargetWidthReceived(const std_msgs::StringConstPtr& msg) {
	this->pYumiInterface->setFingerDistance("right", std::stof(msg->data));
}

/**
 *	Callback method for received trajectory created by path drawers
 */
void ROSCommunication::OnTaskTrajectoryReceived(const geometry_msgs::PoseArrayConstPtr& msg) {
	this->trajectoryKeyframes = msg;
	this->newTrajectoryReceived = true;
}

void ROSCommunication::OnSendButtonClickedReceived(const std_msgs::StringConstPtr& msg)
{
	taskNeedsExecution = true;
}

void ROSCommunication::OnHeadPoseReceived(const geometry_msgs::PoseConstPtr& msg) {

	if (!headPoseNeedsExecution)
	{
		headTargetOrientations.clear();
		Quaternion nextHeadTargetOrientation(msg->orientation.w, V3D(msg->orientation.x, msg->orientation.y, msg->orientation.z));

		Quaternion differenceQuaternion = nextHeadTargetOrientation * previousHeadTargetOrientation.getInverse();
		double differenceAngle = abs(atan(differenceQuaternion.v.length() / differenceQuaternion.s) * 180 / PI);

		int differenceAngleInt = static_cast<int>(differenceAngle) + 1;
		for (int i = 0; i <= differenceAngleInt; i++)
		{
			//interpolate orientation
			float ratio = ((float)i) / ((float)differenceAngleInt);
			Quaternion interpolatedQ = previousHeadTargetOrientation.sphericallyInterpolateWith(nextHeadTargetOrientation, ratio);

			headTargetOrientations.push_back(interpolatedQ);
		}

		previousHeadTargetOrientation = nextHeadTargetOrientation;
		headPoseNeedsExecution = true;
	}
}

geometry_msgs::Pose ROSCommunication::ConstructPoseMessage(const P3D& pos, const Eigen::Quaterniond& rot) {
	geometry_msgs::Pose poseMsg;

	poseMsg.position.x = pos.x();
	poseMsg.position.y = pos.y();
	poseMsg.position.z = pos.z();
	poseMsg.orientation.w = rot.w();
	poseMsg.orientation.x = rot.x();
	poseMsg.orientation.y = rot.y();
	poseMsg.orientation.z = rot.z();

	return poseMsg;
}

/**
 *	Creates a new PoseArray message with the current poses of all robot joints.
 */
geometry_msgs::PoseArray ROSCommunication::GenerateGlobalPoseArrayMsg(const TeleoperationRobot& robot, int seq = 0, bool includeElbow = true) {
	geometry_msgs::PoseArray msg;
	msg.header.frame_id = "yumi";
	msg.header.stamp = ros::Time::now();
	msg.header.seq = seq;

	P3D leftEEPosition = robot.rbEngine->rbs[14]->getCMPosition();
	P3D rightEEPosition = robot.rbEngine->rbs[7]->getCMPosition();

	Eigen::Quaterniond leftEERotation(robot.leftGripper.rotation * Eigen::AngleAxisd(-PI / 2.0, Eigen::Vector3d::UnitY()));
	Eigen::Quaterniond rightEERotation(robot.rightGripper.rotation * Eigen::AngleAxisd(-PI / 2.0, Eigen::Vector3d::UnitY()));

	geometry_msgs::Pose leftEEPose = ConstructPoseMessage(leftEEPosition, leftEERotation.normalized());
	geometry_msgs::Pose rightEEPose = ConstructPoseMessage(rightEEPosition, rightEERotation.normalized());

	msg.poses.push_back(leftEEPose);
	msg.poses.push_back(rightEEPose);

	if (includeElbow)
	{
		P3D leftElbowPosition = robot.rbEngine->rbs[11]->getCMPosition();
		P3D rightElbowPosition = robot.rbEngine->rbs[4]->getCMPosition();
		geometry_msgs::Pose leftElbowPose = ConstructPoseMessage(leftElbowPosition, Eigen::Quaterniond::Identity());
		geometry_msgs::Pose rightElbowPose = ConstructPoseMessage(rightElbowPosition, Eigen::Quaterniond::Identity());
		msg.poses.push_back(leftElbowPose);
		msg.poses.push_back(rightElbowPose);
	}

	return msg;
}

/**
 *	Creates a new PoseArray message with the current poses of all robot joints and applied mesh transformations.
 */
geometry_msgs::PoseArray ROSCommunication::GenerateMeshPoseArrayMsg(const TeleoperationRobot& robot, int seq = 0) {
	geometry_msgs::PoseArray msg;
	msg.header.frame_id = "yumi";
	msg.header.stamp = ros::Time::now();
	msg.header.seq = seq;

	Quaternion q;

	for (auto it = robot.rbEngine->rbs.begin(); it != robot.rbEngine->rbs.end(); ++it) {
		geometry_msgs::Pose meshPoseMsg;
		P3D pos = (*it)->getCMPosition();
		Quaternion rot = (*it)->getOrientation();

		// Apply local mesh transformation to global position and orientation
		for (auto mesh_it = (*it)->meshTransformations.begin(); mesh_it != (*it)->meshTransformations.end(); ++mesh_it) {
			q.setRotationFrom((*mesh_it).R);
			P3D meshPos = pos + rot * (*mesh_it).T;
			Quaternion meshRot = rot * q;

			meshPoseMsg.position.x = meshPos.x();
			meshPoseMsg.position.y = meshPos.y();
			meshPoseMsg.position.z = meshPos.z();
			meshPoseMsg.orientation.w = meshRot.s;
			meshPoseMsg.orientation.x = meshRot.v.x();
			meshPoseMsg.orientation.y = meshRot.v.y();
			meshPoseMsg.orientation.z = meshRot.v.z();
			msg.poses.push_back(meshPoseMsg);
		}
	}

	return msg;
}

/**
 *	Creates a new PoseArray message with the optimal poses for each robot joint and for all keyframes in the given message.
 */
void ROSCommunication::GenerateTrajectoryPoseArrays(geometry_msgs::PoseArray& allMeshPoses, geometry_msgs::PoseArray& allEePoses) {

	RobotState currentSimulationRobotState;
	pSimulationRobot->robot->populateState(&currentSimulationRobotState);
	futureRobot.robot->setState(&currentSimulationRobotState);
	futureRobot.ikSolver->ikPlan->setTargetIKStateFromRobot();
	futureRobot.setupYuMiEndEffectors();

	TeleoperationRobot* calculationRobot = &futureRobot;

	int globalKeyframeCount = 0;

	if (this->trajectoryKeyframes != nullptr) {
		allMeshPoses.header = this->trajectoryKeyframes->header;
		allEePoses.header = this->trajectoryKeyframes->header;
		globalKeyframeCount = this->trajectoryKeyframes->poses.size();
	}

	allMeshPoses.header.stamp = ros::Time::now();
	allEePoses.header.stamp = ros::Time::now();
	if (globalKeyframeCount > 0) {

		leftPositionsToPhysicalRobot.clear();
		leftRotationsToPhysicalRobot.clear();
		rightPositionsToPhysicalRobot.clear();
		rightRotationsToPhysicalRobot.clear();

		// Set both arms
		Eigen::Quaterniond q;
		for (int i = 0; i < globalKeyframeCount; i = i + 2) {

			// Get current keyframe poses and set it to end-effector targets
			geometry_msgs::Pose currentLeftKfPose = this->trajectoryKeyframes->poses[i];
			geometry_msgs::Pose currentRightKfPose = this->trajectoryKeyframes->poses[i + 1];

			calculationRobot->leftGripper.position = P3D(currentLeftKfPose.position.x, currentLeftKfPose.position.y, currentLeftKfPose.position.z);
			q = Eigen::Quaterniond(currentLeftKfPose.orientation.w, currentLeftKfPose.orientation.x, currentLeftKfPose.orientation.y, currentLeftKfPose.orientation.z);
			calculationRobot->leftGripper.rotation = q.normalized().toRotationMatrix() * Eigen::AngleAxisd(PI / 2.0, Eigen::Vector3d::UnitY());

			calculationRobot->rightGripper.position = P3D(currentRightKfPose.position.x, currentRightKfPose.position.y, currentRightKfPose.position.z);
			q = Eigen::Quaterniond(currentRightKfPose.orientation.w, currentRightKfPose.orientation.x, currentRightKfPose.orientation.y, currentRightKfPose.orientation.z);
			calculationRobot->rightGripper.rotation = q.normalized().toRotationMatrix() * Eigen::AngleAxisd(PI / 2.0, Eigen::Vector3d::UnitY());

			// Run optimizer
			calculationRobot->updateEndEffectorTargets(this->EE_shift, Eigen::Matrix3d::Identity());
			calculationRobot->ikSolver->solve();

			// Generate a message for calculated reachable poses of the end-effectors
			geometry_msgs::PoseArray currentEeGlobalPose = GenerateGlobalPoseArrayMsg(*calculationRobot, 0, false);
			// Generate a message for current state of the robot mesh
			geometry_msgs::PoseArray currentMeshPose = GenerateMeshPoseArrayMsg(*calculationRobot);

			allEePoses.poses.insert(allEePoses.poses.end(), currentEeGlobalPose.poses.begin(), currentEeGlobalPose.poses.end());
			allMeshPoses.poses.insert(allMeshPoses.poses.end(), currentMeshPose.poses.begin(), currentMeshPose.poses.end());

			V3D currentLeftKfPosition = V3D(currentLeftKfPose.position.x, currentLeftKfPose.position.y, currentLeftKfPose.position.z);
			V3D currentRightKfPosition = V3D(currentRightKfPose.position.x, currentRightKfPose.position.y, currentRightKfPose.position.z);
			Quaternion currentLeftFrameQ = Quaternion(currentLeftKfPose.orientation.w, currentLeftKfPose.orientation.x, currentLeftKfPose.orientation.y, currentLeftKfPose.orientation.z);
			Quaternion currentRightFrameQ = Quaternion(currentRightKfPose.orientation.w, currentRightKfPose.orientation.x, currentRightKfPose.orientation.y, currentRightKfPose.orientation.z);

			// Save end-effector goals for later use for when SEND pressed
			leftPositionsToPhysicalRobot.push_back(currentLeftKfPosition);
			rightPositionsToPhysicalRobot.push_back(currentRightKfPosition);
			leftRotationsToPhysicalRobot.push_back(Eigen::Quaterniond(currentLeftFrameQ.s, currentLeftFrameQ.v.x(), currentLeftFrameQ.v.y(), currentLeftFrameQ.v.z()));
			rightRotationsToPhysicalRobot.push_back(Eigen::Quaterniond(currentRightFrameQ.s, currentRightFrameQ.v.x(), currentRightFrameQ.v.y(), currentRightFrameQ.v.z()));

			// Interpolate between current kf and next kf
			if (i + 3 < globalKeyframeCount)
			{
				geometry_msgs::Pose nextLeftKfPose = this->trajectoryKeyframes->poses[i + 2];
				geometry_msgs::Pose nextRightKfPose = this->trajectoryKeyframes->poses[i + 3];

				V3D nextLeftKfPosition = V3D(nextLeftKfPose.position.x, nextLeftKfPose.position.y, nextLeftKfPose.position.z);
				Quaternion nextLeftFrameQ = Quaternion(nextLeftKfPose.orientation.w, nextLeftKfPose.orientation.x, nextLeftKfPose.orientation.y, nextLeftKfPose.orientation.z);

				V3D nextRightKfPosition = V3D(nextRightKfPose.position.x, nextRightKfPose.position.y, nextRightKfPose.position.z);
				Quaternion nextRightFrameQ = Quaternion(nextRightKfPose.orientation.w, nextRightKfPose.orientation.x, nextRightKfPose.orientation.y, nextRightKfPose.orientation.z);

				V3D leftVector = (nextLeftKfPosition - currentLeftKfPosition) / keyframeInterpolationRate;
				V3D rightVector = (nextRightKfPosition - currentRightKfPosition) / keyframeInterpolationRate;

				for (int j = 1; j < keyframeInterpolationRate; j++)
				{
					// interpolate position
					V3D interpolatedLeftPosition = currentLeftKfPosition + leftVector * j;
					V3D interpolatedRightPosition = currentRightKfPosition + rightVector * j;
					calculationRobot->leftGripper.position = P3D(interpolatedLeftPosition.x(), interpolatedLeftPosition.y(), interpolatedLeftPosition.z());
					calculationRobot->rightGripper.position = P3D(interpolatedRightPosition.x(), interpolatedRightPosition.y(), interpolatedRightPosition.z());

					//interpolate orientation
					float ratio = ((float)j) / ((float)keyframeInterpolationRate);
					Quaternion interpolatedLeftQ = currentLeftFrameQ.sphericallyInterpolateWith(nextLeftFrameQ, ratio);
					Quaternion interpolatedRightQ = currentRightFrameQ.sphericallyInterpolateWith(nextRightFrameQ, ratio);
					Eigen::Quaterniond interpolatedLeftQeigen = Eigen::Quaterniond(interpolatedLeftQ.s, interpolatedLeftQ.v.x(), interpolatedLeftQ.v.y(), interpolatedLeftQ.v.z());
					Eigen::Quaterniond interpolatedRightQeigen = Eigen::Quaterniond(interpolatedRightQ.s, interpolatedRightQ.v.x(), interpolatedRightQ.v.y(), interpolatedRightQ.v.z());
					calculationRobot->leftGripper.rotation = interpolatedLeftQeigen.normalized().toRotationMatrix() * Eigen::AngleAxisd(PI / 2.0, Eigen::Vector3d::UnitY());
					calculationRobot->rightGripper.rotation = interpolatedRightQeigen.normalized().toRotationMatrix() * Eigen::AngleAxisd(PI / 2.0, Eigen::Vector3d::UnitY());

					// Save end-effector goals for later use for when SEND pressed
					leftPositionsToPhysicalRobot.push_back(interpolatedLeftPosition);
					rightPositionsToPhysicalRobot.push_back(interpolatedRightPosition);
					leftRotationsToPhysicalRobot.push_back(interpolatedLeftQeigen);
					rightRotationsToPhysicalRobot.push_back(interpolatedRightQeigen);

					// Run optimizer
					calculationRobot->updateEndEffectorTargets(this->EE_shift, Eigen::Matrix3d::Identity());
					calculationRobot->ikSolver->solve();

					geometry_msgs::PoseArray currentMeshPose = GenerateMeshPoseArrayMsg(*calculationRobot);

					allMeshPoses.poses.insert(allMeshPoses.poses.end(), currentMeshPose.poses.begin(), currentMeshPose.poses.end());
				}
			}
		}

		// Delete global keyframes
		this->trajectoryKeyframes = nullptr;
	}

	this->newTrajectoryReceived = false;
}

void ROSCommunication::TryExecutingTask()
{
	if (currentTaskExecutionIndex < leftPositionsToPhysicalRobot.size())
	{
		pSimulationRobot->leftGripper.position = leftPositionsToPhysicalRobot[currentTaskExecutionIndex];
		pSimulationRobot->leftGripper.rotation = leftRotationsToPhysicalRobot[currentTaskExecutionIndex].normalized().toRotationMatrix() * Eigen::AngleAxisd(PI / 2.0, Eigen::Vector3d::UnitY());
		pSimulationRobot->rightGripper.position = rightPositionsToPhysicalRobot[currentTaskExecutionIndex];
		pSimulationRobot->rightGripper.rotation = rightRotationsToPhysicalRobot[currentTaskExecutionIndex].normalized().toRotationMatrix() * Eigen::AngleAxisd(PI / 2.0, Eigen::Vector3d::UnitY());
		currentTaskExecutionIndex++;
	}
	else
	{
		currentTaskExecutionIndex = 0;
		taskNeedsExecution = false;
	}
}

/**
 *	Grabs the current image from the camera and publishes it.
 */
void ROSCommunication::PublishCameraFrameMsg(const int seq = 0) {
	std_msgs::Header header;
	header.frame_id = "yumi";
	header.stamp = ros::Time::now();
	header.seq = seq;

	if (zed.grab() == sl::SUCCESS) {
		// INFO: left_image_ocv shares the same address space with left_image_zed, same for right
		zed.retrieveImage(left_image_zed, sl::VIEW_LEFT);
		sensor_msgs::CompressedImagePtr msg_left = cv_bridge::CvImage(header, "bgr8", left_image_ocv).toCompressedImageMsg();
		leftCameraPublisher.publish(msg_left);

		if (rightCameraPublishingEnabled)
		{
			zed.retrieveImage(right_image_zed, sl::VIEW_RIGHT);
			sensor_msgs::CompressedImagePtr msg_right = cv_bridge::CvImage(header, "bgr8", right_image_ocv).toCompressedImageMsg();
			rightCameraPublisher.publish(msg_right);
		}
	}
}

void ROSCommunication::Publish3DPointCloud(int seq)
{
	if (zed.grab(runtime_params) == sl::SUCCESS) {
		zed.retrieveImage(left_image_zed, sl::VIEW_LEFT);

		cv::Mat gray_image;
		cv::cvtColor(left_image_ocv, gray_image, cv::COLOR_BGR2GRAY);
		std::vector<int> all_ids, correct_ids;
		std::vector<std::vector<cv::Point2f>> all_corners, correct_corners;
#ifdef MR_ARUCO_ENABLED	
		cv::aruco::detectMarkers(gray_image, dictionary, all_corners, all_ids);
#endif
		// Check if the correct ArUco marker is detected
		int index = -1;
		for (int i = 0; i < all_ids.size(); i++)
		{
			const int id = all_ids[i];
			if (id == 1)
			{
				index = i;
			}
		}

		// Found, operate on it
		if (index != -1)
		{
			std::vector<std::vector<cv::Point2f>> correct_corners;
			correct_corners.push_back(all_corners[index]);

			// Estimate the pose of the marker - left_camera
			std::vector<cv::Vec3d> rvecs, tvecs;
			const vector<double> distCoeffs(calibration_data.left_cam.disto, calibration_data.left_cam.disto +
				sizeof calibration_data.left_cam.disto / sizeof calibration_data.left_cam.disto[0]);
#ifdef MR_ARUCO_ENABLED
			cv::aruco::estimatePoseSingleMarkers(correct_corners, markerSideLength, cameraMatrix, distCoeffs, rvecs, tvecs);
#endif
			kalmanFilter.UpdateKalmanFilter(tvecs[0], rvecs[0]);
			cv::Vec3d tvec = kalmanFilter.GetEstimatedTranslation();
			cv::Vec3d rvec = kalmanFilter.GetEstimatedRotation();

			// Convert rvec and tvec into pose 4x4 pose matrix
			cv::Mat homogeneousRow = cv::Mat::zeros(1, 4, CV_64FC1);
			homogeneousRow.at<double>(0, 3) = 1.0;
			cv::Mat rotMat(3, 3, CV_64FC1);
			cv::Mat tempMat(3, 4, CV_64FC1);
			cv::Mat poseMat(4, 4, CV_64FC1);
			cv::Rodrigues(rvec, rotMat);
			cv::hconcat(rotMat, tvec, tempMat);
			cv::vconcat(tempMat, homogeneousRow, poseMat);

			// Apply coordinate transformation
			cv::Mat poseMatInv = poseMat.inv();
			poseMatInv.convertTo(poseMatInv, CV_32FC1);

			// Get the point cloud from zed
			int width = point_cloud_zed.getResolution().width;
			int height = point_cloud_zed.getResolution().height;
			vector<cv::Vec3f> validCoords;
			vector<cv::Vec3b> validColors;
			if (zed.retrieveMeasure(point_cloud_zed, sl::MEASURE_XYZRGBA, sl::MEM_CPU, width, height) == sl::SUCCESS)
			{
				for (int i = 0; i < point_cloud_zed.getResolution().height; i++)
				{
					for (int j = depthImageCropAmount; j < point_cloud_zed.getResolution().width - depthImageCropAmount; j++)
					{
						cv::Vec4f pixel = point_cloud_ocv.at<cv::Vec4f>(i, j);
						float x = pixel[0];
						float y = pixel[1];
						float z = pixel[2];
						float color = pixel[3];

						uint8_t color_bytes[sizeof(float)];
						*reinterpret_cast<float*>(color_bytes) = color;

						cv::Vec4f xyz1(x, y, z, 1.0f);

						cv::Mat position = poseMatInv * cv::Mat(xyz1);
						x = position.at<float>(0, 0);
						y = position.at<float>(1, 0);
						z = position.at<float>(2, 0);

						if (z > 0.01 && x > -0.3 && x < 0.3 && y > -0.57 && y < 0.03)
						{
							validCoords.push_back(cv::Vec3f(x, y, z));
							validColors.push_back(cv::Vec3b(color_bytes[0], color_bytes[1], color_bytes[2]));
						}
					}
				}

				// Place point cloud in a PointCloud2 message
				cv::Mat coords(validCoords.size(), 3, CV_32FC1, validCoords.data());
				cv::Mat colors(validColors.size(), 3, CV_8UC1, validColors.data());

				sensor_msgs::PointCloud2 message = CreatePointCloud2Message(coords, colors);
				pointCloudPublisher.publish(message);
			}
		}
		else
		{
			if (!kalmanFilter.isReset)
			{
				cout << "Marker not detected, initializing Kalman Filter" << endl;
				kalmanFilter.Initialize(targetPointCloudLoopDuration / 1000.0f);
				kalmanFilter.isReset = true;
			}

			sensor_msgs::PointCloud2 cloud;
			cloud.header.frame_id = "marker";
			cloud.header.stamp = ros::Time::now();
			cloud.width = 0;
			cloud.height = 1;
			cloud.is_bigendian = false;
			cloud.is_dense = false; // there may be invalid points
			pointCloudPublisher.publish(cloud);
		}
	}
}

/**
 *	Sleeps the remaining frame duration in order to meet the FPS limit.
 */
void ROSCommunication::SleepRemainingDuration(const std::chrono::time_point<std::chrono::high_resolution_clock>& frameStartTime, const double& desiredDuration) const
{
	auto frameEndTime = this->timer.now();

	std::chrono::duration<double, std::milli> frameDuration = frameEndTime - frameStartTime;
	std::chrono::duration<double, std::milli> delta(desiredDuration - frameDuration.count());
	auto deltaDuration = std::chrono::duration_cast<std::chrono::milliseconds>(delta);
	if (delta.count() > 0) std::this_thread::sleep_for(std::chrono::milliseconds(deltaDuration.count()));
}

cv::Mat SlMat2CvMat(sl::Mat& input) {
	int cv_type = -1;
	switch (input.getDataType()) {
	case sl::MAT_TYPE_32F_C1: cv_type = CV_32FC1; break;
	case sl::MAT_TYPE_32F_C2: cv_type = CV_32FC2; break;
	case sl::MAT_TYPE_32F_C3: cv_type = CV_32FC3; break;
	case sl::MAT_TYPE_32F_C4: cv_type = CV_32FC4; break;
	case sl::MAT_TYPE_8U_C1: cv_type = CV_8UC1; break;
	case sl::MAT_TYPE_8U_C2: cv_type = CV_8UC2; break;
	case sl::MAT_TYPE_8U_C3: cv_type = CV_8UC3; break;
	case sl::MAT_TYPE_8U_C4: cv_type = CV_8UC4; break;
	default: break;
	}
	// Since cv::Mat data requires a uchar* pointer, we get the uchar1 pointer from sl::Mat (getPtr<T>())
	// cv::Mat and sl::Mat will share a single memory structure
	return cv::Mat(input.getHeight(), input.getWidth(), cv_type, input.getPtr<sl::uchar1>(sl::MEM_CPU));
}

sensor_msgs::PointField constructPointField(string name, uint32_t offset, uint8_t datatype, uint32_t count)
{
	sensor_msgs::PointField point_field;

	point_field.name = name;
	point_field.offset = offset;
	point_field.datatype = datatype;
	point_field.count = count;

	return point_field;
}

//convert point cloud image to ros message
sensor_msgs::PointCloud2 CreatePointCloud2Message(const cv::Mat& coords, const cv::Mat& colors)
{
	// coords is a numpoints x 3 matrix
	// colors is a numpoints x 3 matrix

	//figure out number of points
	int numpoints = coords.rows;

	//declare message and sizes
	sensor_msgs::PointCloud2 cloud;
	cloud.header.frame_id = "marker";
	cloud.header.stamp = ros::Time::now();
	cloud.width = numpoints;
	cloud.height = 1;
	cloud.is_bigendian = false;
	cloud.is_dense = false; // there may be invalid points

	//for fields setup
	sensor_msgs::PointCloud2Modifier modifier(cloud);
	modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
	modifier.resize(numpoints);

	//iterators
	sensor_msgs::PointCloud2Iterator<float> out_x(cloud, "x");
	sensor_msgs::PointCloud2Iterator<float> out_y(cloud, "y");
	sensor_msgs::PointCloud2Iterator<float> out_z(cloud, "z");
	sensor_msgs::PointCloud2Iterator<uint8_t> out_rgb(cloud, "rgb");

	for (int i = 0; i < coords.rows; i++)
	{
		//get the image coordinate for this point
		cv::Vec3f coord = coords.row(i);
		float X_World = coord.val[0];
		float Y_World = coord.val[1];
		float Z_World = coord.val[2];

		//get the colors
		cv::Vec3b color = colors.row(i);
		uint8_t r = (color[0]);
		uint8_t g = (color[1]);
		uint8_t b = (color[2]);

		//store xyz in point cloud
		//*out_x = Z_World;
		//*out_y = -X_World;
		//*out_z = -Y_World;
		*out_x = X_World;
		*out_y = Y_World;
		*out_z = Z_World;

		// store colors
		out_rgb[0] = r;
		out_rgb[1] = g;
		out_rgb[2] = b;

		//increment
		++out_x;
		++out_y;
		++out_z;
		++out_rgb;
	}
	return cloud;
}
