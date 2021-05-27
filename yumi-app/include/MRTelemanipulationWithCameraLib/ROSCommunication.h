#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/PointField.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <std_msgs/String.h>
#include <MRTelemanipulationWithCameraLib/TeleoperationRobot.h>
#include <MRTelemanipulationWithCameraLib/KalmanFilter.h>
#include <YuMiLib/YuMiControlInterface.h>
#include <thread>
#include <chrono>
#include <vector>
#include <opencv2/opencv.hpp>
#include <sl_zed/Camera.hpp>
#include <sl/Core.hpp>

// if you don't have opencv/aruco module, you can still run by commenting out this preprocessor statement
// #define MR_ARUCO_ENABLED
#ifdef MR_ARUCO_ENABLED
#include <opencv2/aruco.hpp>
#endif

class ROSCommunication {
private:
	// SETTINGS //
	double EE_shift = 0.05; // you may keep it as it is

	sl::DEPTH_MODE depthMode = sl::DEPTH_MODE_QUALITY;
	sl::RESOLUTION captureResolution = sl::RESOLUTION_HD720; // can't run higher (causes ROS bridge to crash) without further optimizations in the code !!!
	const bool leftCameraPublishingEnabled = true; // set this true if you want to enable publishing the left camera images
	const bool rightCameraPublishingEnabled = false; // set this true if you want to enable publishing the right camera image
	const double targetControlLoopDuration = 1000 / (double)30; // target duration per frame in ms (e.g. 1000 / 60 = 60 Hz target frame rate)
	const double targetPointCloudLoopDuration = 1000 / (double)1; // target duration per frame in ms (e.g. 1000 / 60 = 60 Hz target frame rate)
	const double targetCameraLoopDuration = 1000 / (double)30; // target duration per frame in ms (e.g. 1000 / 60 = 60 Hz target frame rate)
	const int depthImageDownscaleBy = 4; // increase performance by downscaling the depth map calculation
	const int depthImageCropAmount = 25; // crop the blind spots of the stereo
	
#ifdef MR_ARUCO_ENABLED
	const bool pointCloudPublishingEnabled = true; // set this true if you want to enable point cloud publishing
#else
	const bool pointCloudPublishingEnabled = false; // don't touch this, it is here because of the preprocessor statement
#endif

	// ROS INTERFACE //
	ros::NodeHandle nodeHandle;
	bool isRosInitialized = false;
	std::chrono::high_resolution_clock timer;
	ros::Publisher meshPosePublisher; // sends the current mesh pose (global + local) of the whole robot
	ros::Publisher globalPosePublisher; // sends the current global pose of the end-effectors and elbows
	ros::Publisher leftCameraPublisher; // sends the left camera images of ZED
	ros::Publisher rightCameraPublisher; // sends the right camera images of ZED
	ros::Publisher trajectoryMeshPosesPublisher; // sends the mesh poses of whole robot of the whole trajectory when a task is received
	ros::Publisher trajectoryCheckerPublisher; // sends the global poses of end-effectors of the whole trajectory when a task is received
	ros::Publisher pointCloudPublisher;
	ros::Subscriber leftEESubscriber; // receives left end-effector goal pose
	ros::Subscriber rightEESubscriber; // receives right end-effector goal pose
	ros::Subscriber globalTrajectorySubscriber; // receives the task
	ros::Subscriber taskSentSubscriber; // receives if send button clicked on the HoloLens (task will be executed by the simulation robot)
	ros::Subscriber headPoseSubscriber; // receives the head pose from the HoloLens
	ros::Subscriber leftGripperSubscriber; // receives the left gripper goal width (opening/closing the grippers)
	ros::Subscriber rightGripperSubscriber; // receives the right gripper goal width (opening/closing the grippers)111

	// THREADING //
	std::thread controlThread; // the thread that handles controlling the robot
	std::thread cameraThread; // the thread that handles camera image acquisition and sending
	std::thread pointCloudThread; // the thread that handles point cloud calculation and sending
	bool isControlThreadRunning = false;
	bool isCameraThreadRunning = false;
	bool isPointCloudThreadRunning = false;
	void SleepRemainingDuration(const std::chrono::time_point<std::chrono::high_resolution_clock>&, const double&) const;
	
	// ROBOT CONTROL //
	TeleoperationRobot* pSimulationRobot = nullptr; // main simulation robot, which can be connected to the physical robot
	YuMiControlInterface* pYumiInterface = nullptr; // utilized to control the grippers of the simulation robot
	Quaternion* robotHeadTargetOrientation = nullptr; // utilized to control the camera arm of the simulation robot
	Quaternion previousHeadTargetOrientation;
	vector<Quaternion> headTargetOrientations;
	atomic_bool headPoseNeedsExecution = false;
	int currentHeadPoseExecutionIndex = 0;
	
	bool newTrajectoryReceived = false; // This and below are the used for calculation and execution of a received task trajectory
	geometry_msgs::PoseArrayConstPtr trajectoryKeyframes;
	TeleoperationRobot futureRobot;
	int keyframeInterpolationRate = 5;
	bool taskNeedsExecution = false;
	int currentTaskExecutionIndex = 0;
	vector<P3D> leftPositionsToPhysicalRobot;
	vector<P3D> rightPositionsToPhysicalRobot;
	vector<Eigen::Quaterniond> leftRotationsToPhysicalRobot;
	vector<Eigen::Quaterniond> rightRotationsToPhysicalRobot;
	
	void ControlLoop(); // Main loop of the control thread and below are functions utilized during a loop
	void OnLeftEEGoalPoseReceived(const geometry_msgs::PoseConstPtr&);
	void OnRightEEGoalPoseReceived(const geometry_msgs::PoseConstPtr&);
	void OnLeftGripperTargetWidthReceived(const std_msgs::StringConstPtr&);
	void OnRightGripperTargetWidthReceived(const std_msgs::StringConstPtr&);
	void OnHeadPoseReceived(const geometry_msgs::PoseConstPtr&);
	void OnTaskTrajectoryReceived(const geometry_msgs::PoseArrayConstPtr&);
	void OnSendButtonClickedReceived(const std_msgs::StringConstPtr&);
	static geometry_msgs::Pose ConstructPoseMessage(const P3D&, const Eigen::Quaterniond&);
	geometry_msgs::PoseArray GenerateGlobalPoseArrayMsg(const TeleoperationRobot&, int, bool);
	geometry_msgs::PoseArray GenerateMeshPoseArrayMsg(const TeleoperationRobot&, int);
	void GenerateTrajectoryPoseArrays(geometry_msgs::PoseArray&, geometry_msgs::PoseArray&); // Function that generates all the MESH poses and end-effector GLOBAL poses for a received TRAJECTORY
	void TryExecutingTask();
	
	// CAMERA and POINT CLOUD//
	bool isZEDConnected = false;
	sl::Camera zed;
	sl::InitParameters init_params;
	sl::RuntimeParameters runtime_params;
	sl::CalibrationParameters calibration_data;
	cv::Mat cameraMatrix;
	KalmanFilter kalmanFilter;
	sl::Mat left_image_zed, right_image_zed, point_cloud_zed;
	cv::Mat left_image_ocv, right_image_ocv, point_cloud_ocv;
#ifdef MR_ARUCO_ENABLED
	cv::Ptr<cv::aruco::Dictionary> dictionary = getPredefinedDictionary(cv::aruco::PREDEFINED_DICTIONARY_NAME(4));
#endif
	double markerSideLength = 0.0755;
	void CameraLoop();
	void PointCloudLoop();
	void Publish3DPointCloud(int);
	void PublishCameraFrameMsg(int);

public:
	ROSCommunication();
	~ROSCommunication();

	void Init(TeleoperationRobot*, YuMiControlInterface*, Quaternion*, double);

	void Start();
	void Stop();
};

cv::Mat SlMat2CvMat(sl::Mat&);
sensor_msgs::PointField constructPointField(string name, uint32_t offset, uint8_t datatype, uint32_t count);
sensor_msgs::PointCloud2 CreatePointCloud2Message(const cv::Mat& coords, const cv::Mat& colors);
