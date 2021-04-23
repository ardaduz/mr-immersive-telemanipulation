using System.Collections;
using System;
using System.Collections.Generic;
using System.Data;
using RosSharp.RosBridgeClient;
using UnityEngine;
using Microsoft.MixedReality.Toolkit.UI;

public class GlobalTaskController : Publisher<RosSharp.RosBridgeClient.Messages.Geometry.PoseArray>
{
    [Header("Object References")]

    [SerializeField]
    [Tooltip("PathDrawer implementing TaskController for the left Arm.")]
    public TaskController leftArmTaskController;

    [SerializeField]
    [Tooltip("PathDrawer implementing TaskController for the right Arm.")]
    public TaskController rightArmTaskController;

    public GameObject previewRobotBase;
    private GameObject currentManipulatedGameObject;

    private RosSharp.RosBridgeClient.Messages.Geometry.PoseArray message;
    private RosSharp.RosBridgeClient.Messages.Geometry.Pose[] poses;
    private int[] keyframeIndices;

    protected override void Start()
    {
        base.Start();
        message = new RosSharp.RosBridgeClient.Messages.Geometry.PoseArray();
        OnResetPressed();
    }

    void OnDisable()
    {
        OnResetPressed();
    }

    private void UpdateMessage()
    {
        message.header.Update();
        message.poses = GeneratePoseArray();
    }

    // below is the function for the callback button for sending the path drawn to the robot
    public void OnSendPressed()
    {
        if(leftArmTaskController.GetKeyframeList().Count > 1 || rightArmTaskController.GetKeyframeList().Count > 1)
        {
            SendTaskNotifyPublisher sendTaskNotifyPublisher = GetComponent<SendTaskNotifyPublisher>();
            sendTaskNotifyPublisher.SendTaskNotify();

            RobotTrajectorySubscriber robotTrajectorySubscriber = GetComponent<RobotTrajectorySubscriber>();
            robotTrajectorySubscriber.OnResetPressed();

            leftArmTaskController.OnSendPressed();
            rightArmTaskController.OnSendPressed();
        }
    }

    public void OnPreviewPressed()
    {
        Debug.Log("GlobalTaskController.OnPreviewPressed() called");
        TrajectoryCheckerSubscriber trajectoryCheckerSubscriber = this.GetComponentInChildren<TrajectoryCheckerSubscriber>();
        if (trajectoryCheckerSubscriber != null)
        {
            trajectoryCheckerSubscriber.TrajectoryNeedsServerUpdate();
        }
        leftArmTaskController.SnapToLastKeyframe();
        rightArmTaskController.SnapToLastKeyframe();
        Debug.Log("Preview pressed");
        UpdateMessage();
        Publish(message);

    }

    // below is the function for the callback button for resetting the path drawn
    public void OnResetPressed()
    {
        rightArmTaskController.OnResetPressed();
        leftArmTaskController.OnResetPressed();
    }

    private RosSharp.RosBridgeClient.Messages.Geometry.Pose[] GeneratePoseArray()
    {
        Debug.Log("GeneratePoseArray Begin");

        List<GameObject> rightArmKeyframeList = rightArmTaskController.GetKeyframeList();
        List<GameObject> leftArmKeyframeList = leftArmTaskController.GetKeyframeList();

        List<DateTime> rightArmKeyframeTimestampList = rightArmTaskController.GetKeyframeTimestampList();
        List<DateTime> leftArmKeyframeTimestampList = leftArmTaskController.GetKeyframeTimestampList();

        int rightCount = rightArmKeyframeList.Count;
        int leftCount = leftArmKeyframeList.Count;
        int keyframeCount = rightCount + leftCount;

        Debug.Log("Keyframelist Right Count: " + rightCount + " Left Count: " + leftCount);
        Debug.Log("KeyframeTimelist Right Count: " + rightArmKeyframeTimestampList.Count + " Left Count: " + leftArmKeyframeTimestampList.Count);

        RosSharp.RosBridgeClient.Messages.Geometry.Pose[] tempPoses = new RosSharp.RosBridgeClient.Messages.Geometry.Pose[2 * keyframeCount];
        int[] tempKeyframeIndices = new int[2 * keyframeCount];

        // START MERGING TWO KEYFRAME LISTS
        int currentRightArmKeyframeIndex = 0;
        int currentLeftArmKeyframeIndex = 0;

        bool isLeftEnded = false;
        bool isRightEnded = false;
        int currentIndex = 0;
        while (!isLeftEnded || !isRightEnded)
        {
            DateTime currentRightKeyframeTimestamp;
            DateTime currentLeftKeyframeTimestamp;

            currentRightKeyframeTimestamp = !isRightEnded
                ? rightArmKeyframeTimestampList[currentRightArmKeyframeIndex]
                : DateTime.UtcNow;
            currentLeftKeyframeTimestamp =
                !isLeftEnded ? leftArmKeyframeTimestampList[currentLeftArmKeyframeIndex]
                : DateTime.UtcNow;

            tempPoses[2 * currentIndex] = leftArmTaskController.GeneratePoseMessage(leftArmKeyframeList[currentLeftArmKeyframeIndex]);
            tempPoses[2 * currentIndex + 1] = rightArmTaskController.GeneratePoseMessage(rightArmKeyframeList[currentRightArmKeyframeIndex]);

            tempKeyframeIndices[2 * currentIndex] = currentLeftArmKeyframeIndex;
            tempKeyframeIndices[2 * currentIndex + 1] = currentRightArmKeyframeIndex;

            int timestampCompareResult = DateTime.Compare(currentLeftKeyframeTimestamp, currentRightKeyframeTimestamp);
            if (timestampCompareResult < 0)
            {
                //Left keyframe is earlier, it is the one should be moved further
                if (currentLeftArmKeyframeIndex < leftCount - 1)
                    currentLeftArmKeyframeIndex++;
                else
                    isLeftEnded = true;
            }
            else
            {
                //Right keyframe is earlier or equal, it is the one should be moved further
                if (currentRightArmKeyframeIndex < rightCount - 1)
                    currentRightArmKeyframeIndex++;
                else
                    isRightEnded = true;
            }

            currentIndex++;
        }

        Debug.Log("GeneratePoseArray End leftCount: " + leftCount + " rightCount: " + rightCount + " curretLeftArmKeyframeIndex: " + currentLeftArmKeyframeIndex + " currentRightArmKeyframeIndex: " + currentRightArmKeyframeIndex);

        poses = tempPoses;
        keyframeIndices = tempKeyframeIndices;

        return tempPoses;
    }

    public RosSharp.RosBridgeClient.Messages.Geometry.Pose[] GetLastPoseArray()
    {
        return poses;
    }

    public int[] GetKeyframeIndices()
    {
        return keyframeIndices;
    }

    public void OnOpenGripperActivated()
    {
        if (currentManipulatedGameObject)
        {
            //previewRobotBase.GetComponentInChildren<RobotTrajectorySubscriber>().OpenGripperBy(currentManipulatedGameObject, -1);
            // TODO: store gripper width somewhere and send it to the simulation with the other stuff.
        }
    }

    public void OnOpenGripper1Activated()
    {
        if (currentManipulatedGameObject)
        {
            //previewRobotBase.GetComponentInChildren<RobotTrajectorySubscriber>().OpenGripperBy(currentManipulatedGameObject, 1);
            // TODO: store gripper width somewhere and send it to the simulation with the other stuff.
        }
    }

    public void OnOpenGripper5Activated()
    {
        if (currentManipulatedGameObject)
        {
            //previewRobotBase.GetComponentInChildren<RobotTrajectorySubscriber>().OpenGripperBy(currentManipulatedGameObject, 5);
            // TODO: store gripper width somewhere and send it to the simulation with the other stuff.
        }
    }

    public void OnOpenGripper10Activated()
    {
        if (currentManipulatedGameObject)
        {
            //previewRobotBase.GetComponentInChildren<RobotTrajectorySubscriber>().OpenGripperBy(currentManipulatedGameObject, 10);
            // TODO: store gripper width somewhere and send it to the simulation with the other stuff.
        }
    }

    public void OnCloseGripperActivated()
    {
        if (currentManipulatedGameObject)
        {
            //previewRobotBase.GetComponentInChildren<RobotTrajectorySubscriber>().CloseGripperBy(currentManipulatedGameObject, -1);
            // TODO: store gripper width somewhere and send it to the simulation with the other stuff.
        }
    }

    public void OnCloseGripper1Activated()
    {
        if (currentManipulatedGameObject)
        {
            //previewRobotBase.GetComponentInChildren<RobotTrajectorySubscriber>().CloseGripperBy(currentManipulatedGameObject, 1);
            // TODO: store gripper width somewhere and send it to the simulation with the other stuff.
        }
    }

    public void OnCloseGripper5Activated()
    {
        if (currentManipulatedGameObject)
        {
            //previewRobotBase.GetComponentInChildren<RobotTrajectorySubscriber>().CloseGripperBy(currentManipulatedGameObject, 5);
            // TODO: store gripper width somewhere and send it to the simulation with the other stuff.
        }
    }

    public void OnCloseGripper10Activated()
    {
        if (currentManipulatedGameObject)
        {
            //previewRobotBase.GetComponentInChildren<RobotTrajectorySubscriber>().CloseGripperBy(currentManipulatedGameObject, 10);
            // TODO: store gripper width somewhere and send it to the simulation with the other stuff.
        }
    }

    public void OnPathDrawerManipulationStarted(ManipulationEventData manipulationEventData)
    {
        ManipulationHandler manipulationHandler = manipulationEventData.ManipulationSource;
        Debug.Log("OnPathDrawerManipulationStarted" + manipulationHandler);
        if (manipulationHandler)
        {
            currentManipulatedGameObject = manipulationHandler.gameObject;
        }
    }

    public void OnPathDrawerManipulationEnded(ManipulationEventData manipulationEventData)
    {
        Debug.Log("OnPathDrawerManipulationEnded");
        currentManipulatedGameObject = null;
        OnPreviewPressed();
    }

    public void OnKeyframeManipulationStarted(ManipulationEventData manipulationEventData)
    {
        ManipulationHandler manipulationHandler = manipulationEventData.ManipulationSource;
        Debug.Log("OnKeyframeManipulationStarted" + manipulationHandler);
        if (manipulationHandler)
        {
            currentManipulatedGameObject = manipulationHandler.gameObject;
        }
    }

    public void OnKeyframeManipulationEnded(ManipulationEventData manipulationEventData)
    {
        Debug.Log("OnKeyframeManipulationEnded");
        currentManipulatedGameObject = null;
        OnPreviewPressed();
    }

}
