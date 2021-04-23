using System;
using System.Collections;
using System.Collections.Generic;
using Microsoft.MixedReality.Toolkit.Examples.Demos.EyeTracking;
using UnityEngine;
using RosSharp.RosBridgeClient;

public class EffectorTargetController : Publisher<RosSharp.RosBridgeClient.Messages.Geometry.Pose>
{
    public GameObject robotBase;

    private RosSharp.RosBridgeClient.Messages.Geometry.Pose message;
    private bool isBeingManipulated;

    protected override void Start()
    {
        base.Start();
        message = new RosSharp.RosBridgeClient.Messages.Geometry.Pose();
    }

    void Update()
    {
        if (isBeingManipulated)
        {
            UpdateMessage();
        }
        else
        {
            SnapToCurrentGlobalPosition();
        }
    }

    private void UpdateMessage()
    {
        Matrix4x4 localTargetPose = Matrix4x4.TRS(transform.localPosition, transform.localRotation, transform.localScale);
        Matrix4x4 originBasePose = Matrix4x4.TRS(robotBase.transform.localPosition, robotBase.transform.localRotation, robotBase.transform.localScale);
        Matrix4x4 transformedEndGoalPose = originBasePose * localTargetPose;

        message.position = Utils.GetGeometryPoint(Utils.PositionFromMatrix(transformedEndGoalPose));
        message.orientation = Utils.GetGeometryQuaternion(Utils.QuaternionFromMatrix(transformedEndGoalPose));

        Publish(message);
    }

    // below are the callbacks and functions for signalling manipulation to the communication interface to activate sending
    public void OnManipulationStarted()
    {
        isBeingManipulated = true;
    }

    public void OnManipulationEnded()
    {
        isBeingManipulated = false;
    }

    public bool GetManipulationStatus()
    {
        return isBeingManipulated;
    }

    private void SnapToCurrentGlobalPosition()
    {
        if (!RobotGlobalPoseSubscriber.hasReceivedFirstMessage) return;

        try
        {
            Vector3 EEGlobalPosition;
            Quaternion EEGlobalRotation;
            if (gameObject.name.ToLower().Contains("left"))
            {
                EEGlobalPosition = RobotGlobalPoseSubscriber.LeftEEPositionFromServer;
                EEGlobalRotation = RobotGlobalPoseSubscriber.LeftEERotationFromServer;
            }
            else
            {
                EEGlobalPosition = RobotGlobalPoseSubscriber.RightEEPositionFromServer;
                EEGlobalRotation = RobotGlobalPoseSubscriber.RightEERotationFromServer;
            }

            // We know that: Matrix4x4 transformedEndGoalPose = originPose * localBallPose, if so following should give the local ball pose:
            // originPose.inverse * transformedEndGoalPose = localBallPose

            Matrix4x4 originBasePose = Matrix4x4.TRS(robotBase.transform.localPosition, robotBase.transform.localRotation, robotBase.transform.localScale);
            Matrix4x4 localBallPose = originBasePose.inverse * Matrix4x4.TRS(EEGlobalPosition, EEGlobalRotation, Vector3.one);
            transform.localPosition = Utils.PositionFromMatrix(localBallPose);
            transform.localRotation = Utils.QuaternionFromMatrix(localBallPose);
            transform.localRotation.Normalize();
        }
        catch
        {
            return;
        }
    }
}
