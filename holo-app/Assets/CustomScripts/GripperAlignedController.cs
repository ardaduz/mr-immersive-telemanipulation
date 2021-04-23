using System;
using System.Collections;
using System.Collections.Generic;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;
using UnityEngine;
using RosSharp.RosBridgeClient;

public class GripperAlignedController : Publisher<RosSharp.RosBridgeClient.Messages.Geometry.Pose>
{
    public GameObject robotBase;
    public double GripperOffset;
    public HandTracker handTracker;

    private RosSharp.RosBridgeClient.Messages.Geometry.Pose message;

    protected override void Start()
    {
        base.Start();
       
        message = new RosSharp.RosBridgeClient.Messages.Geometry.Pose();
    }

    void Update()
    {
        if (handTracker.HandsDetected)
        {
            AlignToFingerTips();
            UpdateMessage();
            Publish(message);
        }
    }

    private void AlignToFingerTips()
    {
        MixedRealityPose thumbTip = handTracker.ThumbTip;
        MixedRealityPose indexTip = handTracker.IndexTip;
        MixedRealityPose indexKnuckle = handTracker.IndexKnuckle;

        Vector3 A = thumbTip.Position;
        Vector3 B = indexTip.Position;
        Vector3 P = indexKnuckle.Position;

        // Project indexKnuckle onto line between thumbTip and IndexTip
        Vector3 AP = P - A;
        Vector3 AB = B - A;
        Vector3 Phat = A + Vector3.Dot(AP, AB) / Vector3.Dot(AB, AB) * AB;
        Vector3 forwardDirection = (P - Phat).normalized;
        Vector3 upwardDirection;
        if (handTracker.handedness == Handedness.Left)
        {
            upwardDirection = Vector3.Cross(forwardDirection, AB);
        } else
        {
            upwardDirection = Vector3.Cross(AB, forwardDirection);
        }
        Vector3 AB_center = 0.5f * (A + B);
        Vector3 gripperPos = (float)GripperOffset * forwardDirection + AB_center;

        transform.position = gripperPos;
        transform.rotation = Quaternion.LookRotation(forwardDirection, upwardDirection);
    }

    private void UpdateMessage()
    {
        Matrix4x4 localTargetPose = Matrix4x4.TRS(transform.localPosition, transform.localRotation, transform.localScale);
        Matrix4x4 originBasePose = Matrix4x4.TRS(robotBase.transform.localPosition, robotBase.transform.localRotation, robotBase.transform.localScale);
        Matrix4x4 transformedEndGoalPose = originBasePose * localTargetPose;

        message.position = Utils.GetGeometryPoint(Utils.PositionFromMatrix(transformedEndGoalPose));
        message.orientation = Utils.GetGeometryQuaternion(Utils.QuaternionFromMatrix(transformedEndGoalPose));
    }
}
