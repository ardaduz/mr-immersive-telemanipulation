using System;
using System.Collections;
using System.Collections.Generic;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;
using UnityEngine;
using RosSharp.RosBridgeClient;

public class GripperWidthController : Publisher<RosSharp.RosBridgeClient.Messages.Standard.String>
{
    public string endEffectorSide;
    public HandTracker leftHandTracker;
    public HandTracker rightHandTracker;
    public double minWidth = 0.01;
    public double maxWidth = 0.05;

    public UIController uiController;

    private EffectorTargetController effectorTargetController;
    private RosSharp.RosBridgeClient.Messages.Standard.String message;
    private HandTracker internalModeHandTracker;

    protected override void Start()
    {
        base.Start();
        message = new RosSharp.RosBridgeClient.Messages.Standard.String("");

        effectorTargetController = GetComponentInParent<EffectorTargetController>();

        if (endEffectorSide.ToLower().Equals("left"))
        {
            internalModeHandTracker = leftHandTracker;
        }
        else
        {
            internalModeHandTracker = rightHandTracker;
        }
    }

    void Update()
    {
        if (uiController.GetCurrentMode() == UIController.Mode.EXT)
        {
            if (effectorTargetController.GetManipulationStatus() == false)
            {
                if (leftHandTracker.HandsDetected)
                {
                    Vector3 fingerAveragePosition = (leftHandTracker.ThumbTip.Position + leftHandTracker.IndexTip.Position) / 2.0f;
                    float distance = (transform.position - fingerAveragePosition).magnitude;
                    if (distance < 0.03)
                    {
                        UpdateMessage(leftHandTracker.ThumbTip.Position, leftHandTracker.IndexTip.Position);
                        Publish(message);
                    }
                }

                if (rightHandTracker.HandsDetected)
                {
                    Vector3 fingerAveragePosition = (rightHandTracker.ThumbTip.Position + rightHandTracker.IndexTip.Position) / 2.0f;
                    float distance = (transform.position - fingerAveragePosition).magnitude;
                    if (distance < 0.03)
                    {
                        UpdateMessage(rightHandTracker.ThumbTip.Position, rightHandTracker.IndexTip.Position);
                        Publish(message);
                    }
                }
            }
        }
        else if(uiController.GetCurrentMode() == UIController.Mode.INT || uiController.GetCurrentMode() == UIController.Mode.FULL)
        {
            if (internalModeHandTracker.HandsDetected)
            {
                UpdateMessage(internalModeHandTracker.ThumbTip.Position, internalModeHandTracker.IndexTip.Position);
                Publish(message);
            }
        }
    }

    void UpdateMessage(Vector3 thumbTipPosition, Vector3 indexTipPosition)
    {
        float distPercent = (Vector3.Magnitude(thumbTipPosition - indexTipPosition) - (float)minWidth) / ((float)maxWidth - (float)minWidth);
        distPercent = Math.Max(0.0f, Math.Min(1.0f, distPercent));
        message.data = Convert.ToString(distPercent, System.Globalization.CultureInfo.InvariantCulture);
    }

}
