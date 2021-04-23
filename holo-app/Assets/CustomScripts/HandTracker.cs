using System.Collections;
using System.Collections.Generic;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.Utilities;
using UnityEngine;

public class HandTracker : MonoBehaviour
{
    public Handedness handedness;
    private MixedRealityPose thumbTip;
    private MixedRealityPose indexTip;
    private MixedRealityPose indexKnuckle;

    // Field accessors
    public bool HandsDetected { get; private set; }
    public MixedRealityPose ThumbTip => thumbTip;
    public MixedRealityPose IndexTip => indexTip;
    public MixedRealityPose IndexKnuckle => indexKnuckle;

    // Start is called before the first frame update
    protected virtual void Start()
    {
        HandsDetected = false;
    }

    // Update is called once per frame
    void Update()
    {
        HandsDetected = true;
        HandsDetected &= HandJointUtils.TryGetJointPose(TrackedHandJoint.ThumbTip, handedness, out thumbTip);
        HandsDetected &= HandJointUtils.TryGetJointPose(TrackedHandJoint.IndexTip, handedness, out indexTip);
        HandsDetected &= HandJointUtils.TryGetJointPose(TrackedHandJoint.IndexKnuckle, handedness, out indexKnuckle);
    }

    
}
