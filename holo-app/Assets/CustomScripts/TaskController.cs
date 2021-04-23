using System.Collections;
using System;
using System.Collections.Generic;
using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.UI;
using RosSharp.RosBridgeClient;
using UnityEngine;
using UnityEngine.Events;

public class TaskController : MonoBehaviour
{
    public GameObject robotBase;
    public GameObject keyframeObject;
    public LineRenderer lineRenderer;

    private RosSharp.RosBridgeClient.Messages.Geometry.PoseArray message;
    private float keyframeScale = 0.5f;
    private List<GameObject> keyframeList;
    private Vector3[] keyframePositions;
    private List<DateTime> keyframeTimestampList;
    private bool isKeyframeManipulated;
    private bool isPathDrawerManipulated;

    private bool continuousReset = true;

    void Start()
    {
        message = new RosSharp.RosBridgeClient.Messages.Geometry.PoseArray();
        keyframeList = new List<GameObject>();
        keyframeTimestampList = new List<DateTime>();
        OnResetPressed();
    }

    void Update()
    {
        if (AddKeyframe() || isKeyframeManipulated)
        {
            lineRenderer.positionCount = keyframeList.Count;
            keyframePositions = new Vector3[keyframeList.Count];
            for (int i = 0; i < keyframeList.Count; i++)
            {
                GameObject kf = keyframeList[i];
                keyframePositions[i] = new Vector3(kf.transform.localPosition.x, kf.transform.localPosition.y, kf.transform.localPosition.z);
            }

            lineRenderer.SetPositions(keyframePositions);
        }

        if(isPathDrawerManipulated)
        {
            continuousReset = false;
        }
        
        if(continuousReset)
        {
            OnResetPressed();
        }
    }

    void OnDisable()
    {
        OnResetPressed();
    }

    private bool AddKeyframe()
    {
        if (!isPathDrawerManipulated) return false;

        Vector3 currentLocalPosition = transform.localPosition;
        Quaternion currentLocalOrientation = transform.localRotation;

        Vector3 currentGlobalPosition = transform.position;
        float currentGlobalScale = transform.lossyScale.x;
        Quaternion currentGlobalOrientation = transform.rotation;

        if (keyframeList.Count > 0)
        {
            Vector3 lastKfPosition = keyframeList[keyframeList.Count - 1].transform.position;
            Quaternion lastKfOrientation = keyframeList[keyframeList.Count - 1].transform.rotation;

            float distance = Vector3.Distance(currentGlobalPosition, lastKfPosition);
            float angle = Quaternion.Angle(currentGlobalOrientation, lastKfOrientation);

            if (distance > Utils.keyframe_distance_threshold * currentGlobalScale || angle > Utils.keyframe_angle_threshold)
            {
                GameObject newKeyframe = Instantiate(keyframeObject, robotBase.transform);
                newKeyframe.transform.localPosition = currentLocalPosition;
                newKeyframe.transform.localRotation = currentLocalOrientation;
                newKeyframe.transform.localScale = new Vector3(keyframeScale, keyframeScale, keyframeScale);
                newKeyframe.SetActive(true);
                if (keyframeTimestampList.Count == 1)
                {
                    // Set the timestamp of the first keyframe here!!!
                    keyframeTimestampList[0] = DateTime.UtcNow;
                }
                keyframeList.Add(newKeyframe);
                keyframeTimestampList.Add(DateTime.UtcNow);
                return true;
            }
        }
        else
        {
            OnResetPressed();
            GameObject newKeyframe = Instantiate(keyframeObject, robotBase.transform);
            newKeyframe.transform.localPosition = currentLocalPosition;
            newKeyframe.transform.localRotation = currentLocalOrientation;
            newKeyframe.transform.localScale = new Vector3(keyframeScale, keyframeScale, keyframeScale);
            newKeyframe.SetActive(false);
            newKeyframe.GetComponent<ManipulationHandler>().enabled = false;
            newKeyframe.GetComponent<NearInteractionGrabbable>().enabled = false;
            keyframeList.Add(newKeyframe);
            keyframeTimestampList.Add(DateTime.UtcNow);
            return true;
        }
        return false;
    }

    public RosSharp.RosBridgeClient.Messages.Geometry.Pose GeneratePoseMessage(GameObject obj)
    {
        RosSharp.RosBridgeClient.Messages.Geometry.Pose message = new RosSharp.RosBridgeClient.Messages.Geometry.Pose();
        Matrix4x4 localTargetPose = Matrix4x4.TRS(obj.transform.localPosition, obj.transform.localRotation, obj.transform.localScale);
        Matrix4x4 originBasePose = Matrix4x4.TRS(robotBase.transform.localPosition, robotBase.transform.localRotation, robotBase.transform.localScale);
        Matrix4x4 transformedEndGoalPose = originBasePose * localTargetPose;

        message.position = Utils.GetGeometryPoint(Utils.PositionFromMatrix(transformedEndGoalPose));
        message.orientation = Utils.GetGeometryQuaternion(Utils.QuaternionFromMatrix(transformedEndGoalPose));

        return message;
    }

    private void SnapToCurrentGlobalPosition()
    {
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

    public void SnapToLastKeyframe()
    {
        if (keyframeList.Count > 0)
        {
            transform.localPosition = keyframeList[keyframeList.Count - 1].transform.localPosition;
            transform.localRotation = keyframeList[keyframeList.Count - 1].transform.localRotation;
        }
        else
        {
            Debug.LogWarning("You called SnapToLastKeyframe with an empty keyframe list on Object " + gameObject.name);
        }
    }

    public void OnSendPressed()
    {
        // Update the position and orientation of the drawer
        //SnapToLastKeyframe();
        continuousReset = true;

        lineRenderer.positionCount = 0;

        if (keyframeList != null)
        {
            foreach (GameObject kf in keyframeList)
            {
                Destroy(kf);
            }
            keyframeList.Clear();
            keyframeTimestampList.Clear();
        }
    }

    // below is the function for the callback button for resetting the path drawn
    public void OnResetPressed()
    {
        // Update the position and orientation of the drawer
        SnapToCurrentGlobalPosition();
        lineRenderer.positionCount = 0;

        if (keyframeList != null)
        {
            foreach (GameObject kf in keyframeList)
            {
                Destroy(kf);
            }
            keyframeList.Clear();
            keyframeTimestampList.Clear();
        }
    }

    // below are the callbacks and functions for signalling manipulation to some other interface
    public void OnKeyframeManipulationStarted()
    {
        isKeyframeManipulated = true;
    }

    public void OnKeyframeManipulationEnded()
    {
        isKeyframeManipulated = false;
    }

    public void OnPathDrawerManipulationStarted()
    {
        isPathDrawerManipulated = true;
    }

    public void OnPathDrawerManipulationEnded()
    {
        isPathDrawerManipulated = false;
    }

    public List<GameObject> GetKeyframeList()
    {
        // Many functions depend on keyframe count to be larger than 0, so make it
        if (keyframeList.Count == 0)
        {
            Vector3 currentLocalPosition = transform.localPosition;
            Quaternion currentLocalOrientation = transform.localRotation;

            GameObject newKeyframe = Instantiate(keyframeObject, robotBase.transform);
            newKeyframe.transform.localPosition = currentLocalPosition;
            newKeyframe.transform.localRotation = currentLocalOrientation;
            newKeyframe.transform.localScale = new Vector3(keyframeScale, keyframeScale, keyframeScale);
            newKeyframe.SetActive(false);
            newKeyframe.GetComponent<ManipulationHandler>().enabled = false;
            newKeyframe.GetComponent<NearInteractionGrabbable>().enabled = false;

            List<GameObject> tmpList = new List<GameObject>();
            tmpList.Add(newKeyframe);
            return tmpList;
        }

        return keyframeList;
    }

    public List<DateTime> GetKeyframeTimestampList()
    {
        // Many functions depend on keyframe count to be larger than 0, so make it
        if (keyframeTimestampList.Count == 0)
        {
            List<DateTime> tmpList = new List<DateTime>();
            tmpList.Add(DateTime.UtcNow);
            return tmpList;
        }

        return keyframeTimestampList;
    }
}
