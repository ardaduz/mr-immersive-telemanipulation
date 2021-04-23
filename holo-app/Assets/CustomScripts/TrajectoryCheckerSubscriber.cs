

using UnityEngine;
using RosSharp;
using RosSharp.RosBridgeClient;
using System.Collections;

using System.Collections.Generic;
using System;
using System.Diagnostics;
using Debug = UnityEngine.Debug;
using Random = UnityEngine.Random;
using Microsoft.MixedReality.Toolkit.UI;
using Microsoft.MixedReality.Toolkit.Input;

public class TrajectoryCheckerSubscriber : Subscriber<RosSharp.RosBridgeClient.Messages.Geometry.PoseArray>
{
    private RosSharp.RosBridgeClient.Messages.Geometry.PoseArray msg;

    private bool hasSimulationError = false;
    private bool needsButtonUpdate = false;

    private float threshold = 0.1f;

    private HashSet<int> leftApplyErrorShaderOnUpdate;
    private HashSet<int> rightApplyErrorShaderOnUpdate;
    private HashSet<int> leftApplyDefaultShaderOnUpdate;
    private HashSet<int> rightApplyDefaultShaderOnUpdate;

    [Header("Object References")]

    public GlobalTaskController globalTaskController;
    public TaskController leftArmTaskController;
    public TaskController rightArmTaskController;

    public Material defaultLeftArmMaterial;
    public Material defaultRightArmMaterial;
    public Material errorPoseMaterial;

    public Material buttonEnabledMaterial;
    public Material buttonDisabledMaterial;

    public GameObject[] SendButtons => sendButtons;
    [SerializeField]
    [Tooltip("The sendButtons to be disabled on existing errors.")]
    private GameObject[] sendButtons = new GameObject[0];

    private bool isReadyToColor;

    protected override void Start()
    {
        base.Start();

        leftApplyErrorShaderOnUpdate = new HashSet<int>();
        rightApplyErrorShaderOnUpdate = new HashSet<int>();
        leftApplyDefaultShaderOnUpdate = new HashSet<int>();
        rightApplyDefaultShaderOnUpdate = new HashSet<int>();
    }

    private void Update()
    {
        if (isReadyToColor)
        {
            UpdateKeyframeColors();
            isReadyToColor = false;
        }

        if (needsButtonUpdate)
        {
            UpdateStatusOnSendButtons();
        }

    }

    private void UpdateStatusOnSendButtons()
    {
        foreach (GameObject sendButton in sendButtons)
        {
            sendButton.GetComponentInChildren<PressableButtonHoloLens2>().enabled = !hasSimulationError;
            sendButton.GetComponentInChildren<NearInteractionTouchable>().enabled = !hasSimulationError;
            sendButton.GetComponentInChildren<Interactable>().enabled = !hasSimulationError;
            sendButton.GetComponentInChildren<MeshRenderer>().material = hasSimulationError ? buttonDisabledMaterial : buttonEnabledMaterial;
        }
        needsButtonUpdate = false;
    }

    private void UpdateKeyframeColors()
    {
        List<GameObject> rightArmKeyframeList = rightArmTaskController.GetKeyframeList();
        List<GameObject> leftArmKeyframeList = leftArmTaskController.GetKeyframeList();
        GameObject element;

        foreach (int i in leftApplyErrorShaderOnUpdate)
        {
            element = leftArmKeyframeList[i];
            element.GetComponentInChildren<MeshRenderer>().material = errorPoseMaterial;
        }
        leftApplyErrorShaderOnUpdate.Clear();

        foreach (int i in rightApplyErrorShaderOnUpdate)
        {
            element = rightArmKeyframeList[i];
            element.GetComponentInChildren<MeshRenderer>().material = errorPoseMaterial;
        }
        rightApplyErrorShaderOnUpdate.Clear();

        foreach (int i in leftApplyDefaultShaderOnUpdate)
        {
            element = leftArmKeyframeList[i];
            element.GetComponentInChildren<MeshRenderer>().material = defaultLeftArmMaterial;
        }
        leftApplyDefaultShaderOnUpdate.Clear();

        foreach (int i in rightApplyDefaultShaderOnUpdate)
        {
            element = rightArmKeyframeList[i];
            element.GetComponentInChildren<MeshRenderer>().material = defaultRightArmMaterial;
        }
        rightApplyDefaultShaderOnUpdate.Clear();
    }

    protected override void ReceiveMessage(RosSharp.RosBridgeClient.Messages.Geometry.PoseArray message)
    {
        isReadyToColor = false;

        int messageCount = message.poses.Length;

        RosSharp.RosBridgeClient.Messages.Geometry.Pose[] lastPoseArray = globalTaskController.GetLastPoseArray();
        int[] lastKeyframeIndices = globalTaskController.GetKeyframeIndices();
        hasSimulationError = false;

        if (messageCount != lastPoseArray.Length)
        {
            Debug.LogError("Trajectory message send poses and received poses size does not match !!!");
        }

        for (int i = 0; i < messageCount; i = i + 2)
        {
            RosSharp.RosBridgeClient.Messages.Geometry.Pose leftMessagePose = message.poses[i];
            RosSharp.RosBridgeClient.Messages.Geometry.Pose rightMessagePose = message.poses[i + 1];

            RosSharp.RosBridgeClient.Messages.Geometry.Pose leftKeyframePose = lastPoseArray[i];
            RosSharp.RosBridgeClient.Messages.Geometry.Pose rightKeyframePose = lastPoseArray[i + 1];

            int leftKeyframeIndex = lastKeyframeIndices[i];
            int rightKeyframeIndex = lastKeyframeIndices[i + 1];

            Vector3 leftMessagePosition = new Vector3(-leftMessagePose.position.x, leftMessagePose.position.y, leftMessagePose.position.z);
            Vector3 rightMessagePosition = new Vector3(-rightMessagePose.position.x, rightMessagePose.position.y, rightMessagePose.position.z);

            Vector3 leftKeyframePosition = new Vector3(-leftKeyframePose.position.x, leftKeyframePose.position.y, leftKeyframePose.position.z);
            Vector3 rightKeyframePosition = new Vector3(-rightKeyframePose.position.x, rightKeyframePose.position.y, rightKeyframePose.position.z);

            float currentLeftDistance = Math.Abs(Vector3.Distance(leftMessagePosition, leftKeyframePosition));
            float currentRightDistance = Math.Abs(Vector3.Distance(rightMessagePosition, rightKeyframePosition));

            if (currentLeftDistance > threshold)
            {
                hasSimulationError = true;
                leftApplyErrorShaderOnUpdate.Add(leftKeyframeIndex);
            }
            else
            {
                leftApplyDefaultShaderOnUpdate.Add(leftKeyframeIndex);
            }

            if (currentRightDistance > threshold)
            {
                hasSimulationError = true;
                rightApplyErrorShaderOnUpdate.Add(rightKeyframeIndex);
            }
            else
            {
                rightApplyDefaultShaderOnUpdate.Add(rightKeyframeIndex);
            }

            isReadyToColor = true;
        }

        needsButtonUpdate = true;
    }

    public void TrajectoryNeedsServerUpdate()
    {
        hasSimulationError = true;
        needsButtonUpdate = true;
    }
}
