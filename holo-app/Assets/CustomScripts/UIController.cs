using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class UIController : MonoBehaviour
{
    public GameObject handMenu;
    public GameObject taskModeOneHandMenu;
    public GameObject externalModeOneHandMenu;
    public GameObject leftEffectorManipulator;
    public GameObject rightEffectorManipulator;
    public GameObject leftPathDrawer;
    public GameObject rightPathDrawer;
    public GameObject mainCamera;
    public GameObject leftHand;
    public GameObject rightHand;
    public GameObject yumi;
    public GameObject body;
    public GameObject globalTaskController;

    public double headingThreshold = 90;
    public double bodyDistanceThreshold = 0.2;
    public double handDistanceThreshold = 0.2;

    private GripperAlignedController leftInternalEE;
    private GripperAlignedController rightInternalEE;
    private GripperWidthController leftGripper;
    private GripperWidthController rightGripper;
    private EffectorTargetController leftExternalEE;
    private EffectorTargetController rightExternalEE;
    private HandTracker leftHandTracker;
    private HandTracker rightHandTracker;
    private GlobalTaskController taskController;
    private RobotTrajectorySubscriber trajectorySubscriber;

    // MODE EXT: External mode (grabbing and dragging the arms)
    // MODE TASK: Task mode (drawing a task)
    // MODE INT: Internal mode (tracking hands and mapping them onto robot end-effectors directly)
    // MODE FULL: Internal mode with full-screen RGB streams (fully immersive mode, robot hologram not visible)
    public enum Mode { EXT, TASK, INT, FULL}
    private Mode currentMode = Mode.EXT;

    void Start()
    {
        Debug.developerConsoleVisible = false;
        leftInternalEE = leftEffectorManipulator.GetComponent<GripperAlignedController>();
        rightInternalEE = rightEffectorManipulator.GetComponent<GripperAlignedController>();
        leftGripper = leftEffectorManipulator.GetComponentInChildren<GripperWidthController>();
        rightGripper = rightEffectorManipulator.GetComponentInChildren<GripperWidthController>();
        leftExternalEE = leftEffectorManipulator.GetComponent<EffectorTargetController>();
        rightExternalEE = rightEffectorManipulator.GetComponent<EffectorTargetController>();
        leftHandTracker = leftHand.GetComponent<HandTracker>();
        rightHandTracker = rightHand.GetComponent<HandTracker>();
        taskController = globalTaskController.GetComponent<GlobalTaskController>();
        trajectorySubscriber = globalTaskController.GetComponent<RobotTrajectorySubscriber>();

        UpdateUI();
    }

    void Update()
    {
        bool userInHologram = UserStandsInHologram();
        bool userHandsInEE = UserHandsCloseToEE();

        if (currentMode == Mode.EXT && userInHologram && userHandsInEE)
        {
            Debug.Log("Automatically switch to INT");
            currentMode = Mode.INT;
        }
        else if (currentMode == Mode.TASK && userInHologram && userHandsInEE)
        {
            Debug.Log("Automatically switch to INT");
            taskController.OnResetPressed();
            trajectorySubscriber.OnResetPressed();
            currentMode = Mode.INT;
        }
        else if ((currentMode == Mode.INT || currentMode == Mode.FULL) && !userInHologram)
        {
            Debug.Log("Automatically switch to EXT");
            currentMode = Mode.EXT;
        }

        UpdateUI();
    }

    public void OnExternalModeSwitchRequested()
    {
        if (currentMode == Mode.INT || currentMode == Mode.FULL)
        {
            MoveRobotInFrontOfTheCamera();
        }

        taskController.OnResetPressed();
        trajectorySubscriber.OnResetPressed();

        currentMode = Mode.EXT;
    }

    public void OnTaskModeSwitchRequested()
    {
        if (currentMode == Mode.INT || currentMode == Mode.FULL)
        {
            MoveRobotInFrontOfTheCamera();
        }

        taskController.OnResetPressed();
        trajectorySubscriber.OnResetPressed();

        currentMode = Mode.TASK;
    }

    public void OnInternalModeSwitchRequested()
    {
        if (currentMode == Mode.EXT || currentMode == Mode.TASK)
        {
            MoveRobotToCamera();
        }

        taskController.OnResetPressed();
        trajectorySubscriber.OnResetPressed();
        // Will be automatically switched if users hands are close to the end-effectors
    }

    public void OnFullscreenToggleRequested()
    {
        Debug.Log("Fullscreen Toggle Requested");
        if (currentMode == Mode.INT)
        {
            currentMode = Mode.FULL;
            DisableRenderingYuMi();
        }
        else if (currentMode == Mode.FULL)
        {
            currentMode = Mode.INT;
            EnableRenderingYuMi();
        }
    }

    public void UpdateUI()
    {
        bool isEXT = (currentMode == Mode.EXT);
        bool isINT = (currentMode == Mode.INT);
        bool isTASK = (currentMode == Mode.TASK);
        bool isFULL = (currentMode == Mode.FULL);

        handMenu.SetActive(isEXT || isTASK);
        externalModeOneHandMenu.SetActive(isEXT);
        taskModeOneHandMenu.SetActive(isTASK);

        leftInternalEE.enabled = isINT || isFULL;
        rightInternalEE.enabled = isINT || isFULL;
        leftGripper.enabled = isEXT || isINT || isFULL;
        rightGripper.enabled = isEXT || isINT || isFULL;

        leftExternalEE.enabled = isEXT;
        rightExternalEE.enabled = isEXT;

        leftPathDrawer.SetActive(isTASK);
        rightPathDrawer.SetActive(isTASK);
    }

    private bool UserStandsInHologram()
    {
        float headingDiff = Math.Abs(Mathf.DeltaAngle(mainCamera.transform.eulerAngles.y, yumi.transform.eulerAngles.y));

        Vector3 expectedGlobalBodyPosition = mainCamera.transform.position + 0.6f * Vector3.down; // camera position + mesh offset
        Vector3 currentGlobalBodyPosition = body.transform.position;
        Vector3 positionDifference = expectedGlobalBodyPosition - currentGlobalBodyPosition;
        return (positionDifference.magnitude <= bodyDistanceThreshold && headingDiff <= headingThreshold);
    }

    private bool UserHandsCloseToEE()
    {
        bool handsDetected = leftHandTracker.HandsDetected && rightHandTracker.HandsDetected;

        if (handsDetected)
        {
            double leftHandDistance = Vector3.Distance(leftHandTracker.ThumbTip.Position, leftExternalEE.transform.position);
            double rightHandDistance = Vector3.Distance(rightHandTracker.ThumbTip.Position, rightExternalEE.transform.position);

            if (currentMode == Mode.TASK)
            {
                leftHandDistance = Vector3.Distance(leftHandTracker.ThumbTip.Position, leftPathDrawer.transform.position);
                rightHandDistance = Vector3.Distance(rightHandTracker.ThumbTip.Position, rightPathDrawer.transform.position);
            }

            if (leftHandDistance < handDistanceThreshold && rightHandDistance < handDistanceThreshold)
            {
                return true;
            }
        }

        return false;
    }

    private void EnableRenderingYuMi()
    {
        MeshRenderer[] meshRenderers = yumi.GetComponentsInChildren<MeshRenderer>();

        foreach (MeshRenderer meshRenderer in meshRenderers)
        {
            meshRenderer.enabled = true;
        }

    }
    private void DisableRenderingYuMi()
    {
        MeshRenderer[] meshRenderers = yumi.GetComponentsInChildren<MeshRenderer>();

        foreach (MeshRenderer meshRenderer in meshRenderers)
        {
            meshRenderer.enabled = false;
        }
    }

    public Mode GetCurrentMode()
    {
        return currentMode;
    }

    public void MoveRobotToCamera()
    {
        // Rotate the YuMi to the correct orientation
        yumi.transform.eulerAngles = new Vector3(0, mainCamera.transform.eulerAngles.y, 0);

        // Get the final goal for the body pose
        Quaternion targetGlobalBodyOrientation = body.transform.rotation; // we already rotated yumi, we want to keep this orientation
        Vector3 targetGlobalBodyPosition = mainCamera.transform.position + 0.6f * Vector3.down; // camera position + mesh offset
        Matrix4x4 targetGlobalBodyPose = Matrix4x4.TRS(targetGlobalBodyPosition, targetGlobalBodyOrientation, Vector3.one);

        // Get the current poses
        Matrix4x4 currentGlobalBodyPose = Matrix4x4.TRS(body.transform.position, body.transform.rotation, Vector3.one);
        Matrix4x4 currentLocalBodyPose = Matrix4x4.TRS(body.transform.localPosition, body.transform.localRotation, Vector3.one);
        Matrix4x4 currentYuMiPose = Matrix4x4.TRS(yumi.transform.position, yumi.transform.rotation, Vector3.one);

        // CurrentGlobalBodyPose = YuMiPose * CurrentLocalBodyPose
        // TargetGlobalBodyPose = ? * CurrentGlobalBodyPose = ? * YuMiPose * CurrentLocalBodyPose
        Matrix4x4 transformation = targetGlobalBodyPose * currentGlobalBodyPose.inverse;
        Matrix4x4 newYuMiPose = transformation * currentYuMiPose;

        yumi.transform.position = Utils.PositionFromMatrix(newYuMiPose);
    }

    public void MoveRobotInFrontOfTheCamera()
    {
        // Rotate the YuMi to the correct orientation (reverse direction of the camera around y axis)
        yumi.transform.eulerAngles = new Vector3(0, mainCamera.transform.eulerAngles.y + 180, 0);

        Quaternion trimmedCameraOrientation = Quaternion.Euler(0, mainCamera.transform.eulerAngles.y, 0);
        Vector3 cameraPosition = mainCamera.transform.position;
        GameObject trimmedCameraObject = new GameObject();
        trimmedCameraObject.transform.position = cameraPosition;
        trimmedCameraObject.transform.rotation = trimmedCameraOrientation;

        // Get the final goal for the body pose
        Quaternion targetGlobalBodyOrientation = body.transform.rotation; // we already rotated yumi, we want to keep this orientation
        Vector3 targetGlobalBodyPosition = trimmedCameraObject.transform.position - 0.2f * trimmedCameraObject.transform.up + 1.5f* trimmedCameraObject.transform.forward; // camera position + mesh offset
        Matrix4x4 targetGlobalBodyPose = Matrix4x4.TRS(targetGlobalBodyPosition, targetGlobalBodyOrientation, Vector3.one);

        // Get the current poses
        Matrix4x4 currentGlobalBodyPose = Matrix4x4.TRS(body.transform.position, body.transform.rotation, Vector3.one);
        Matrix4x4 currentLocalBodyPose = Matrix4x4.TRS(body.transform.localPosition, body.transform.localRotation, Vector3.one);
        Matrix4x4 currentYuMiPose = Matrix4x4.TRS(yumi.transform.position, yumi.transform.rotation, Vector3.one);

        // CurrentGlobalBodyPose = YuMiPose * CurrentLocalBodyPose
        // TargetGlobalBodyPose = ? * CurrentGlobalBodyPose = ? * YuMiPose * CurrentLocalBodyPose
        Matrix4x4 transformation = targetGlobalBodyPose * currentGlobalBodyPose.inverse;
        Matrix4x4 newYuMiPose = transformation * currentYuMiPose;

        yumi.transform.position = Utils.PositionFromMatrix(newYuMiPose);

        Destroy(trimmedCameraObject);
    }
}
