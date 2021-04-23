using UnityEngine;
using RosSharp.RosBridgeClient;
using System.Collections;
using System.Diagnostics;
using Debug = UnityEngine.Debug;

public class RobotTrajectorySubscriber : Subscriber<RosSharp.RosBridgeClient.Messages.Geometry.PoseArray>
{
    public GameObject previewYumi;

    private RosSharp.RosBridgeClient.Messages.Geometry.PoseArray message;
    private bool isAnimating;
    private bool needsNewAnimation;
    private bool needsLoopingSameAnimation;
    private int meshCount;

    private Stopwatch stopwatch;
    private long animationPeriod = 66; //in milliseconds
    private float animationDelay = 2.0f; //in seconds
    private int animationStep;

    private Vector3[] positions;
    private Quaternion[] rotations;

    protected override void Start()
    {
        base.Start();
        meshCount = previewYumi.transform.childCount;
        stopwatch = Stopwatch.StartNew();
    }

    private void Update()
    {
        UpdateRobotPose();
    }

    protected override void ReceiveMessage(RosSharp.RosBridgeClient.Messages.Geometry.PoseArray message)
    {
        ParsePositions(message);
        ParseRotations(message);

        isAnimating = false;
        needsNewAnimation = true;
        needsLoopingSameAnimation = true;
    }

    private void UpdateRobotPose()
    {
        if (needsNewAnimation && needsLoopingSameAnimation)
        {
            if (!isAnimating)
            {
                stopwatch.Restart();
                animationStep = 0;
                previewYumi.SetActive(true);
                isAnimating = true;
            }

            long elapsed = stopwatch.ElapsedMilliseconds;
            if (elapsed >= animationStep * animationPeriod)
            { 
                int currentAnimationStepBegin = animationStep * meshCount;
                int currentAnimationStepEnd = currentAnimationStepBegin + meshCount;
                animationStep++;

                for (int i = 0; i < meshCount; i++)
                {
                    Transform child = previewYumi.transform.GetChild(i);
                    child.localPosition = positions[currentAnimationStepBegin + i];
                    child.localRotation = rotations[currentAnimationStepBegin + i];
                }

                if (currentAnimationStepEnd == positions.Length)
                {
                    isAnimating = false;
                    needsLoopingSameAnimation = false;
                    StartCoroutine(WaitInBetweenAnimationCoroutine());
                }
            }
        }
    }

    public void OnResetPressed()
    {
        StopAllCoroutines();
        previewYumi.SetActive(false);
        isAnimating = true;
        needsNewAnimation = false;
    }

    private void ParsePositions(RosSharp.RosBridgeClient.Messages.Geometry.PoseArray message)
    {
        positions = new Vector3[message.poses.Length];
        for (int i = 0; i < message.poses.Length; i++)
        {
            var pose = message.poses[i];
            var position = new Vector3(-pose.position.x, pose.position.y, pose.position.z);
            positions[i] = position;
        }
    }

    private void ParseRotations(RosSharp.RosBridgeClient.Messages.Geometry.PoseArray message)
    {
        rotations = new Quaternion[message.poses.Length];
        for (int i = 0; i < message.poses.Length; i++)
        {
            var pose = message.poses[i];
            var rotation = new Quaternion(-pose.orientation.x, pose.orientation.y, pose.orientation.z, -pose.orientation.w);
            rotations[i] = rotation;
        }
    }

    IEnumerator WaitInBetweenAnimationCoroutine()
    {
        yield return new WaitForSeconds(animationDelay);
        needsLoopingSameAnimation = true;
    }
}
