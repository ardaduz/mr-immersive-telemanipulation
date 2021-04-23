using System.Collections;
using System.Collections.Generic;
using RosSharp.RosBridgeClient;
using UnityEngine;

public class HeadPosePublisher : Publisher<RosSharp.RosBridgeClient.Messages.Geometry.Pose>
{
    public UIController uiController;
    public GameObject YuMi;

    private RosSharp.RosBridgeClient.Messages.Geometry.Pose message;
    private Quaternion defaultRotation;

    protected override void Start()
    {
        base.Start();
        message = new RosSharp.RosBridgeClient.Messages.Geometry.Pose();
        defaultRotation = Quaternion.Euler(0.0f, 0.0f, 0.0f);
    }

    void Update()
    {
        if (uiController.GetCurrentMode() == UIController.Mode.INT || uiController.GetCurrentMode() == UIController.Mode.FULL)
        {
            SendCurrentHeadPose();
        }
        else
        {
            SendDefaultPose();
        }
    }

    private void SendCurrentHeadPose()
    {

        Quaternion localRotation = Quaternion.Inverse(YuMi.transform.rotation) * transform.rotation;
        message.position = Utils.GetGeometryPoint(Vector3.zero);

        RosSharp.RosBridgeClient.Messages.Geometry.Quaternion temp = Utils.GetGeometryQuaternion(defaultRotation * localRotation);

        message.orientation.x = temp.z;
        message.orientation.y = temp.y;
        message.orientation.z = -temp.x;
        message.orientation.w = temp.w;

        Publish(message);
    }

    private void SendDefaultPose()
    {
        if (message == null)
        {
            Debug.LogError("SendDefaultPose message is null.");
            return;
        } 

        message.position = Utils.GetGeometryPoint(Vector3.zero);
        message.orientation.w = 0.85f;
        message.orientation.x = 0.0f;
        message.orientation.y = 0.0f;
        message.orientation.z = -0.5f;
        Publish(message);
    }
}
