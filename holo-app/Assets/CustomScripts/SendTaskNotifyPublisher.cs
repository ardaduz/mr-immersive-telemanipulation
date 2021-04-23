using System.Collections;
using System.Collections.Generic;
using RosSharp.RosBridgeClient;
using UnityEngine;

public class SendTaskNotifyPublisher : Publisher<RosSharp.RosBridgeClient.Messages.Standard.String>
{
    private RosSharp.RosBridgeClient.Messages.Standard.String message;

    protected override void Start()
    {
        base.Start();
    }

    public void SendTaskNotify()
    {
        message = new RosSharp.RosBridgeClient.Messages.Standard.String(" ");
        Publish(message);
    }
}
