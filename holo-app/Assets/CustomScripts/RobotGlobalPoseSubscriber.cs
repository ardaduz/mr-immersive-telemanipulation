/*
© Siemens AG, 2017-2018
Author: Dr. Martin Bischoff (martin.bischoff@siemens.com)

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
<http://www.apache.org/licenses/LICENSE-2.0>.
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

using UnityEngine;
using RosSharp;
using RosSharp.RosBridgeClient;

public class RobotGlobalPoseSubscriber : Subscriber<RosSharp.RosBridgeClient.Messages.Geometry.PoseArray>
{
    private Vector3[] positions;
    private Quaternion[] rotations;

    // these fields can only be read outside of this class, do not modify it for safety
    public static Vector3 LeftEEPositionFromServer { get; private set; }
    public static Vector3 RightEEPositionFromServer { get; private set; }
    public static Quaternion LeftEERotationFromServer { get; private set; }
    public static Quaternion RightEERotationFromServer { get; private set; }
    public static Vector3 LeftElbowPositionFromServer { get; private set; }
    public static Vector3 RightElbowPositionFromServer { get; private set; }
    public static bool hasReceivedFirstMessage { get; private set; }

    protected override void Start()
    {
        base.Start();
        hasReceivedFirstMessage = false;
    }

    void Update()
    {
        try
        {
            LeftEEPositionFromServer = positions[0];
            RightEEPositionFromServer = positions[1];
            LeftEERotationFromServer = rotations[0];
            RightEERotationFromServer = rotations[1];
            LeftElbowPositionFromServer = positions[2];
            RightElbowPositionFromServer = positions[3];
        }
        catch
        {
            return;
        }
    }

    protected override void ReceiveMessage(RosSharp.RosBridgeClient.Messages.Geometry.PoseArray message)
    {
        ParsePositions(message);
        ParseRotations(message);
        hasReceivedFirstMessage = true;
    }

    private void ParsePositions(RosSharp.RosBridgeClient.Messages.Geometry.PoseArray message)
    {
        positions = new Vector3[message.poses.Length];
        for (int i = 0; i < message.poses.Length; i++)
        {
            RosSharp.RosBridgeClient.Messages.Geometry.Pose pose = message.poses[i];
            Vector3 position = new Vector3(-pose.position.x, pose.position.y, pose.position.z);
            positions[i] = position;
        }
    }

    private void ParseRotations(RosSharp.RosBridgeClient.Messages.Geometry.PoseArray message)
    {
        rotations = new Quaternion[message.poses.Length];
        for (int i = 0; i < message.poses.Length; i++)
        {
            RosSharp.RosBridgeClient.Messages.Geometry.Pose pose = message.poses[i];
            Quaternion rotation = new Quaternion(-pose.orientation.x, pose.orientation.y, pose.orientation.z, -pose.orientation.w);
            rotations[i] = rotation;
        }
    }
}
