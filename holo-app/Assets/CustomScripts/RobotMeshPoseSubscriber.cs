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

public class RobotMeshPoseSubscriber : Subscriber<RosSharp.RosBridgeClient.Messages.Geometry.PoseArray>
{
    private Vector3[] positions;
    private Quaternion[] rotations;
    private bool isMessageReceived;
    private GameObject armMeshes;
    private GameObject leftWorkspace;
    private GameObject rightWorkspace;

    private bool isFirstMessage = true;

    protected override void Start()
    {
        base.Start();

        armMeshes = gameObject.transform.Find("ArmMeshes").gameObject;
        if (armMeshes == null) Debug.LogError("Could not find ArmMeshes gameObject!");

        leftWorkspace = gameObject.transform.Find("left_workspace").gameObject;
        if (leftWorkspace == null) Debug.LogError("Could not find left_workspace gameObject!");

        rightWorkspace = gameObject.transform.Find("right_workspace").gameObject;
        if (rightWorkspace == null) Debug.LogError("Could not find right_workspace gameObject!");
    }

    private void Update()
    {
        if (isMessageReceived)
            UpdateRobotPose();

        // stimulate the end effector target poses in the beginning
        if (isFirstMessage)
        {
            EffectorTargetController[] effectorControllers = GetComponentsInChildren<EffectorTargetController>();

            foreach (var effectorController in effectorControllers)
            {
                effectorController.OnManipulationEnded();
            }

            isFirstMessage = false;
        }
            
    }

    protected override void ReceiveMessage(RosSharp.RosBridgeClient.Messages.Geometry.PoseArray message)
    {
        ParsePositions(message);
        ParseRotations(message);
        isMessageReceived = true;
    }

    private void UpdateRobotPose()
    {
        if (armMeshes.transform.childCount != positions.Length - 1) Debug.LogError("Received MeshPose message size differs from the number of mesh gameobjects: " + armMeshes.transform.childCount.ToString() + " vs " + (positions.Length - 1).ToString());

        // update pose of left and right workspace (robot body)
        leftWorkspace.transform.localPosition = positions[0];
        leftWorkspace.transform.localRotation = rotations[0];
        leftWorkspace.transform.localPosition = positions[0];
        leftWorkspace.transform.localRotation = rotations[0];

        // Update all other joints
        for (int i = 0; i < armMeshes.transform.childCount; i++)
        {
            Transform child = armMeshes.transform.GetChild(i);
            child.localPosition = positions[i + 1];
            child.localRotation = rotations[i + 1];
        }

        isMessageReceived = false;
    }

    private void ParsePositions(RosSharp.RosBridgeClient.Messages.Geometry.PoseArray message)
    {
        positions = new Vector3[message.poses.Length];

        for (int i = 0; i < message.poses.Length; i++)
        {
            RosSharp.RosBridgeClient.Messages.Geometry.Pose pose = message.poses[i];
            positions[i] = new Vector3(-pose.position.x, pose.position.y, pose.position.z);
        }
    }

    private void ParseRotations(RosSharp.RosBridgeClient.Messages.Geometry.PoseArray message)
    {
        rotations = new Quaternion[message.poses.Length];

        for (int i = 0; i < message.poses.Length; i++)
        {
            RosSharp.RosBridgeClient.Messages.Geometry.Pose pose = message.poses[i];
            rotations[i] = new Quaternion(-pose.orientation.x, pose.orientation.y, pose.orientation.z, -pose.orientation.w);
        }
    }
}
