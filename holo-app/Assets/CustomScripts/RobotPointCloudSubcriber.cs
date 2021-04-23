using System;
using System.Collections;
using System.Collections.Concurrent;
using System.Collections.Generic;
using RosSharp;
using RosSharp.RosBridgeClient;
using UnityEngine;

public class RobotPointCloudSubcriber : Subscriber<RosSharp.RosBridgeClient.Messages.Sensor.PointCloud2>
{
    public MeshFilter meshFilter;

    private bool isMessageReceived;
    private int targetVertexCount = 65535; // Unity max for mesh vertex count without submeshing
    private float tessellationOffset = 0.0025f;

    private Vector3[] pointCloudVertices;
    private Color[] pointCloudColors;
    private int[] pointCloudIndices;

    private List<Vector3> tempVertices;
    private List<Color> tempColors;
    private List<int> tempIndices;

    protected override void Start()
    {
        base.Start();
        tempVertices = new List<Vector3>();
        tempColors = new List<Color>();
        tempIndices = new List<int>();
    }

    void Update()
    {
        if (isMessageReceived)
        {
            meshFilter.mesh.Clear();
            meshFilter.mesh.vertices = pointCloudVertices;
            meshFilter.mesh.colors = pointCloudColors;
            meshFilter.mesh.SetIndices(pointCloudIndices, MeshTopology.Points, 0);
            isMessageReceived = false;
        }
    }

    protected override void ReceiveMessage(RosSharp.RosBridgeClient.Messages.Sensor.PointCloud2 message)
    {
        if (message.width > 0)
        {
            PopulateMeshWithVertices(message);

            pointCloudVertices = tempVertices.ToArray();
            pointCloudColors = tempColors.ToArray();
            pointCloudIndices = tempIndices.ToArray();
        }
        else
        {
            pointCloudVertices = new Vector3[0];
            pointCloudColors = new Color[0];
            pointCloudIndices = new int[0];
        }

        isMessageReceived = true;
    }

    private void PopulateMeshWithVertices(RosSharp.RosBridgeClient.Messages.Sensor.PointCloud2 message)
    {
        int oneAxisValueByteSize = 4; // 4 (float32)
        int oneVertexByteSize = message.point_step;
        int numberOfVertices = message.width;

        tempVertices.Clear();
        tempColors.Clear();
        tempIndices.Clear();

        // populate with tessellation
        int index = 0;
        for (uint round = 0; round < 15; round++)
        {
            Vector3 tessellationVector = GetTessellationVectorOffset(round);
            int offset = 0;
            for (int i = 0; i < numberOfVertices; i++)
            {
                float x = BitConverter.ToSingle(message.data, offset);
                float y = BitConverter.ToSingle(message.data, offset + oneAxisValueByteSize);
                float z = BitConverter.ToSingle(message.data, offset + 2 * oneAxisValueByteSize);

                float r_float = message.data[offset + 4 * oneAxisValueByteSize] / 255.0f;
                float g_float = message.data[offset + 4 * oneAxisValueByteSize + 1] / 255.0f;
                float b_float = message.data[offset + 4 * oneAxisValueByteSize + 2] / 255.0f;

                Vector3 point = new Vector3(y, -x, -z) + tessellationVector;
                Color color = new Color(r_float, g_float, b_float);
                tempVertices.Add(point);
                tempColors.Add(color);
                tempIndices.Add(index);

                offset += oneVertexByteSize;
                index++;

                if (index == targetVertexCount)
                {
                    return;
                }
            }
        }

    }

    private Vector3 GetTessellationVectorOffset(uint round)
    {
        switch (round)
        {
            case 1:
                return Vector3.up * tessellationOffset;
            case 2:
                return Vector3.down * tessellationOffset;
            case 3:
                return Vector3.left * tessellationOffset;
            case 4:
                return Vector3.right * tessellationOffset;
            case 5:
                return Vector3.back * tessellationOffset;
            case 6:
                return Vector3.forward * tessellationOffset;
            case 7:
                return new Vector3(1, 1, 1).normalized * tessellationOffset;
            case 8:
                return new Vector3(1, 1, -1).normalized * tessellationOffset;
            case 9:
                return new Vector3(1, -1, 1).normalized * tessellationOffset;
            case 10:
                return new Vector3(1, -1, -1).normalized * tessellationOffset;
            case 11:
                return new Vector3(-1, 1, 1).normalized * tessellationOffset;
            case 12:
                return new Vector3(-1, 1, -1).normalized * tessellationOffset;
            case 13:
                return new Vector3(-1, -1, 1).normalized * tessellationOffset;
            case 14:
                return new Vector3(-1, -1, -1).normalized * tessellationOffset;
            default:
                return Vector3.zero;
        }
    }
}
