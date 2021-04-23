using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public static class Utils
{
    public static string ROS_SERVER_IP = "ws://10.4.5.157:9090";
    public static float keyframe_distance_threshold = 0.15f;
    public static float keyframe_angle_threshold = 30.0f;
    public static float aspectRatio = 1.77778f; // aspect ratio of one frame

    public static Quaternion QuaternionFromMatrix(Matrix4x4 m)
	{
		if (m.GetColumn(2) == Vector4.zero)
		{
			return Quaternion.identity;
		}
		return Quaternion.LookRotation(m.GetColumn(2), m.GetColumn(1));
	}

	public static Vector3 PositionFromMatrix(Matrix4x4 m)
	{
		return m.GetColumn(3);
	}

	public static Vector3 ScaleFromMatrix(Matrix4x4 m)
	{
		var x = Mathf.Sqrt(m.m00 * m.m00 + m.m01 * m.m01 + m.m02 * m.m02);
		var y = Mathf.Sqrt(m.m10 * m.m10 + m.m11 * m.m11 + m.m12 * m.m12);
		var z = Mathf.Sqrt(m.m20 * m.m20 + m.m21 * m.m21 + m.m22 * m.m22);

		return new Vector3(x, y, z);
	}

    public static RosSharp.RosBridgeClient.Messages.Geometry.Point GetGeometryPoint(Vector3 position)
    {
        RosSharp.RosBridgeClient.Messages.Geometry.Point geometryPoint = new RosSharp.RosBridgeClient.Messages.Geometry.Point();
        geometryPoint.x = -position.x;
        geometryPoint.y = position.y;
        geometryPoint.z = position.z;
        return geometryPoint;
    }

    public static RosSharp.RosBridgeClient.Messages.Geometry.Quaternion GetGeometryQuaternion(Quaternion quaternion)
    {
        RosSharp.RosBridgeClient.Messages.Geometry.Quaternion geometryQuaternion = new RosSharp.RosBridgeClient.Messages.Geometry.Quaternion();
        geometryQuaternion.x = -quaternion.x;
        geometryQuaternion.y = quaternion.y;
        geometryQuaternion.z = quaternion.z;
        geometryQuaternion.w = -quaternion.w;
        return geometryQuaternion;
    }
}
