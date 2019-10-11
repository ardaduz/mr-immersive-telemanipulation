using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class EffectorTargetControl : MonoBehaviour
{
	public BoxCollider workspaceBounds;

    void Start()
    {

    }
    // Update is called once per frame
    void Update()
    {
        Vector3 currentMins = workspaceBounds.bounds.min;
        Vector3 currentMaxs = workspaceBounds.bounds.max;

        float clampedX = Mathf.Clamp(transform.position.x, currentMins.x, currentMaxs.x);
        float clampedY = Mathf.Clamp(transform.position.y, currentMins.y, currentMaxs.y);
        float clampedZ = Mathf.Clamp(transform.position.z, currentMins.z, currentMaxs.z);

        transform.position = new Vector3(clampedX, clampedY, clampedZ);
    }
}
