using Microsoft.MixedReality.Toolkit.Input;
using Microsoft.MixedReality.Toolkit.UI;
using Microsoft.MixedReality.Toolkit.Utilities.Solvers;
using RosSharp.RosBridgeClient;
using System;
using System.Collections;
using UnityEngine;

public class CameraStreamScreenHandler : Subscriber<RosSharp.RosBridgeClient.Messages.Sensor.CompressedImage>
{
    public Camera mainCamera;
    public UIController uiController;

    private Texture2D texture2D;
    private byte[] imageData;
    private bool isMessageReceived;
    private Transform initialTransform; // target transform in EXT mode
    private Transform changedTransform; // start transform in INT mode
    private float timeToReachTarget;
    private UIController.Mode currentMode;
    private UIController.Mode previousMode;

    // Components
    private MeshRenderer meshRenderer;
    private ManipulationHandler manipulationHandler;
    private NearInteractionGrabbable nearInteractionGrabbable;
    private BoundingBox boundingBox;
    private SolverHandler solverHandler;
    private RadialView radialView;

    protected override void Start()
    {
        base.Start();
        texture2D = new Texture2D(1, 1);
        meshRenderer.material = new Material(Shader.Find("Standard"));
    }

    private void Awake()
    {
        meshRenderer = gameObject.GetComponent<MeshRenderer>();
        manipulationHandler = gameObject.GetComponent<ManipulationHandler>();
        nearInteractionGrabbable = gameObject.GetComponent<NearInteractionGrabbable>();
        boundingBox = gameObject.GetComponent<BoundingBox>();
        solverHandler = gameObject.GetComponent<SolverHandler>();
        radialView = gameObject.GetComponent<RadialView>();

        //initialTransform = gameObject.transform;
        //Debug.Log("Initial position of CermaStreamQuad Object: " + initialTransform.position);

    }

    private void Update()
    {
        currentMode = uiController.GetCurrentMode();

        if (currentMode == UIController.Mode.EXT || currentMode == UIController.Mode.TASK)
        {
            // User stands outside the robot
            manipulationHandler.enabled = true;
            nearInteractionGrabbable.enabled = true;
            boundingBox.enabled = true;
            solverHandler.enabled = false;
            radialView.enabled = false;
            
            this.transform.localScale = new Vector3(this.transform.localScale.y * Camera.main.aspect, this.transform.localScale.y, 1.0f);
        }
        else if (currentMode == UIController.Mode.INT)
        {
            // User stands inside the robot and screen projected outside, not Fullscreen yet
            manipulationHandler.enabled = false;
            nearInteractionGrabbable.enabled = false;
            boundingBox.enabled = false;
            solverHandler.enabled = true;
            radialView.enabled = true;

            //changedTransform = gameObject.transform; // get transform of object changed by the radial view handler
            //Debug.Log("Changed Transform of CermaStreamQuad Object: " + changedTransform);

        }
        else if (currentMode == UIController.Mode.FULL)
        {
            // User stands inside the robot and is in Fullscreen mode
            manipulationHandler.enabled = false;
            nearInteractionGrabbable.enabled = false;
            boundingBox.enabled = false;
            solverHandler.enabled = false;
            radialView.enabled = false;

            // fix screen to head pose
            transform.rotation = mainCamera.transform.rotation;
            Vector3 offset = mainCamera.transform.forward * 0.7f;
            transform.position = mainCamera.transform.position + offset;
        }
        else
        {
            Debug.LogError("Current Mode not defined.");
        }

        if (isMessageReceived) StartCoroutine(ProcessMessage());
    }

    protected override void ReceiveMessage(RosSharp.RosBridgeClient.Messages.Sensor.CompressedImage compressedImage)
    {
        imageData = compressedImage.data;
        isMessageReceived = true;
    }

    private IEnumerator ProcessMessage()
    {
        //Debug.Log("ProcessMessage called.");
        texture2D.LoadImage(imageData); 
        texture2D.Apply(); // Actually apply all previous SetPixel and SetPixels changes.
        meshRenderer.material.SetTexture("_MainTex", texture2D);
        // or use: GetComponent<Renderer>().material.mainTexture = texture2D;
        isMessageReceived = false;
        yield return null;
    }
}