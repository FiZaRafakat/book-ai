---
sidebar_position: 2
---

# Chapter 2: Visual Environments with Unity

## Introduction to Unity for Robotics Applications

Unity is a powerful cross-platform game engine that has found significant application in robotics, particularly for creating high-fidelity visual environments. In the context of digital twins for humanoid robots, Unity excels at providing realistic rendering capabilities that complement the physics simulation provided by tools like Gazebo.

Unity's strength in robotics applications comes from its ability to create visually compelling environments that can be used for various purposes:

- **AI Training**: Visual environments provide rich sensory input for machine learning algorithms
- **Human-Robot Interaction**: Realistic visualizations help humans understand robot behavior
- **Simulation Validation**: Visual feedback allows for intuitive verification of robot actions
- **Prototyping**: Rapid development of complex visual scenarios for testing robot capabilities

The Unity robotics ecosystem includes specialized packages and tools that facilitate integration with robotics frameworks, making it an ideal choice for creating digital twins of humanoid robots.

## Unity Robotics Packages and Tools

Unity provides several official packages specifically designed for robotics applications:

### Unity Robotics Hub
The Unity Robotics Hub is a package manager and launcher that provides easy access to robotics-specific tools, samples, and tutorials. It streamlines the setup process for robotics projects and ensures compatibility between different Unity robotics packages.

### ROS# (ROS Sharp)
ROS# enables communication between Unity and ROS 2 through the Robot Operating System. This package allows Unity to send and receive messages from ROS 2 nodes, making it possible to integrate Unity-based visualizations with the broader ROS 2 ecosystem.

### Unity Perception Package
The Perception package provides tools for generating synthetic training data, including:
- Synthetic image generation with ground truth annotations
- Sensor simulation (cameras, LiDAR, etc.)
- Domain randomization for robust AI model training

### ML-Agents Toolkit
Unity's ML-Agents toolkit allows for training AI agents directly within Unity environments, making it possible to develop and test humanoid robot behaviors in realistic visual scenarios.

## Creating Realistic Environments for Humanoid Robots

Designing effective visual environments for humanoid robots requires careful consideration of both aesthetics and functionality. The environment should be realistic enough to provide meaningful training data while maintaining performance for real-time simulation.

### Environmental Design Principles

1. **Scale Appropriateness**: Environments should be appropriately scaled for humanoid robots, typically human-sized spaces with corresponding furniture and obstacles.

2. **Lighting Conditions**: Realistic lighting that matches the intended deployment environment, including natural light from windows, artificial lighting, and shadows.

3. **Material Properties**: Accurate representation of surface materials with appropriate reflectance, roughness, and other physical properties that affect visual perception.

4. **Interactive Elements**: Objects that the humanoid robot can interact with, such as doors, buttons, furniture, and tools.

### Example Environment Setup

Here's how to create a basic indoor environment for humanoid robot simulation in Unity:

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class HumanoidEnvironment : MonoBehaviour
{
    [Header("Environment Configuration")]
    public float roomWidth = 10f;
    public float roomDepth = 8f;
    public float roomHeight = 3f;

    [Header("Robot Spawn Point")]
    public Vector3 robotSpawnPosition = new Vector3(0f, 0f, 0f);

    void Start()
    {
        // Create room boundaries
        CreateRoomBoundaries();

        // Add furniture and obstacles
        CreateFurniture();

        // Set up lighting
        SetupLighting();
    }

    void CreateRoomBoundaries()
    {
        // Create floor
        GameObject floor = GameObject.CreatePrimitive(PrimitiveType.Cube);
        floor.transform.position = new Vector3(0f, -roomHeight/2f, 0f);
        floor.transform.localScale = new Vector3(roomWidth, roomHeight, roomDepth);
        floor.GetComponent<Renderer>().material.color = Color.gray;

        // Create walls
        CreateWall(Vector3.forward * (roomDepth/2f), new Vector3(roomWidth, roomHeight, 0.2f));
        CreateWall(Vector3.forward * (-roomDepth/2f), new Vector3(roomWidth, roomHeight, 0.2f));
        CreateWall(Vector3.right * (roomWidth/2f), new Vector3(0.2f, roomHeight, roomDepth));
        CreateWall(Vector3.right * (-roomWidth/2f), new Vector3(0.2f, roomHeight, roomDepth));
    }

    GameObject CreateWall(Vector3 position, Vector3 scale)
    {
        GameObject wall = GameObject.CreatePrimitive(PrimitiveType.Cube);
        wall.transform.position = position;
        wall.transform.localScale = scale;
        wall.GetComponent<Renderer>().material.color = Color.white;
        return wall;
    }

    void CreateFurniture()
    {
        // Add a table
        GameObject table = GameObject.CreatePrimitive(PrimitiveType.Cube);
        table.transform.position = new Vector3(2f, 0.5f, 1f);
        table.transform.localScale = new Vector3(1.5f, 1f, 0.8f);
        table.GetComponent<Renderer>().material.color = Color.brown;

        // Add a chair
        GameObject chair = GameObject.CreatePrimitive(PrimitiveType.Cube);
        chair.transform.position = new Vector3(2f, 0.25f, -1f);
        chair.transform.localScale = new Vector3(0.5f, 0.5f, 0.5f);
        chair.GetComponent<Renderer>().material.color = Color.blue;
    }

    void SetupLighting()
    {
        // Create directional light (sun)
        GameObject sunLight = new GameObject("Sun");
        sunLight.AddComponent<Light>();
        sunLight.GetComponent<Light>().type = LightType.Directional;
        sunLight.transform.rotation = Quaternion.Euler(50f, -30f, 0f);
        sunLight.GetComponent<Light>().color = Color.white;
        sunLight.GetComponent<Light>().intensity = 1f;
    }
}
```

## Material and Shader Development for Robots

Creating realistic visual representations of humanoid robots requires careful attention to material properties and shader development. Materials define how light interacts with robot surfaces, affecting the visual appearance and realism of the digital twin.

### Material Properties for Robot Components

Different robot components require different material properties:

1. **Metallic Surfaces**: Shiny, reflective surfaces like metal joints and actuators
2. **Plastic Components**: Matte or semi-gloss surfaces for body panels and covers
3. **Rubber/Polymer**: Soft, slightly textured surfaces for grippers and feet
4. **LED Indicators**: Emissive materials for status lights and sensors

### Example Material Setup for Robot Components

```csharp
using UnityEngine;

public class RobotMaterialSetup : MonoBehaviour
{
    [Header("Material References")]
    public Material metallicMaterial;
    public Material plasticMaterial;
    public Material rubberMaterial;
    public Material ledMaterial;

    void Start()
    {
        SetupRobotMaterials();
    }

    void SetupRobotMaterials()
    {
        // Metallic material for joints and actuators
        if (metallicMaterial != null)
        {
            metallicMaterial.SetColor("_Color", new Color(0.7f, 0.7f, 0.8f));
            metallicMaterial.SetFloat("_Metallic", 0.9f);
            metallicMaterial.SetFloat("_Smoothness", 0.8f);
        }

        // Plastic material for body panels
        if (plasticMaterial != null)
        {
            plasticMaterial.SetColor("_Color", new Color(0.3f, 0.3f, 0.3f));
            plasticMaterial.SetFloat("_Metallic", 0.1f);
            plasticMaterial.SetFloat("_Smoothness", 0.3f);
        }

        // Rubber material for feet and grippers
        if (rubberMaterial != null)
        {
            rubberMaterial.SetColor("_Color", new Color(0.1f, 0.1f, 0.1f));
            rubberMaterial.SetFloat("_Metallic", 0.0f);
            rubberMaterial.SetFloat("_Smoothness", 0.1f);
        }

        // LED material for status indicators
        if (ledMaterial != null)
        {
            ledMaterial.SetColor("_EmissionColor", Color.green);
            ledMaterial.EnableKeyword("_EMISSION");
        }
    }

    // Method to change LED color based on robot state
    public void UpdateLEDState(string state)
    {
        if (ledMaterial != null)
        {
            switch (state)
            {
                case "ready":
                    ledMaterial.SetColor("_EmissionColor", Color.green);
                    break;
                case "busy":
                    ledMaterial.SetColor("_EmissionColor", Color.yellow);
                    break;
                case "error":
                    ledMaterial.SetColor("_EmissionColor", Color.red);
                    break;
                default:
                    ledMaterial.SetColor("_EmissionColor", Color.black);
                    break;
            }
        }
    }
}
```

## Lighting Systems and Shadow Implementation

Proper lighting is crucial for creating realistic visual environments that match real-world conditions. Unity offers several lighting options that can be used to simulate different environmental conditions.

### Types of Lighting in Unity

1. **Directional Lights**: Simulate sunlight or other distant light sources
2. **Point Lights**: Omnidirectional lights like bulbs or LEDs
3. **Spot Lights**: Focused lighting like flashlights or stage lights
4. **Area Lights**: Rectangular or disc-shaped lights for soft shadows

### Dynamic Lighting for Robot Environments

```csharp
using UnityEngine;

public class DynamicLightingController : MonoBehaviour
{
    [Header("Light Configuration")]
    public Light mainLight;
    public AnimationCurve intensityCurve;
    public Gradient colorGradient;

    [Header("Time Parameters")]
    public float dayNightCycleDuration = 120f; // seconds
    private float currentTime = 0f;

    void Start()
    {
        if (mainLight == null)
            mainLight = GetComponent<Light>();
    }

    void Update()
    {
        UpdateLightingBasedOnTime();
    }

    void UpdateLightingBasedOnTime()
    {
        currentTime += Time.deltaTime;
        float normalizedTime = (currentTime % dayNightCycleDuration) / dayNightCycleDuration;

        // Update light intensity based on time of day
        float intensity = intensityCurve.Evaluate(normalizedTime);
        mainLight.intensity = Mathf.Lerp(0.2f, 1.0f, intensity);

        // Update light color based on time of day
        Color lightColor = colorGradient.Evaluate(normalizedTime);
        mainLight.color = lightColor;
    }

    // Method to add dynamic lighting effects
    public void AddLightingEffect(string effectType)
    {
        switch (effectType)
        {
            case "flicker":
                StartCoroutine(FlickerLight());
                break;
            case "pulse":
                StartCoroutine(PulseLight());
                break;
        }
    }

    System.Collections.IEnumerator FlickerLight()
    {
        float originalIntensity = mainLight.intensity;
        for (int i = 0; i < 5; i++)
        {
            mainLight.intensity = Random.Range(0.1f, originalIntensity);
            yield return new WaitForSeconds(Random.Range(0.05f, 0.2f));
        }
        mainLight.intensity = originalIntensity;
    }

    System.Collections.IEnumerator PulseLight()
    {
        float originalIntensity = mainLight.intensity;
        float targetIntensity = originalIntensity * 1.5f;

        for (float t = 0; t < 2f; t += Time.deltaTime)
        {
            float progress = Mathf.PingPong(t, 1f);
            mainLight.intensity = Mathf.Lerp(originalIntensity, targetIntensity, progress);
            yield return null;
        }
        mainLight.intensity = originalIntensity;
    }
}
```

## Camera Systems and Visualization Techniques

Camera systems in Unity are essential for providing different perspectives of the humanoid robot and its environment. Different camera setups serve different purposes in the digital twin.

### Common Camera Setups for Robotics

1. **Robot Perspective Camera**: First-person view from the robot's sensors
2. **Third-Person Camera**: Follows the robot from a distance
3. **Overhead Camera**: Top-down view for navigation and planning
4. **Fixed Cameras**: Stationary cameras for monitoring specific areas

### Robot Perspective Camera Implementation

```csharp
using UnityEngine;

public class RobotCameraController : MonoBehaviour
{
    [Header("Camera Configuration")]
    public Camera robotCamera;
    public Transform headJoint; // Reference to robot's head joint
    public float fieldOfView = 60f;
    public float nearClipPlane = 0.1f;
    public float farClipPlane = 100f;

    [Header("Visualization Options")]
    public bool showDepthOverlay = false;
    public bool showSegmentation = false;
    public Material depthMaterial;
    public Material segmentationMaterial;

    void Start()
    {
        if (robotCamera == null)
            robotCamera = GetComponent<Camera>();

        SetupCameraProperties();
    }

    void Update()
    {
        UpdateCameraPosition();
        HandleVisualizationModes();
    }

    void SetupCameraProperties()
    {
        robotCamera.fieldOfView = fieldOfView;
        robotCamera.nearClipPlane = nearClipPlane;
        robotCamera.farClipPlane = farClipPlane;
    }

    void UpdateCameraPosition()
    {
        if (headJoint != null)
        {
            transform.position = headJoint.position;
            transform.rotation = headJoint.rotation;
        }
    }

    void HandleVisualizationModes()
    {
        if (showDepthOverlay && depthMaterial != null)
        {
            robotCamera.SetReplacementShader(depthMaterial.shader, "RenderType");
        }
        else if (showSegmentation && segmentationMaterial != null)
        {
            robotCamera.SetReplacementShader(segmentationMaterial.shader, "RenderType");
        }
        else
        {
            robotCamera.ResetReplacementShader();
        }
    }

    // Method to capture camera data for ROS publishing
    public Texture2D CaptureCameraImage()
    {
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = robotCamera.targetTexture;

        robotCamera.Render();

        Texture2D image = new Texture2D(robotCamera.targetTexture.width, robotCamera.targetTexture.height);
        image.ReadPixels(new Rect(0, 0, robotCamera.targetTexture.width, robotCamera.targetTexture.height), 0, 0);
        image.Apply();

        RenderTexture.active = currentRT;

        return image;
    }
}
```

## Human-Robot Interaction Scene Design

Designing effective human-robot interaction scenes requires understanding both human and robot capabilities and limitations. These scenes should facilitate intuitive interaction while providing clear feedback about robot states and intentions.

### Key Elements of HRI Scenes

1. **Visual Feedback Systems**: Clear indicators of robot state, intentions, and capabilities
2. **Interaction Zones**: Clearly defined areas where humans can safely interact
3. **Communication Interfaces**: Visual elements that facilitate human-robot communication
4. **Safety Boundaries**: Visual indicators of robot workspace and safety perimeters

### Example HRI Scene Controller

```csharp
using UnityEngine;
using System.Collections.Generic;

public class HumanRobotInteractionScene : MonoBehaviour
{
    [Header("Interaction Configuration")]
    public GameObject humanoidRobot;
    public List<GameObject> interactionPoints = new List<GameObject>();
    public Color availableColor = Color.green;
    public Color busyColor = Color.yellow;
    public Color unavailableColor = Color.red;

    [Header("Visual Feedback")]
    public GameObject interactionIndicator;
    public GameObject safetyBoundary;
    public LineRenderer interactionPath;

    private Dictionary<GameObject, GameObject> interactionVisuals = new Dictionary<GameObject, GameObject>();

    void Start()
    {
        SetupInteractionVisuals();
    }

    void SetupInteractionVisuals()
    {
        foreach (GameObject point in interactionPoints)
        {
            // Create visual indicator for each interaction point
            GameObject indicator = GameObject.CreatePrimitive(PrimitiveType.Sphere);
            indicator.transform.SetParent(point.transform);
            indicator.transform.localPosition = Vector3.zero;
            indicator.transform.localScale = Vector3.one * 0.2f;
            indicator.GetComponent<Renderer>().material.color = availableColor;

            interactionVisuals[point] = indicator;
        }

        // Set up safety boundary visualization
        if (safetyBoundary != null)
        {
            safetyBoundary.GetComponent<Renderer>().material.color = new Color(1f, 0f, 0f, 0.3f);
            safetyBoundary.GetComponent<Renderer>().material.SetFloat("_Mode", 3f); // Transparent
            safetyBoundary.GetComponent<Renderer>().material.SetInt("_SrcBlend", (int)UnityEngine.Rendering.BlendMode.SrcAlpha);
            safetyBoundary.GetComponent<Renderer>().material.SetInt("_DstBlend", (int)UnityEngine.Rendering.BlendMode.OneMinusSrcAlpha);
        }
    }

    public void UpdateInteractionPointStatus(GameObject interactionPoint, string status)
    {
        if (interactionVisuals.ContainsKey(interactionPoint))
        {
            Color newColor = unavailableColor;

            switch (status)
            {
                case "available":
                    newColor = availableColor;
                    break;
                case "busy":
                    newColor = busyColor;
                    break;
                case "unavailable":
                    newColor = unavailableColor;
                    break;
            }

            interactionVisuals[interactionPoint].GetComponent<Renderer>().material.color = newColor;
        }
    }

    public void HighlightInteractionPath(GameObject startPoint, GameObject endPoint)
    {
        if (interactionPath != null)
        {
            interactionPath.enabled = true;
            interactionPath.SetPosition(0, startPoint.transform.position);
            interactionPath.SetPosition(1, endPoint.transform.position);
        }
    }

    public void ClearInteractionPath()
    {
        if (interactionPath != null)
        {
            interactionPath.enabled = false;
        }
    }
}
```

## VR/AR Integration Possibilities

Virtual and Augmented Reality technologies can enhance digital twin experiences by providing immersive interfaces for human-robot interaction and robot monitoring.

### VR Integration for Robot Teleoperation

```csharp
#if UNITY_STANDALONE_WIN || UNITY_EDITOR
using UnityEngine.XR;
using UnityEngine.XR.Interaction.Toolkit;
#endif

public class VRRobotController : MonoBehaviour
{
    [Header("VR Configuration")]
    public Transform vrHeadset;
    public XRNode vrControllerLeft;
    public XRNode vrControllerRight;

    [Header("Robot Control Mapping")]
    public Transform robotHead;
    public Transform robotLeftHand;
    public Transform robotRightHand;

    void Update()
    {
        UpdateRobotFromVR();
    }

    void UpdateRobotFromVR()
    {
        if (vrHeadset != null && robotHead != null)
        {
            // Mirror head movement from VR to robot
            robotHead.position = vrHeadset.position;
            robotHead.rotation = vrHeadset.rotation;
        }

        // Get controller positions and update robot hands
        Vector3 leftControllerPos = InputTracking.GetLocalPosition(vrControllerLeft);
        Quaternion leftControllerRot = InputTracking.GetLocalRotation(vrControllerLeft);

        Vector3 rightControllerPos = InputTracking.GetLocalPosition(vrControllerRight);
        Quaternion rightControllerRot = InputTracking.GetLocalRotation(vrControllerRight);

        if (robotLeftHand != null)
        {
            robotLeftHand.position = leftControllerPos;
            robotLeftHand.rotation = leftControllerRot;
        }

        if (robotRightHand != null)
        {
            robotRightHand.position = rightControllerPos;
            robotRightHand.rotation = rightControllerRot;
        }
    }
}
```

## Performance Optimization for Real-Time Rendering

Real-time rendering of complex humanoid robot models and environments requires careful optimization to maintain smooth performance.

### Optimization Techniques

1. **Level of Detail (LOD)**: Use simplified models when the robot is far from the camera
2. **Occlusion Culling**: Don't render objects that are not visible
3. **Texture Atlasing**: Combine multiple textures into single atlases
4. **Shader Optimization**: Use efficient shaders with minimal overdraw
5. **Batching**: Combine similar objects for efficient rendering

### LOD System for Robot Models

```csharp
using UnityEngine;

public class RobotLODController : MonoBehaviour
{
    [Header("LOD Configuration")]
    public Transform[] lodLevels; // Array of transforms for different LOD levels
    public float[] lodDistances; // Distance thresholds for each LOD level
    public Camera referenceCamera; // Camera to calculate distance from

    private int currentLOD = 0;

    void Start()
    {
        if (referenceCamera == null)
            referenceCamera = Camera.main;

        UpdateLOD();
    }

    void Update()
    {
        UpdateLOD();
    }

    void UpdateLOD()
    {
        if (referenceCamera == null) return;

        float distance = Vector3.Distance(transform.position, referenceCamera.transform.position);

        int newLOD = 0;
        for (int i = 0; i < lodDistances.Length; i++)
        {
            if (distance > lodDistances[i])
                newLOD = i + 1;
        }

        // Clamp to valid range
        newLOD = Mathf.Min(newLOD, lodLevels.Length - 1);

        if (newLOD != currentLOD)
        {
            SetLOD(newLOD);
            currentLOD = newLOD;
        }
    }

    void SetLOD(int lodIndex)
    {
        for (int i = 0; i < lodLevels.Length; i++)
        {
            if (lodLevels[i] != null)
            {
                lodLevels[i].gameObject.SetActive(i == lodIndex);
            }
        }
    }
}
```

## Asset Importing and Model Preparation

Proper preparation of robot models for Unity is crucial for achieving good visual quality and performance.

### Model Import Best Practices

1. **Scale Consistency**: Ensure models are imported at the correct scale (typically meters)
2. **Center of Mass**: Position the model origin at the appropriate location
3. **Texture Resolution**: Use appropriate texture sizes for the intended viewing distance
4. **Polygon Count**: Balance detail with performance requirements
5. **Animation Rigging**: Prepare models for animation if movement is required

## Exercises

### Exercise 1: Identify Visual Components
Examine the following Unity C# script and identify the visual components being configured:

```csharp
public class RobotVisualizer : MonoBehaviour
{
    public Material bodyMaterial;
    public Material jointMaterial;
    public Light statusLight;

    void Start()
    {
        bodyMaterial.SetColor("_Color", Color.gray);
        bodyMaterial.SetFloat("_Metallic", 0.2f);
        bodyMaterial.SetFloat("_Smoothness", 0.3f);

        jointMaterial.SetColor("_Color", Color.blue);
        jointMaterial.SetFloat("_Metallic", 0.8f);
        jointMaterial.SetFloat("_Smoothness", 0.7f);

        statusLight.color = Color.green;
        statusLight.intensity = 1.0f;
    }
}
```

What visual properties does this script configure for the robot? How would changing the metallic values affect the appearance?

### Exercise 2: Environment Design
Design a Unity scene for a humanoid robot that includes:
- A living room environment with furniture
- Appropriate lighting for indoor scenarios
- At least 3 different material types for robot components
- A camera system for robot perspective view

## Summary

This chapter explored the creation of high-fidelity visual environments using Unity for digital twins of humanoid robots. We covered Unity robotics packages, environmental design principles, material and shader development, lighting systems, camera techniques, human-robot interaction scene design, VR/AR integration possibilities, and performance optimization strategies.

Visual environments play a crucial role in digital twins by providing the rich sensory input needed for AI training and validation. Unity's powerful rendering capabilities, combined with its robotics packages, make it an excellent choice for creating visually compelling digital twins that can be used for various robotics applications.

In the next chapter, we'll examine how to simulate various sensors in digital environments, completing the sensor simulation component of our digital twin system.