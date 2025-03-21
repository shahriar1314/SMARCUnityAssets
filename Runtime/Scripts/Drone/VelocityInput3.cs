using System;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;


// Directives for publishing messages
using Unity.Robotics.Core; // Clock
using Unity.Robotics.ROSTCPConnector;
using StdMessages = RosMessageTypes.Std;
using VehicleComponents.Actuators;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using DefaultNamespace.LookUpTable;

public class VelocityInput3 : MonoBehaviour
{
    [Header("Basics")] 
    [Tooltip("Baselink of drone")]
    public GameObject BaseLink;
    public GameObject BaseLinkSAM;

    public DroneController.DroneController droneController;

    private Vector<double> initialPosition;
    private Vector<double> initialPositionSAM;
    private Vector<double> currentPosition;
    private int step = 0; // Step to track movement phase
    public double sideLength = 3.0; // Length of each side of the square
    public double speed = 1.0; // Speed of movement

    void Start()
    {
        if (droneController != null)
        {
            // Convert the initial position to ENU and store it
            initialPosition = BaseLink.transform.position.To<ENU>().ToDense();
            initialPositionSAM = BaseLinkSAM.transform.position.To<ENU>().ToDense();
        }
    }

    void Update()
    {
        if (droneController == null) return;

        // Track current position and convert it to ENU
        currentPosition = droneController.BaseLink.transform.position.To<ENU>().ToDense();

        Debug.Log($"Initial Position SAM (Center): x = {initialPositionSAM[0]}, y = {initialPositionSAM[1]}, z = {initialPositionSAM[2]}");
        Debug.Log($"Current Position: x = {currentPosition[0]}, y = {currentPosition[1]}, z = {currentPosition[2]}");

        double halfSide = sideLength / 2.0;

        switch (step)
        {
            case 0: // Move right
                droneController.SetTargetVelocity(new Vector3((float)speed, 0, 0));
                Debug.Log("GOING RIGHT");
                if (currentPosition[0] >= initialPositionSAM[0] + halfSide)
                {
                    step++; 
                }
                break;

            case 1: // Move forward
                droneController.SetTargetVelocity(new Vector3(0, 0, (float)speed));
                Debug.Log("GOING UP");
                if (currentPosition[1] >= initialPositionSAM[1] + halfSide)
                {
                    step++; 
                }
                break;

            default:
                break;

            case 2: // Move left
                droneController.SetTargetVelocity(new Vector3((float)-speed, 0, 0));
                Debug.Log("GOING LEFT");
                if (currentPosition[0] <= initialPositionSAM[0] - halfSide)
                {
                    step++; 
                }
                break;

            case 3: // Move back
                droneController.SetTargetVelocity(new Vector3(0, 0, (float)-speed));
                Debug.Log("GOING DOWN");
                if (currentPosition[1] <= initialPositionSAM[1] - halfSide)
                {
                    step = 0; 
                }
                break;
        }
        
    }
}
