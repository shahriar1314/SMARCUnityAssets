using System;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

using DefaultNamespace; // ResetArticulationBody() extension


// Directives for publishing messages
using Unity.Robotics.Core; // Clock
using Unity.Robotics.ROSTCPConnector;
using StdMessages = RosMessageTypes.Std;
using VehicleComponents.Actuators;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using DefaultNamespace.LookUpTable;
using GluonGui.WorkspaceWindow.Views.WorkspaceExplorer.Explorer.Operations;

public class VelocityInput7 : MonoBehaviour
{
    [Header("Basics")] 
    [Tooltip("Baselink of drone")]
    public GameObject BaseLink;
    public GameObject BaseLinkSAM;

    public Transform Drone; 

    public DroneController.DroneController droneController;

    private Vector<double> initialPosition;
    private Vector<double> initialPositionSAM;
    private Vector<double> currentPosition;
    private int reset_in_progress=1;
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

    void FixedUpdate()
    {
        if (droneController == null) return;

        // Track current position and convert it to ENU
        currentPosition = droneController.BaseLink.transform.position.To<ENU>().ToDense();

        // Use the initial position from BaseLink and convert it properly
        var NewPosition = ENU.ConvertToRUF(
            new Vector3(
                (float)initialPositionSAM[0],  
                (float)initialPositionSAM[1],
                (float)initialPositionSAM[2]+5f//keeping the position same as the initial position of the drone 
            ));

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
                    step=0;
                    if (reset_in_progress!=0)
                    {
                        RelocateArticulationBody(Drone.GetComponent<ArticulationBody>(), NewPosition, Quaternion.identity) ;
                        reset_in_progress+=1;
                        reset_in_progress = reset_in_progress%3; 
                    }
                    
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


    private void RelocateArticulationBody(ArticulationBody articulationBody, Vector3 position, Quaternion rotation)
    {
        articulationBody.immovable = true;

        articulationBody.TeleportRoot(position, rotation);
        
        articulationBody.linearVelocity = Vector3.zero;
        articulationBody.angularVelocity = Vector3.zero;

        foreach (ArticulationBody child in articulationBody.GetComponentsInChildren<ArticulationBody>())
        {
            ArticulationReducedSpace zeroPos = new ArticulationReducedSpace(child.dofCount);
            child.jointPosition = zeroPos;

            ArticulationReducedSpace zeroVel = new ArticulationReducedSpace(child.dofCount);
            child.jointVelocity = zeroVel;
            // child.jointAcceleration = zeroVel;
            child.jointForce = zeroVel;

            child.linearVelocity = Vector3.zero;
            child.angularVelocity = Vector3.zero;
            child.ResetArticulationBody();
        }

        foreach (Rigidbody rb in articulationBody.GetComponentsInChildren<Rigidbody>())
        {
            rb.linearVelocity = Vector3.zero;
            rb.angularVelocity = Vector3.zero;
        }

        articulationBody.immovable = false;
    }
}
