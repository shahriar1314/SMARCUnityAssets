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

public class VelocityInput4 : MonoBehaviour
{
    [Header("Basics")] 
    [Tooltip("Baselink of drone")]
    public GameObject BaseLink;
    public GameObject BaseLinkSAM;
    public Transform Target;
    ArticulationBody[] ABparts;

    int immovableStage = 2;

    public DroneController.DroneController droneController;

    private Vector<double> initialPosition;
    private Vector<double> initialPositionSAM;
    private Vector<double> currentPosition;
    private int ResetFlag = 1; // to track reset agent phase, 0=Reset, 1=No need to reset
    public double speed = 0.5; // Speed of movement

    void Start()
    {
        if (droneController != null)
        {
            // Convert the initial position to ENU and store it
            initialPosition = BaseLink.transform.position.To<ENU>().ToDense();
            initialPositionSAM = BaseLinkSAM.transform.position.To<ENU>().ToDense();

            ABparts = Target.gameObject.GetComponentsInChildren<ArticulationBody>();
        }

        ResetFlag = 1; 
    }

    void Update()
    {
        if (droneController == null) return;

        // Track current position and convert it to ENU
        currentPosition = BaseLink.transform.position.To<ENU>().ToDense();
        
        Debug.Log($"Current Position: x = {currentPosition[0]}, y = {currentPosition[1]}, z = {currentPosition[2]}");
        Debug.Log($"Initial Position: x = {initialPosition[0]}, y = {initialPosition[1]}, z = {initialPosition[2]}");


        if (ResetFlag == 0)
        {
            
            if(ResetFlag == 0 && immovableStage >=2)
            {
                ResetPosition();
                Debug.Log("NEED TO SET NEW POSITION");
            }
            


            switch(immovableStage)
            {
                case 0:
                    immovableStage = 1;
                    break;
                case 1:
                    if(Target.TryGetComponent(out ArticulationBody targetAb))
                    {
                        if(!targetAb.isRoot) return;
                        targetAb.immovable = false;
                    }
                    immovableStage = 2;
                    ResetFlag = 1;
                    break;
                default:
                    ResetFlag = 1;
                    break;
            }

            Debug.Log("IMMOVABLE STAGE WORKED");


        }

        else if (currentPosition[0] >= (initialPositionSAM[0] + 5.0) && ResetFlag!=0)
        {
            Debug.Log("CHANGING RESET FLAG");
            ResetFlag = 0;
            immovableStage = 2; 
        }

        else
        {
            droneController.SetTargetVelocity(new Vector3((float)speed, 0, 0));
            Debug.Log("VELOCITY IS SET");
            ResetFlag = 1;
        }
        
    }

    void ResetPosition()
    {   
        
        // Use the initial position from BaseLink and convert it properly
        var NewPosition = ENU.ConvertToRUF(
            new Vector3(
                (float)initialPosition[0],  
                (float)initialPosition[1]+2f,
                (float)initialPosition[2] //keeping the position same as the initial position of the drone 
            ));
         // Use a default orientation (identity quaternion)
        var NewOrientation = Quaternion.identity;


        if (Target.TryGetComponent(out ArticulationBody targetAb))
            {
                if (!targetAb.isRoot) return;
                //targetAb.immovable = true;
                immovableStage = 0;
                targetAb.TeleportRoot(NewPosition, NewOrientation);
                targetAb.linearVelocity = Vector3.zero;
                targetAb.angularVelocity = Vector3.zero;
                Debug.Log("NEW POSITION IS SET");
                // ResetFlag = 1;
            }
            else
            {
                Debug.Log("Target is not an Articulation Body");
            }


            foreach (var ab in ABparts)
            {
                ab.linearVelocity = Vector3.zero;
                ab.angularVelocity = Vector3.zero;
                ab.ResetArticulationBody();
            }

    }
}
