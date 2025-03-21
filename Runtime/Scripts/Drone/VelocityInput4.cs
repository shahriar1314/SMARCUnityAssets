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
    private int ResetFlag = 0; // ResetFlag to track reset agent phase
    private int step = 0; // Step to track movement phase
    public double sideLength = 6.0; // Length of each side of the square
    public double speed = 1.0; // Speed of movement

    void Start()
    {
        if (droneController != null)
        {
            // Convert the initial position to ENU and store it
            initialPosition = BaseLink.transform.position.To<ENU>().ToDense();
            initialPositionSAM = BaseLinkSAM.transform.position.To<ENU>().ToDense();

            ABparts = Target.gameObject.GetComponentsInChildren<ArticulationBody>();
        }

        ResetFlag = 0; 
    }

    void Update()
    {
        if (droneController == null) return;

        // Track current position and convert it to ENU
        currentPosition = droneController.BaseLink.transform.position.To<ENU>().ToDense();

        //Debug.Log($"Initial Position SAM (Center): x = {initialPositionSAM[0]}, y = {initialPositionSAM[1]}, z = {initialPositionSAM[2]}");
        Debug.Log($"Current Position: x = {currentPosition[0]}, y = {currentPosition[1]}, z = {currentPosition[2]}");

        double halfSide = sideLength / 2.0;

        if (ResetFlag == 0)
        {
            Debug.Log($"If Loop WHEN ****Reset Flag = {ResetFlag}");
            ResetPosition();
            ResetFlag++;

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
                    break;
                default:
                    break;
            }

        }

        else
        {   
            Debug.Log($"ELSE Loop WHEN ****Reset Flag = {ResetFlag}");
            switch (step)
            {
                case 0: // Move right
                    droneController.SetTargetVelocity(new Vector3((float)speed, 0, 0));
                    Debug.Log("GOING RIGHT");
                    if (currentPosition[0] >= initialPositionSAM[0] + halfSide)
                    {
                        //step++; 
                        ResetFlag=0;
                    }
                    break;
            }
        }

        
    }

    void ResetPosition()
    {   

        float halfSide = (float) (sideLength / 2.0);
        // Use the initial position from BaseLink and convert it properly
        var NewPosition = ENU.ConvertToRUF(
            new Vector3(
                (float)initialPositionSAM[0]-halfSide,  // Assuming initialPosition is a vector-like structure
                (float)initialPositionSAM[1],
                (float)initialPosition[2]+2f //keeping the height same as the initial position of the drone 
            ));
         // Use a default orientation (identity quaternion)
        var NewOrientation = Quaternion.identity;


        if (Target.TryGetComponent(out ArticulationBody targetAb))
            {
                if (!targetAb.isRoot) return;
                targetAb.immovable = true;
                immovableStage = 0;
                targetAb.TeleportRoot(NewPosition, NewOrientation);
                targetAb.linearVelocity = Vector3.zero;
                targetAb.angularVelocity = Vector3.zero;
                Debug.Log("NEW POSITION IS SET");
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
