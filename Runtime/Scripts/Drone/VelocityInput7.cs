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

public class VelocityInput7 : MonoBehaviour
{
    [Header("Basics")] 
    [Tooltip("Baselink of drone")]
    public GameObject BaseLink;
    public GameObject BaseLinkSAM;

    [Tooltip("Transform to Teleport(Drone Baselink)")]
    public Transform Target; // Transform to Teleport

    [Tooltip("Drone Actuator or Winch System (Rigid Body)")]
    public Transform DroneActuator;

    ArticulationBody[] ABparts;
    Rigidbody[] RBparts;

    public DroneController.DroneController droneController;

    private Vector<double> initialPosition;
    private Vector<double> initialPositionSAM;
    private Vector<double> currentPosition;
    public double PathLenght = 6.0; // Length of the straight path the drone will go
    public double speed = 1.0; // Speed of movement
    int immovableStage = 0; // to make the AB immovable, 0=preparing, 1=immovable, 2=default/nothing

    private int ResetInProgress = 0; 
    private int ResetFlag = 0; // 0=No need to Reset, 1=Reset


    void Start()
    {
        if (droneController != null)
        {
            // Convert the initial position to ENU and store it
            initialPosition = BaseLink.transform.position.To<ENU>().ToDense();
            initialPositionSAM = BaseLinkSAM.transform.position.To<ENU>().ToDense();

            ABparts = Target.gameObject.GetComponentsInChildren<ArticulationBody>();
            RBparts = DroneActuator.gameObject.GetComponentsInChildren<Rigidbody>();
        }
    }

    void FixedUpdate()
    {

        if (droneController == null) return;

        // Track current position and convert it to ENU
        currentPosition = droneController.BaseLink.transform.position.To<ENU>().ToDense();

        // Debug.Log($"Initial Position SAM (Center): x = {initialPositionSAM[0]}, y = {initialPositionSAM[1]}, z = {initialPositionSAM[2]}");
        Debug.Log($"Current Position: x = {currentPosition[0]}, y = {currentPosition[1]}, z = {currentPosition[2]}");
        Debug.Log("Reset Flag: "+ResetFlag);


        if (ResetInProgress==1)
        {          
            
            if(ResetFlag==1)  ResetPosition();

            // In unity it needs 2 frames to reset the AB object. 
            // (Tried to copy the same thing as mentioned in teleporter_sub.cs)

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
                    ResetInProgress = 0;
                    break;
            }

        }


        // otherwise it will go PathLength distance from the same
        else
        {
            if(currentPosition[0] < initialPositionSAM[0] + PathLenght)
            {
                droneController.SetTargetVelocity(new Vector3((float)speed, 0, 0));
                Debug.Log("Velocity is set");
            }
            
            else
            {
                droneController.SetTargetVelocity(new Vector3(0, 0, 0));
                ResetFlag=1;
                ResetInProgress=1;
            }
        }
        
    }






    void ResetPosition()
    {   
        
        // Use the initial position from BaseLink and convert it properly
        var NewPosition = ENU.ConvertToRUF(
            new Vector3(
                (float)initialPosition[0],  
                (float)initialPosition[1],
                (float)initialPosition[2] //keeping the position same as the initial position of the drone 
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
            targetAb.linearDamping = 0f;
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


        for (int i = 0; i < RBparts.Length; i++)
        {   
            // Reset position and rotation to initial values
            RBparts[i].transform.position = NewPosition; // initialPositionWinch[i];
            RBparts[i].rotation = Quaternion.Euler(0f, 0f, 0f);

            // Reset velocity to stop movement
            RBparts[i].linearVelocity = Vector3.zero;
            RBparts[i].angularVelocity = Vector3.zero;
        }    

        ResetFlag = 0;
        Debug.Log("RESET IS DONE");

    }
}
