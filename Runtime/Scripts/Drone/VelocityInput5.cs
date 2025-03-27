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

public class VelocityInput5 : MonoBehaviour
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
    private int step = 0; // Step to track movement phase
    public double sideLength = 6.0; // Length of each side of the square
    public double speed = 1.0; // Speed of movement
    int immovableStage = 2; // to make the AB immovable, 0=preparing, 1=immovable, 2=default/nothing

    private double ResetFrameNo =100;    
    private double FrameCounter      =100; // to make a delay 


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
        // Debug.Log($"Current Position: x = {currentPosition[0]}, y = {currentPosition[1]}, z = {currentPosition[2]}");

        double halfSide = sideLength / 2.0;   // just to make a square trajectory 

        // For resetting the drone this if logic has to be inside the FixedUpdate() 
        // everything else will be inside 'else' 
        // Also do not forget to copy the ResetPosition(0) method 

        
        
        
        // When I need to reset the Drone, The Frame Counter will be set to ResetFrameNo (for ex 100)
        // Basically, for 100 FixedUpdate I will 
        if (FrameCounter>=0)
        {
            
            if(FrameCounter==ResetFrameNo)
            {
                FrameCounter--;
                
                
                ResetPosition();
                switch(immovableStage)
                {
                    case 0:
                        immovableStage = 1;
                        break;
                    case 1:
                        if(Target.TryGetComponent(out ArticulationBody targetAb))
                        {
                            if(!targetAb.isRoot) return;
                            // targetAb.immovable = false;
                        }
                        immovableStage = 2;
                        break; 
                    default:
                        break;
                }

                Debug.Log("IMMOVABLE STAGE WORKED");
                return;
            }



            else
            {
                FrameCounter --; 


                // Use the initial position from BaseLink and convert it properly
                var NewPosition = ENU.ConvertToRUF(
                    new Vector3(
                        (float)initialPositionSAM[0],  
                        (float)initialPositionSAM[1],
                        (float)initialPositionSAM[2]+5f//keeping the position same as the initial position of the drone 
                    ));
                // Use a default orientation (identity quaternion)
                var NewOrientation = Quaternion.identity;
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
                return;
            }


        }

        else
        {
            switch (step)
            {
                case 0: // Move right
                    droneController.SetTargetVelocity(new Vector3((float)speed, 0, 0));
                    Debug.Log("GOING RIGHT");
                    if (currentPosition[0] >= initialPositionSAM[0] + halfSide)
                    {
                        step=0; 
                    }
                    break;

                // case 1: // Move forward
                //     droneController.SetTargetVelocity(new Vector3(0, 0, (float)speed));
                //     Debug.Log("GOING UP");
                //     if (currentPosition[1] >= initialPositionSAM[1] + halfSide)
                //     {
                //         step=0;
                //         FrameCounter=ResetFrameNo;
                //     }
                //     break;

                // default:
                //     break;

                // case 2: // Move left
                //     droneController.SetTargetVelocity(new Vector3((float)-speed, 0, 0));
                //     Debug.Log("GOING LEFT");
                //     if (currentPosition[0] <= initialPositionSAM[0] )
                //     {
                //         step=0;
                //         FrameCounter=ResetFrameNo;
                //     }
                //     break;

                // case 3: // Move back
                //     droneController.SetTargetVelocity(new Vector3(0, 0, (float)-speed));
                //     Debug.Log("GOING DOWN");
                //     if (currentPosition[1] <= initialPositionSAM[1] - halfSide)
                //     {
                //         step = 0; 
                //     }
                //     break;
            }
        }
        
    }






    void ResetPosition()
    {   
        
        // Use the initial position from BaseLink and convert it properly
        var NewPosition = ENU.ConvertToRUF(
            new Vector3(
                (float)initialPositionSAM[0],  
                (float)initialPositionSAM[1],
                (float)initialPositionSAM[2]+5f //keeping the position same as the initial position of the drone 
            ));
         // Use a default orientation (identity quaternion)
        var NewOrientation = Quaternion.identity;


        if (Target.TryGetComponent(out ArticulationBody targetAb))
            {
                if (!targetAb.isRoot) return;
                // targetAb.immovable = true;
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

    }
}
