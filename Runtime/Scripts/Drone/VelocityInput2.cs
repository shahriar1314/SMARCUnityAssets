using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MathNet.Numerics.LinearAlgebra;
using MathNet.Numerics.LinearAlgebra.Double;

using DefaultNamespace; // ResetArticulationBody() extension
using Unity.Robotics.Core; // Clock
using Unity.Robotics.ROSTCPConnector;
using StdMessages = RosMessageTypes.Std;
using VehicleComponents.Actuators;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using DefaultNamespace.LookUpTable;

public class VelocityInput2 : MonoBehaviour
{
    [Header("Basics")] 
    [Tooltip("Baselink of drone")]
    public GameObject BaseLink;
    public GameObject BaseLinkSAM;

    [Tooltip("Transform to Teleport(Drone Baselink)")]
    public Transform Target; // Transform to Teleport

    [Tooltip("Drone Actuator or Winch System (Rigid Body)")]
    public Transform DroneActuator;

    public DroneController.DroneController droneController;

    ArticulationBody[] ABparts;
    Rigidbody[] RBparts;

    private Vector<double> initialPosition;
    private Vector<double> initialPositionSAM;
    private Vector<double> currentPosition;
    public double PathLenght = 6.0;
    public double speed = 1.0;

    private int ResetInProgress = 0; 
    private int ResetFlag = 0;

    void Start()
    {
        if (droneController != null)
        {
            initialPosition = BaseLink.transform.position.To<ENU>().ToDense();
            initialPositionSAM = BaseLinkSAM.transform.position.To<ENU>().ToDense();

            ABparts = Target.gameObject.GetComponentsInChildren<ArticulationBody>();
            RBparts = DroneActuator.gameObject.GetComponentsInChildren<Rigidbody>();
        }
    }

    void FixedUpdate()
    {
        if (droneController == null) return;

        currentPosition = droneController.BaseLink.transform.position.To<ENU>().ToDense();
        Debug.Log($"Current Position: x = {currentPosition[0]}, y = {currentPosition[1]}, z = {currentPosition[2]}");
        Debug.Log("Reset Flag: " + ResetFlag);

        if (ResetInProgress == 1)
        {
            if (ResetFlag == 1) ResetPosition();
        }
        else
        {
            if (currentPosition[0] < initialPositionSAM[0] + PathLenght)
            {
                droneController.SetTargetVelocity(new Vector3((float)speed, 0, 0));
                Debug.Log("Velocity is set");
            }
            else
            {
                droneController.SetTargetVelocity(new Vector3(0, 0, 0));
                ResetFlag = 1;
                ResetInProgress = 1;
            }
        }
    }

    void ResetPosition()
    {
        var NewPosition = ENU.ConvertToRUF(new Vector3(
            (float)initialPosition[0],
            (float)initialPosition[1],
            (float)initialPosition[2]
        ));

        var NewOrientation = Quaternion.identity;

        if (Target.TryGetComponent(out ArticulationBody targetAb))
        {
            if (!targetAb.isRoot) return;

            targetAb.immovable = true;

            Collider[] colliders = Target.GetComponentsInChildren<Collider>();
            foreach (var col in colliders)
            {
                col.enabled = false;
            }


            droneController.SetTargetVelocity(Vector3.zero);

            targetAb.TeleportRoot(NewPosition, NewOrientation);
            targetAb.linearVelocity = Vector3.zero;
            targetAb.angularVelocity = Vector3.zero;

            foreach (var ab in ABparts)
            {
                ab.linearVelocity = Vector3.zero;
                ab.angularVelocity = Vector3.zero;
                ab.ResetArticulationBody();
            }

            foreach (var rb in RBparts)
            {
                rb.transform.position = NewPosition;
                rb.rotation = Quaternion.Euler(0f, 0f, 0f);
                rb.linearVelocity = Vector3.zero;
                rb.angularVelocity = Vector3.zero;
            }

            StartCoroutine(ReEnablePhysics(targetAb));
        }
        else
        {
            Debug.Log("Target is not an Articulation Body");
        }

        ResetFlag = 0;
        Debug.Log("RESET IS DONE");
    }

    IEnumerator ReEnablePhysics(ArticulationBody targetAb)
    {
        yield return new WaitForFixedUpdate();
        yield return new WaitForFixedUpdate();

        Collider[] colliders = Target.GetComponentsInChildren<Collider>();
        foreach (var col in colliders)
        {
            col.enabled = true;
        }


        targetAb.immovable = false;
        ResetInProgress = 0;

        Debug.Log("Physics re-enabled and drone is now movable");
    }
}
