using UnityEngine;

public class VelocityInput : MonoBehaviour
{
    public DroneController.DroneController droneController;

    void Start()
    {
        if (droneController != null)
        {
            droneController.SetTargetVelocity(new Vector3(1.0f, 0.0f, 0.0f)); // Example velocity
        }
    }
}
