using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;

public class UFO4 : MonoBehaviour
{
    public float alpha = 0.25f;
    public Transform AUVTransform;
    public bool debugMode = false; 
    public float desired_height = 7f;
    public float desired_displacement = 5f;

    public bool useExternalInput = false; // Flag to control position update source

    void Start()
    {
        if (AUVTransform == null)
        {
            Debug.LogWarning("No AUVTransform set for UFO sensor. Disabling.");
            enabled = false;
        }
    }

    void FixedUpdate()
    {
        if (!useExternalInput) // Only update position if external input is not used
        {
            float t = Time.time;
            float t_dash = t % (float)Math.Floor(2f * (float)desired_displacement / (float)alpha);
            transform.position = new Vector3(AUVTransform.position.x - desired_displacement + alpha * t_dash, desired_height, AUVTransform.position.z);
        }
    }

    public void SetPosition(Vector3 newPosition)
    {
        useExternalInput = true; // Disable FixedUpdate position control
        transform.position = newPosition;
        // if(debugMode) Debug.Log($"X:{newPosition.x} Y:{newPosition.y} Z:{newPosition.z} ");
    }
}
