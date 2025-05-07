using UnityEngine;
using System;
public class UFO5 : MonoBehaviour {
    [Header("Trajectory Parameters")]
    public Transform target;            
    public float heightOffset = 0.3f;      // ← how far above the water line
    public float totalTime = 35f;        
    public float k = 0.8f;
    public float kd_alpha = 0.4f;
    public float initialVelocity = 4.5f;
    public float alpha0 = Mathf.PI / 4; 

    private Vector3 p0, raisedTarget;    // starting position, raised target

    void Start()
    {
        p0 = transform.position;
        if (target == null)
        {
            Debug.LogError("Assign a target Transform for perching.");
            enabled = false;
        }
        
        raisedTarget = target.position + Vector3.up * heightOffset;

    }

    void Update() {
        // clamp time so we never overshoot
        float t = Mathf.Clamp(Time.time, 0f, totalTime);

        // 0) build your “raised” target position:
        

        // 1) compute tau0 based on raisedTarget
        float dist0 = Vector3.Distance(raisedTarget, p0);
        float tau0 = dist0 / initialVelocity;

        // 2) distance gap d(t)
        float frac = 1f - t / totalTime;
        float d_t = initialVelocity * tau0 * Mathf.Pow(frac, 1f / k);

        // 3) angular gap α(t)
        float alpha_t = alpha0 * Mathf.Pow(d_t / (initialVelocity * tau0), 1f / kd_alpha);

        // 4) composite matrices M1, M2
        float sin0 = Mathf.Sin(alpha0);
        float sin_t = Mathf.Sin(alpha_t);
        float M1 = (sin0 - sin_t) / sin0;
        float M2 = sin_t / sin0;

        // 5) displacement along local “up” (z-axis in your local frame)
        Vector3 M3 = new Vector3(0, 0, d_t * sin_t);

        // 6) assemble p(t) around the raised target
        Vector3 p_t = M1 * raisedTarget + M2 * p0 + M3;

        transform.position = p_t;
    }
}
