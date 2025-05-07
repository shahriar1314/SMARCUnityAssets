using UnityEngine;
using System;

public class UFO5 : MonoBehaviour {
    [Header("Trajectory Parameters")]
    public Transform target;            // ptd
    public float totalTime = 35f;        // td
    public float k = 0.6f;
    public float kd_alpha = 0.4f;
    public float initialVelocity = 4.5f;
    public float alpha0 = Mathf.PI / 4; // 45°

    private Vector3 p0;    // starting position

    void Start() {
        p0 = transform.position;
        if (target == null) {
            Debug.LogError("Assign a target Transform for perching.");
            enabled = false;
        }
    }

    void Update() {
        float t = Mathf.Clamp(Time.time, 0f, totalTime);

        // 1) compute tau0
        float dist0 = Vector3.Distance(target.position, p0);
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

        // 5) displacement along local “up” for M3 (only z in your Python, adjust to Unity if needed)
        Vector3 M3 = new Vector3(0, 0, d_t * sin_t);

        // 6) assemble p(t)
        Vector3 p_t = M1 * target.position + M2 * p0 + M3;

        transform.position = p_t;
    }
}
