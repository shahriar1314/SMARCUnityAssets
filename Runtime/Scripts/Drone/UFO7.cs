using UnityEngine;

public class UFO7 : MonoBehaviour
{
    [Header("Trajectory Parameters")]
    public Transform targetA;           
    public Transform targetB;           
    public float heightOffset     = 0.3f;      // how far above the water line
    public float totalTime        = 35f;       // duration of the perching phase
    public float k                = 3f;        // distance‐decay shape parameter
    public float kd_alpha         = 0.4f;      // angle‐decay shape parameter
    public float initialVelocity  = 4.5f;      // m/s
    public float alpha0           = Mathf.PI/4; // 45° initial pitch‐angle gap

    [Header("Perch Velocity Control")]
    public float minPerchVelocity = 0.5f;      // minimum approach speed (m/s)

    [Header("Departure Parameters")]
    public float flatDuration     = 1f;        // time to go straight (same height) before climbing
    public float exitAngleDeg     = 30f;       // climb‐out angle in degrees
    public float flatSpeed        = 0.5f;      // speed during flat
    public float exitSpeed        = 2.5f;      // speed during climb

    private Vector3 p0;                       // start position
    private float  startTime;                 // when the motion began

    void Start()
    {
        p0 = transform.position;
        if (targetA == null || targetB == null)
        {
            Debug.LogError("Assign both targetA and targetB Transforms.");
            enabled = false;
            return;
        }
        startTime = Time.time;
    }

    void Update()
    {
        float elapsed = Time.time - startTime;

        // --- recompute moving-target geometry each frame ---
        Vector3 raisedA  = targetA.position + Vector3.up * heightOffset;
        Vector3 raisedB  = targetB.position + Vector3.up * heightOffset;
        Vector3 midPoint = (raisedA + raisedB) * 0.5f;

        // horizontal AB vector and its perpendicular
        Vector3 ab    = raisedB - raisedA; ab.y = 0f;
        Vector3 abUnit = ab.normalized;
        Vector3 hUnit  = new Vector3(-abUnit.z, 0f, abUnit.x); 

        // characteristic time for perching phase
        float dist0 = Vector3.Distance(midPoint, p0);
        float tau0  = dist0 / initialVelocity;

        // departure (climb‐out) direction
        float phiExit    = exitAngleDeg * Mathf.Deg2Rad;
        Vector3 departureDir = new Vector3(
            hUnit.x * Mathf.Cos(phiExit),
            Mathf.Sin(phiExit),
            hUnit.z * Mathf.Cos(phiExit)
        );

        // --- Phase 1: Perching (to the line through A–B at its midpoint) ---
        if (elapsed <= totalTime)
        {
            float frac    = 1f - elapsed / totalTime;
            float d_orig  = initialVelocity * tau0 * Mathf.Pow(frac, 1f / k);
            float fallback = minPerchVelocity * (totalTime - elapsed);
            float d_t     = Mathf.Max(d_orig, fallback);

            float alpha_t = alpha0 * Mathf.Pow(d_t / (initialVelocity * tau0), 1f / kd_alpha);
            float sin0    = Mathf.Sin(alpha0);
            float sinT    = Mathf.Sin(alpha_t);
            float M1      = (sin0 - sinT) / sin0;
            float M2      = sinT / sin0;
            Vector3 M3    = new Vector3(0f, d_t * sinT, 0f);

            transform.position = M1 * midPoint + M2 * p0 + M3;
        }
        // --- Phase 2: straight‐ahead at same height along the perpendicular ---
        else if (elapsed <= totalTime + flatDuration)
        {
            float dt = elapsed - totalTime;
            transform.position = midPoint + hUnit * flatSpeed * dt;
        }
        // --- Phase 3: climb‐out from end of straight segment ---
        else
        {
            float dt = elapsed - (totalTime + flatDuration);
            Vector3 flatEnd = midPoint + hUnit * flatSpeed * flatDuration;
            transform.position = flatEnd + departureDir * exitSpeed * dt;
        }
    }
}
