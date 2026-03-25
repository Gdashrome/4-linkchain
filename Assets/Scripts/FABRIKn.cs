using UnityEngine;
using System.Diagnostics;

public class FABRIK_IKn : MonoBehaviour
{
    [Header("Scene References")]
    public Transform[] joints;
    public Transform target;

    [Header("FABRIK Settings")]
    public int maxIterations = 20;

    [Range(0.001f, 1f)]
    public float tolerance = 0.1f;
    [Range(0.01f, 1f)]
    public float stepSize = 0.01f;
    [Range(0f, 5f)]

    float angleOffset = Mathf.PI / 2f;

    bool isSolving = true;
    
    float[] lengths;
    Vector3[] positions;
    Vector3 lastTargetPos;
    Vector2 basePosition;

    float totalLength;

    private float algorithmTime = 0f;
    private int totalIterations = 0;

    void Start()
    {   
        int n = joints.Length;
        lengths = new float[n - 1];
        positions = new Vector3[n];
        basePosition = joints[0].position;

        UpdateLengths();
        UpdatePositions();

        SolveIKStep();

        algorithmTime = 0f;
        totalIterations = 0;
        UpdatePositions();

        for (int i = 0; i < lengths.Length; i++)
            totalLength += lengths[i];
        
        print($"Total arm length: {totalLength}");
        print($"Initial error: {Vector2.Distance(positions[positions.Length - 1], target.position)}");
        print("lengths: " + string.Join(", ", lengths));
        print("joint positions: " + string.Join(", ", System.Array.ConvertAll(joints, j => j.position.ToString())));
    }

    void Update()
    {
        float baseToTargetDistance = Vector2.Distance(positions[0], target.position);

        if ((target.position - lastTargetPos).magnitude > tolerance)
        {
            isSolving = true;
            lastTargetPos = target.position;

            totalIterations = 0;
            algorithmTime = 0f;
        }

        if (!isSolving) return;   
    
        if (baseToTargetDistance > totalLength)
        {
            SolveUnreachable();
            UpdateRotations();
            isSolving = false; 
        }
        SolveIKStep();
        UpdateRotations();
    }

    void UpdateLengths()
    {
        for (int i = 0; i < lengths.Length; i++)
        {
            Vector2 a = joints[i].position;
            Vector2 b = joints[i + 1].position;

            lengths[i] = Vector2.Distance(a, b);
        }
    }

    void UpdatePositions()
    {
        for (int i = 0; i < positions.Length; i++)
        {
            positions[i] = joints[i].position;
        }
    }

    void UpdateRotations()
    {
        for (int i = 0; i < joints.Length - 1; i++)
        {
            Vector3 dir = positions[i + 1] - positions[i];
            float angle = Mathf.Atan2(dir.y, dir.x) * Mathf.Rad2Deg;
            joints[i].rotation = Quaternion.Euler(0, 0, angle - angleOffset * Mathf.Rad2Deg);
        }
    }

    void SolveUnreachable()
    {
        for (int i = 0; i < positions.Length - 1; i++)
        {
            float jointToTarget = Vector2.Distance(positions[i], target.position);
            float lambda = lengths[i] / jointToTarget;
            positions[i + 1] = (1 - lambda) * positions[i] + lambda * target.position;
        }
    }

    void SolveIKStep()
    {
        float error = Vector2.Distance(positions[positions.Length - 1], target.position);

         if (error < tolerance)
        {
            isSolving = false;
            UnityEngine.Debug.Log("IK Converged in " + totalIterations + " iterations, " + "error: " + error + ", algorithm time: " + algorithmTime + "s");
            return;
        }

        if (totalIterations >= maxIterations)
        {
            isSolving = false;
            UnityEngine.Debug.Log("Max iterations reached (" + maxIterations + "), no convergent solution. Algorithm time: " + algorithmTime + "s");
            return;
        }
        
        Stopwatch sw = Stopwatch.StartNew();
        SolveFABRIK();
        sw.Stop();
        algorithmTime += (float)sw.Elapsed.TotalSeconds;
        totalIterations++;

        error = Vector2.Distance(positions[positions.Length - 1], target.position);
        print("Iteration " + totalIterations + ", Error: " + error + ", Iteration Time: " + (float)sw.Elapsed.TotalSeconds + ", Total Time: " + algorithmTime + "s");
    }

    void SolveFABRIK()
    {
        // Backward reaching
        positions[positions.Length - 1] = target.position;

        for (int i = positions.Length - 2; i >= 0; i--)
        {
            float jointToPrevious = Vector2.Distance(positions[i + 1], positions[i]);
            float lambda = lengths[i] / jointToPrevious;
            positions[i] = (1 - lambda) * positions[i + 1] + lambda * positions[i];
        }

        // Forward reaching
        positions[0] = basePosition;

        for (int i = 0; i < positions.Length - 1; i++)
        {
            float jointToNext = Vector2.Distance(positions[i + 1], positions[i]);
            float lambda = lengths[i] / jointToNext;
            positions[i + 1] = (1 - lambda) * positions[i] + lambda * positions[i + 1];
        }
    }
}