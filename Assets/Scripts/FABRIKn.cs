using UnityEngine;
using System.Diagnostics;
using System.Collections.Generic;
using System.Linq;
using System.Data;
using JetBrains.Annotations;

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
    private float iterationTime = 0f;
    private int totalIterations = 0;
    private List<IKRawData> rawData = new List<IKRawData>();
    private List<IKShortData> shortData = new List<IKShortData>();

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
        iterationTime = 0f;
        rawData = new List<IKRawData>();
        shortData = new List<IKShortData>();
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

        if (!isSolving)
        {
            
            return;
        }    
           
    
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

        print("Iteration " + totalIterations + ", Error: " + error + ", Iteration Time: " + iterationTime + ", Total Time: " + algorithmTime + "s");
        rawData.Add(new IKRawData(1, totalIterations, iterationTime, algorithmTime, error));

        if (error < tolerance)
        {
            isSolving = false;
            UnityEngine.Debug.Log("IK Converged in " + totalIterations + " iterations, " + "error: " + error + ", algorithm time: " + algorithmTime + "s");
            shortData.Add(new IKShortData(1, totalIterations, algorithmTime/totalIterations, algorithmTime, error));
            foreach (IKRawData data in rawData)
            {
                print("Test Number: " + data.testNum + ", Iteration: " + data.iteration + ", Iteration Time: " + data.iterationTime + ", Elapsed Time: " + data.elapsedIterationTime + ", Iteration Error: " + data.iterationError);
            }
            for (int i = 0; i < shortData.Count; i++)
            {
                IKShortData data = shortData[i];
                print("Test Number: " + data.testNum + ", Iterations: " + data.iterations + ", Average Iteration Time: " + data.averageIterationTime + ", Total Time: " + data.totalIterationTime + ", Iteration Error: " + data.finalError);
            }
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
        iterationTime = (float)sw.Elapsed.TotalSeconds;
        algorithmTime += iterationTime;
        totalIterations++;
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