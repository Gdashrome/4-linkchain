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
    public bool randomOrientation = false;
    public bool randomTarget = false;
    public int maxIterations = 20;

    [Range(0.00001f, 1f)]
    public float tolerance = 0.1f;
    [Range(0.005f, 1f)]
    public float stepSize = 0.01f;
    
    [Header("Data Collection")]
    [Range(1, 10000)]
    public int maxTestNum = 1;
    public bool collectData = false;
    public string filenameCustom = "";
    
    int testNum = 1;
    float initialError = 0f;

    float angleOffset = Mathf.PI / 2f;

    bool isSolving = false;
    bool isSolvingAngle = false;

    
    float[] lengths;
    Vector3[] positions;
    Vector3 lastTargetPos;
    Vector2 basePosition;

    float[] angles;
    float[] targetAngles;
    Vector3[] originalRotations;
    Vector3 originalTargetPosition;

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
        angles = new float[n - 1];
        targetAngles = new float[n - 1];

        UpdateLengths();
        for (int i = 0; i < lengths.Length; i++)
            totalLength += lengths[i];

        if (randomOrientation)
        {
            for (int i = 0; i < joints.Length; i++)
            {
                float randomAngle = Random.Range(0f, 360f);
                joints[i].eulerAngles = new Vector3(0, 0, randomAngle);
            }
        }

        if (randomTarget)
        {
            SetRandomTarget();
        }

        UpdatePositions();
        
        originalRotations = new Vector3[joints.Length];
        for (int i = 0; i < joints.Length; i++)
            originalRotations[i] = joints[i].eulerAngles;

        originalTargetPosition = target.position;

        SolveIKStep();

        algorithmTime = 0f;
        totalIterations = 0;
        iterationTime = 0f;
        rawData = new List<IKRawData>();
        shortData = new List<IKShortData>();

        if (randomTarget)
        {
            SetRandomTarget();
        }

        UpdatePositions();
        calculateTargetAngles();
        for (int i = 0; i < angles.Length; i++)
            angles[i] = targetAngles[i];
        
        initialError = Vector2.Distance(positions[positions.Length - 1], target.position);
        lastTargetPos = target.position;

        isSolving = true;

        print($"Total arm length: {totalLength}");
        print($"Initial error: {Vector2.Distance(positions[positions.Length - 1], target.position)}");
        print($"Lengths: " + string.Join(", ", lengths));
        print("Joint positions: " + string.Join(", ", System.Array.ConvertAll(joints, j => j.position.ToString())));
        print("Target angles: " + string.Join(", ", targetAngles.Select(a => a.ToString("F2"))));
        print("Angles: " + string.Join(", ", angles.Select(a => a.ToString("F2"))));    
    }

    void Update()
    {
        if ((target.position - lastTargetPos).magnitude > tolerance)
        {
            isSolving = true;
            totalIterations = 0;
            algorithmTime = 0f;
            initialError = Vector2.Distance(positions[positions.Length - 1], target.position);
            lastTargetPos = target.position;
        }

        if (!isSolving)
        {
            if (testNum <= maxTestNum && collectData)
            {
                algorithmTime = 0f;
                totalIterations = 0;
                iterationTime = 0f;

                target.position = originalTargetPosition;
                for (int i = 0; i < joints.Length; i++)
                    joints[i].eulerAngles = originalRotations[i];
                
                if (randomOrientation)
                {
                    for (int i = 0; i < joints.Length; i++)
                    {
                        float randomAngle = Random.Range(0f, 360f);
                        joints[i].eulerAngles = new Vector3(0, 0, randomAngle);
                    }
                }
                if (randomTarget)
                {
                    SetRandomTarget();
                }

                UpdatePositions();
                initialError = Vector2.Distance(positions[positions.Length - 1], target.position);
                lastTargetPos = target.position;
                isSolving = true;
            }
            return;
        }    
        
        if (!isSolvingAngle)
        {
            SolveIKStep();
            //UpdateRotations();
        }
        else if (isSolvingAngle && stepSize == 1f)
        {
            UpdateRotations();
        }
        else
        {
            ApplyAnglesSmooth();
        }
    }

    void SetRandomTarget()
    {
        Vector2 randomDir = Random.insideUnitCircle.normalized;
        float randomDist = Random.Range(0f*totalLength, 1f*totalLength);
        target.position = (Vector2)joints[0].position + randomDir * randomDist;
    }
    // void SetRandomTarget()
    // {
    //     float angle = Random.Range(0f, Mathf.PI * 2f);
    //     float radius = totalLength * Mathf.Sqrt(Random.value);

    //     Vector2 offset = new Vector2(Mathf.Cos(angle), Mathf.Sin(angle)) * radius;
    //     target.position = (Vector2)joints[0].position + offset;
    // }
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
            joints[i].eulerAngles = new Vector3(0, 0, angle - angleOffset * Mathf.Rad2Deg);
        }
        
        isSolvingAngle = false;
    }

    void calculateTargetAngles()
    {
        for (int i = 0; i < joints.Length - 1; i++)
        {
            Vector3 dir = positions[i + 1] - positions[i];
            float angle = Mathf.Atan2(dir.y, dir.x)  * Mathf.Rad2Deg;
            targetAngles[i] = angle - (angleOffset * Mathf.Rad2Deg);
        }
    }

    void ApplyAnglesSmooth() 
    {
        bool done = true;

        for (int i = 0; i < joints.Length - 1; i++)
        {
            float current = angles[i];
            float target = targetAngles[i];

            float newAngle = Mathf.LerpAngle(current, target, stepSize);

            joints[i].eulerAngles = new Vector3(0, 0, newAngle);

            angles[i] = newAngle;

            if (Mathf.Abs(Mathf.DeltaAngle(current, target)) > 0.15)
                done = false;
            else
                angles[i] = targetAngles[i];
        }

        isSolvingAngle = !done;
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
        float baseToTargetDistance = Vector2.Distance(positions[0], target.position);

        float error = Vector2.Distance(positions[positions.Length - 1], target.position);

        //UnityEngine.Debug.Log("Test: " + testNum + ", Iteration: " + totalIterations + ", Iteration Time: " + iterationTime + "s, Elapsed Time: " + algorithmTime + "s, Error: " + error);
        rawData.Add(new IKRawData(testNum, totalIterations, iterationTime, algorithmTime, error));

        if (baseToTargetDistance > totalLength)
        {
            SolveUnreachable();
            UpdateRotations();
            
            return;
        }

        if (error < tolerance)
        {
            isSolving = false;
            UnityEngine.Debug.Log("Trial: " + testNum + ", IK Converged in " + totalIterations + " iterations, " + "error: " + error + ", algorithm time: " + algorithmTime + "s");
            shortData.Add(new IKShortData(testNum, totalIterations, algorithmTime/totalIterations, algorithmTime, initialError, error, target.position, target.position.magnitude));
            testNum++;

            if (testNum > maxTestNum && collectData)
            {
                string rawDataFileNameBase = "FABRIK_RawData";
                string shortDataFileNameBase = "FABRIK_SummaryData";
                string rawDataFileNameComponent;
                string shortDataFileNameComponent;
                string timestamp = System.DateTime.Now.ToString("(yyyy-MM-dd_HH-mm-ss)");
                string toleranceStr = "(Tolerance=" + tolerance.ToString("F3") + ")";
                if (randomOrientation && randomTarget)
                {   
                    rawDataFileNameComponent = "(Random Orientation and Target)";
                    shortDataFileNameComponent = "(Random Orientation and Target)";
                }
                else if (randomOrientation)
                {
                    rawDataFileNameComponent = "(Random Orientation)";
                    shortDataFileNameComponent = "(Random Orientation)";
                }
                else if (randomTarget)
                {
                    rawDataFileNameComponent = "(Random Target)";
                    shortDataFileNameComponent = "(Random Target)";
                }
                else
                {
                    rawDataFileNameComponent = "(Preset Orientation and Target)";
                    shortDataFileNameComponent = "(Preset Orientation and Target)";
                }
                
                string rawDataFileName = filenameCustom + "_" + rawDataFileNameBase + "_" + rawDataFileNameComponent + "_" + toleranceStr + "_" + timestamp + ".csv";
                string shortDataFileName = filenameCustom + "_" + shortDataFileNameBase + "_" + shortDataFileNameComponent + "_" + toleranceStr + "_" + timestamp + ".csv";

                CSVWriter.WriteRawData(rawDataFileName, rawData);
                CSVWriter.WriteShortData(shortDataFileName, shortData);

                // foreach (IKRawData data in rawData)
                // {
                //     print("Test Number: " + data.testNum + ", Iteration: " + data.iteration + ", Iteration Time: " + data.iterationTime + ", Elapsed Time: " + data.elapsedIterationTime + ", Iteration Error: " + data.iterationError);
                // }
                // foreach (IKShortData data in shortData)
                // {
                //     print("Test Number: " + data.testNum + ", Iterations: " + data.iterations + ", Average Iteration Time: " + data.averageIterationTime + ", Total Time: " + data.totalIterationTime + ", Iteration Error: " + data.finalError);
                // }
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

        calculateTargetAngles(); 

        isSolvingAngle = !collectData;
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