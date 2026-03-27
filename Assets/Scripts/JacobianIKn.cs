using UnityEngine;
using System.Diagnostics;
using System.Collections.Generic;
using System.Linq;
using System.Data;
using JetBrains.Annotations;

public class JacobianIKn : MonoBehaviour
{
    public enum IKMethod {
        TransposeNormal,
        TransposePseudoInverse,
        TransposeDampedLeastSquares,
        SVDPseudoInverse,
        SVDDampedLeastSquares
    } 

    [Header("Scene References")]
    public Transform[] joints;
    public Transform endEffector;
    public Transform target;

    [Header("IK Settings")]
    public IKMethod method = IKMethod.TransposeNormal;

    public bool randomOrientation = false;
    public bool randomTarget = false;

    public int maxIterations = 20;

    [Range(0.001f, 1f)]
    public float tolerance = 0.1f;
    [Range(0.01f, 1f)]
    public float stepSize = 0.01f;
    [Range(0f, 5f)]
    public float damping = 2f;

    [Header("Data Collection")]
    [Range(1, 100)]
    public int maxTestNum = 1;
    public bool collectData = false;
    
    int testNum = 1;

    float angleOffset = Mathf.PI / 2f;
    bool isSolving = true;
    bool isSolvingAngle = false;
    Vector2 lastTargetPos;
    float maxReach;

    private float algorithmTime = 0f;
    private float iterationTime = 0f;
    private int totalIterations = 0;
    private List<IKRawData> rawData = new List<IKRawData>();
    private List<IKShortData> shortData = new List<IKShortData>();

    float[] angles;
    float[] startingAngles;
    float[] targetAngles;
    float[] lengths;
    float[,] J;

    Vector3[] originalRotations;
    Vector3 originalTargetPosition;

    void Start()
    {
        int n = joints.Length;
        angles = new float[n];
        startingAngles = new float[n];
        targetAngles = new float[n];
        lengths = new float[n];
        J = new float[2, n];

        UpdateLengths();
        ReadAnglesFromTransforms();

        if (randomOrientation)
        {
            for (int i = 0; i < angles.Length; i++)
                angles[i] += Random.Range(-Mathf.PI, Mathf.PI);
            ApplyAnglesToTransforms();
        }

        if (randomTarget)
        {
            Vector2 randomDir = Random.insideUnitCircle.normalized;
            float randomDist = Random.Range(0f, maxReach);
            target.position = (Vector2)joints[0].position + randomDir * randomDist;
        }

        startingAngles = (float[])angles.Clone();

        originalTargetPosition = target.position;

        originalRotations = new Vector3[joints.Length];
        for (int i = 0; i < joints.Length; i++)
        {
            originalRotations[i] = joints[i].eulerAngles;
        }

        maxReach = 0f;
        for (int i = 0; i < lengths.Length; i++)
            maxReach += lengths[i];
        
        SolveIKStep();
        algorithmTime = 0f;
        totalIterations = 0;
        iterationTime = 0f;
        rawData = new List<IKRawData>();
        shortData = new List<IKShortData>();
        resetRig();
        
        print("Initial End Effector: " + ForwardKinematics(angles));
        print("Target Position: " + target.position);
        print("Intial Lengths: " + string.Join(", ", lengths));
        print("Initial Angles (deg): " + string.Join(", ", angles));
    }

    void Update()
    {
        Vector2 currentTarget = target.position;

        if ((currentTarget - lastTargetPos).magnitude > tolerance)
        {
            isSolving = true;
            lastTargetPos = currentTarget;

            algorithmTime = 0f;
            totalIterations = 0;
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
                    Vector2 randomDir = Random.insideUnitCircle.normalized;
                    float randomDist = Random.Range(0f, maxReach);
                    target.position = (Vector2)joints[0].position + randomDir * randomDist;
                }

                lastTargetPos = target.position;
                resetRig();
                isSolving = true;
            }
            return;
        }

        if (!isSolvingAngle)
        {
            SolveIKStep();
        }
        else
            ApplyAnglesSmooth();
    }

    void resetRig()
    {
        angles = (float[])startingAngles.Clone();
        ApplyAnglesToTransforms();
    }
    void UpdateLengths()
    {
        for (int i = 0; i < joints.Length; i++)
        {
            Vector3 a = joints[i].position;
            Vector3 b = (i == joints.Length - 1)
                ? endEffector.position
                : joints[i + 1].position;

            lengths[i] = Vector3.Distance(a, b);
        }
    }

    void ReadAnglesFromTransforms()
    {
        for (int i = 0; i < joints.Length; i++)
        {
            float a = joints[i].localEulerAngles.z * Mathf.Deg2Rad;
            if (a > Mathf.PI) a -= 2 * Mathf.PI;
            targetAngles[i] = a;
            angles[i] = a;
        }
    }

    void ApplyAnglesToTransforms()
    {
        for (int i = 0; i < joints.Length; i++)
        {
            Vector3 rot = joints[i].localEulerAngles;
            rot.z = angles[i] * Mathf.Rad2Deg;
            joints[i].localEulerAngles = rot;
        }
        isSolvingAngle = false;
    }

    void ApplyAnglesSmooth() 
    {
        bool done = true;

        for (int i = 0; i < joints.Length; i++)
        {
            float current = angles[i] * Mathf.Rad2Deg;
            float target = targetAngles[i] * Mathf.Rad2Deg;

            float newAngle = Mathf.LerpAngle(current, target, stepSize);

            joints[i].localEulerAngles = new Vector3(0, 0, newAngle);

            angles[i] = newAngle * Mathf.Deg2Rad;

            if (Mathf.Abs(Mathf.DeltaAngle(current, target)) > 0.15)
                done = false;
            else
                angles[i] = targetAngles[i];
        }

        isSolvingAngle = !done;
    }

    void ExtendTowardsTarget(Vector2 direction)
    {
        float targetAngle = Mathf.Atan2(direction.y, direction.x) - angleOffset;

        angles[0] = Mathf.LerpAngle(angles[0] * Mathf.Rad2Deg, targetAngle * Mathf.Rad2Deg, stepSize) * Mathf.Deg2Rad;

        for (int i = 1; i < angles.Length; i++)
        {
            angles[i] = Mathf.Lerp(angles[i], 0f, stepSize);
        }
    }


    Vector2 ForwardKinematics(float[] theta)
    {
        float x = 0f;
        float y = 0f;
        float sum = 0f;

        for (int i = 0; i < theta.Length; i++)
        {
            sum += theta[i];
            x += lengths[i] * Mathf.Cos(sum + angleOffset);
            y += lengths[i] * Mathf.Sin(sum + angleOffset);
        }

        return new Vector2(x, y);
    }

    void ComputeJacobian(float[] theta)
    {
        int n = theta.Length;

        for (int j = 0; j < n; j++)
        {
            float dx = 0f;
            float dy = 0f;
            float sum = 0f;

            for (int k = 0; k < n; k++)
            {
                sum += theta[k];

                if (k >= j)
                {
                    dx -= lengths[k] * Mathf.Sin(sum + angleOffset);
                    dy += lengths[k] * Mathf.Cos(sum + angleOffset);
                }
            }

            J[0, j] = dx;
            J[1, j] = dy;
        }
    }

    void SolveIKStep()
    {
        Vector2 current = ForwardKinematics(angles);
        Vector2 rootPos = joints[0].position;
        Vector2 targetPos = target.position;

        Vector2 error = targetPos - current;
        Vector2 toTarget = targetPos - rootPos;
        float roottoTargetDistance = toTarget.magnitude;

        rawData.Add(new IKRawData(testNum, totalIterations, iterationTime, algorithmTime, error.magnitude));

        if (roottoTargetDistance > maxReach)
        {
            ExtendTowardsTarget(toTarget.normalized);
            ApplyAnglesToTransforms();

            return;
        }

        if (error.magnitude < tolerance)
        {
            isSolving = false;
            UnityEngine.Debug.Log("IK Converged in " + totalIterations + " iterations, " + "error: " + error.magnitude + ", algorithm time: " + algorithmTime + "s");
            shortData.Add(new IKShortData(testNum, totalIterations, algorithmTime/totalIterations, algorithmTime, error.magnitude));
            testNum++;

            if (testNum > maxTestNum && collectData)
            {
                string rawDataFileNameBase;
                string shortDataFileNameBase;
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

                if (method == IKMethod.TransposeNormal)
                {
                    rawDataFileNameBase = "JacobianIK_Transpose_RawData";
                    shortDataFileNameBase = "JacobianIK_Transpose_SummaryData";
                }
                else if (method == IKMethod.TransposePseudoInverse)
                {
                    rawDataFileNameBase = "JacobianIK_Transpose-PseudoInverse_RawData";
                    shortDataFileNameBase = "JacobianIK_Transpose-PseudoInverse_SummaryData";
                }
                else if (method == IKMethod.TransposeDampedLeastSquares)
                {
                    rawDataFileNameBase = "JacobianIK_Transpose-DLS_RawData";
                    shortDataFileNameBase = "JacobianIK_Transpose-DLS_SummaryData";
                }
                else if (method == IKMethod.SVDPseudoInverse)
                {
                    rawDataFileNameBase = "JacobianIK_SVD-PseudoInverse_RawData";
                    shortDataFileNameBase = "JacobianIK_SVD-PseudoInverse_SummaryData";
                }
                else if (method == IKMethod.SVDDampedLeastSquares)
                {
                    rawDataFileNameBase = "JacobianIK_SVD-DLS_RawData";
                    shortDataFileNameBase = "JacobianIK_SVD-DLS_SummaryData";
                }
                else
                {
                    rawDataFileNameBase = "JacobianIK_MethodUnknown_RawData";
                    shortDataFileNameBase = "JacobianIK_MethodUnknown_SummaryData";
                }
                
                string rawDataFileName = rawDataFileNameBase + "_" + rawDataFileNameComponent + "_" + toleranceStr + "_" + timestamp + ".csv";
                string shortDataFileName = shortDataFileNameBase + "_" + shortDataFileNameComponent + "_" + toleranceStr + "_" + timestamp + ".csv";

                CSVWriter.WriteRawData(rawDataFileName, rawData);
                CSVWriter.WriteShortData(shortDataFileName, shortData);
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

        ComputeJacobian(angles);

        float[] delta;

        if (method == IKMethod.TransposeNormal)
        {
            delta = SolveJacobianTranspose(J, error);
        }
        else if (method == IKMethod.TransposePseudoInverse)
        {
            delta = SolvePseudoInverse(J, error);
        }
        else if (method == IKMethod.TransposeDampedLeastSquares)
        {
            delta = SolveDLS(J, error);
        }
        else if (method == IKMethod.SVDPseudoInverse)
        {
            delta = SolvePseudoInverse_SVD(J, error);
        }
        else if (method == IKMethod.SVDDampedLeastSquares)
        {
            delta = SolveDLS_SVD(J, error);
        }
        else {
            print("Unknown IK method selected.");
            return;
        }

        sw.Stop();
        algorithmTime += (float)sw.Elapsed.TotalSeconds;
        iterationTime = (float)sw.Elapsed.TotalSeconds;
        totalIterations++;

        error = targetPos - current;
        print("Iteration " + totalIterations + ", Error: " + error.magnitude + ", Iteration Time: " + (float)sw.Elapsed.TotalSeconds + ", Total Time: " + algorithmTime + "s");

        for (int i = 0; i < angles.Length; i++)
        {
            targetAngles[i] = angles[i] + delta[i];
        }  

        isSolvingAngle = true;
    }

    float[] SolveJacobianTranspose(float[,] J, Vector2 error)
    {
        int n = J.GetLength(1);

        float[,] JJt = new float[2, 2];

        for (int j = 0; j < n; j++)
        {
            JJt[0,0] += J[0,j] * J[0,j];
            JJt[0,1] += J[0,j] * J[1,j];
            JJt[1,0] += J[1,j] * J[0,j];
            JJt[1,1] += J[1,j] * J[1,j];
        }

        Vector2 w = new Vector2(
            JJt[0,0] * error.x + JJt[0,1] * error.y,
            JJt[1,0] * error.x + JJt[1,1] * error.y
        );

        float numerator = Vector2.Dot(error, w);
        float denominator = Vector2.Dot(w, w);

        float alpha = 0f;
        if (denominator > 1e-8f)
            alpha = numerator / denominator;

        float[] delta = new float[n];
        for (int i = 0; i < n; i++)
        {
            float jt_e = J[0,i] * error.x + J[1,i] * error.y;
            delta[i] = alpha * jt_e;
        }

        return delta;
    }

    float[] SolvePseudoInverse(float[,] J, Vector2 error)
    {
        int n = J.GetLength(1);
        float[,] JJt = new float[2,2];

        for (int j = 0; j < n; j++)
        {
            JJt[0,0] += J[0,j]*J[0,j];
            JJt[0,1] += J[0,j]*J[1,j];
            JJt[1,0] += J[1,j]*J[0,j];
            JJt[1,1] += J[1,j]*J[1,j];
        }

        float det = JJt[0,0]*JJt[1,1] - JJt[0,1]*JJt[1,0];
        if (Mathf.Abs(det) < 1e-8f)
            return new float[n];

        float inv00 = JJt[1,1]/det;
        float inv01 = -JJt[0,1]/det;
        float inv10 = -JJt[1,0]/det;
        float inv11 = JJt[0,0]/det;

        float[] delta = new float[n];
        for (int i = 0; i < n; i++)
            delta[i] = J[0,i]*(inv00*error.x + inv01*error.y) + J[1,i]*(inv10*error.x + inv11*error.y);

        return delta;
    }

    float[] SolveDLS(float[,] J, Vector2 error)
    {
        float lambda = damping * damping;
        int n = J.GetLength(1);
        float[,] JJt = new float[2,2];

        for (int j = 0; j < n; j++)
        {
            JJt[0,0] += J[0,j]*J[0,j];
            JJt[0,1] += J[0,j]*J[1,j];
            JJt[1,0] += J[1,j]*J[0,j];
            JJt[1,1] += J[1,j]*J[1,j];
        }

        JJt[0,0] += lambda;
        JJt[1,1] += lambda;

        float det = JJt[0,0]*JJt[1,1] - JJt[0,1]*JJt[1,0];
        if (Mathf.Abs(det) < 1e-8f)
            return new float[n];

        float inv00 = JJt[1,1]/det;
        float inv01 = -JJt[0,1]/det;
        float inv10 = -JJt[1,0]/det;
        float inv11 = JJt[0,0]/det;

        float[] delta = new float[n];
        for (int i = 0; i < n; i++)
            delta[i] = J[0,i]*(inv00*error.x + inv01*error.y) + J[1,i]*(inv10*error.x + inv11*error.y);

        return delta;
    }

    float[] SolvePseudoInverse_SVD(float[,] J, Vector2 error)
    {
        int n = J.GetLength(1);
        float[,] JJt = new float[2,2];

        for (int j = 0; j < n; j++)
        {
            JJt[0,0] += J[0,j]*J[0,j];
            JJt[0,1] += J[0,j]*J[1,j];
            JJt[1,0] += J[1,j]*J[0,j];
            JJt[1,1] += J[1,j]*J[1,j];
        }

        float trace = JJt[0,0] + JJt[1,1];
        float det = JJt[0,0]*JJt[1,1] - JJt[0,1]*JJt[1,0];
        float temp = Mathf.Sqrt(Mathf.Max(0f, trace*trace/4f - det));

        float lambda1 = trace/2f + temp;
        float lambda2 = trace/2f - temp;

        float sigma1 = Mathf.Sqrt(Mathf.Max(lambda1, 1e-8f));
        float sigma2 = Mathf.Sqrt(Mathf.Max(lambda2, 1e-8f));

        Vector2 u1, u2;
        if (Mathf.Abs(JJt[0,1]) > 1e-8f)
        {
            u1 = new Vector2(lambda1 - JJt[1,1], JJt[0,1]).normalized;
            u2 = new Vector2(-u1.y, u1.x);
        }
        else
        {
            u1 = new Vector2(1,0);
            u2 = new Vector2(0,1);
        }

        float[] delta = new float[n];

        for (int i = 0; i < n; i++)
        {
            float v1i = (J[0,i]*u1.x + J[1,i]*u1.y) / (sigma1);
            float v2i = (J[0,i]*u2.x + J[1,i]*u2.y) / (sigma2);

            delta[i] = Vector2.Dot(u1,error)/(sigma1) * v1i +
                    Vector2.Dot(u2,error)/(sigma2) * v2i;
        }

        return delta;
    }

    float[] SolveDLS_SVD(float[,] J, Vector2 error)
    {
        float lambda = damping * damping;
        int n = J.GetLength(1);
        float[,] JJt = new float[2,2];

        for(int j=0;j<n;j++){
            JJt[0,0] += J[0,j]*J[0,j];
            JJt[0,1] += J[0,j]*J[1,j];
            JJt[1,0] += J[1,j]*J[0,j];
            JJt[1,1] += J[1,j]*J[1,j];
        }

        float trace = JJt[0,0] + JJt[1,1];
        float det   = JJt[0,0]*JJt[1,1] - JJt[0,1]*JJt[1,0];
        float temp  = Mathf.Sqrt(Mathf.Max(0f, trace*trace/4 - det));
        float lambda1 = trace/2 + temp;
        float lambda2 = trace/2 - temp;

        float sigma1 = Mathf.Sqrt(Mathf.Max(lambda1, 1e-8f));
        float sigma2 = Mathf.Sqrt(Mathf.Max(lambda2, 1e-8f));

        Vector2 u1, u2;
        if(Mathf.Abs(JJt[0,1])>1e-8f){
            u1 = new Vector2(lambda1 - JJt[1,1], JJt[0,1]).normalized;
            u2 = new Vector2(lambda2 - JJt[1,1], JJt[0,1]).normalized;
        } else {
            u1 = new Vector2(1,0);
            u2 = new Vector2(0,1);
        }

        float[] v1 = new float[n];
        float[] v2 = new float[n];
        for(int i=0;i<n;i++){
            v1[i] = (J[0,i]*u1.x + J[1,i]*u1.y)/sigma1;
            v2[i] = (J[0,i]*u2.x + J[1,i]*u2.y)/sigma2;
        }

        float factor1 = sigma1/(sigma1*sigma1 + lambda);
        float factor2 = sigma2/(sigma2*sigma2 + lambda);

        float dot1 = Vector2.Dot(u1,error)*factor1;
        float dot2 = Vector2.Dot(u2,error)*factor2;

        float[] delta = new float[n];
        for(int i=0;i<n;i++){
            delta[i] = dot1*v1[i] + dot2*v2[i];
        }

        return delta;
    }
}