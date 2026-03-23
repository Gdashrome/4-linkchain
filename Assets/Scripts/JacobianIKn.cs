using UnityEngine;
using System.Diagnostics;

public class JacobianIKn : MonoBehaviour
{
    public enum InverseForm {Transpose, SVD}
    public enum IKMethod {PseudoInverse, DampedLeastSquares}

    [Header("Scene References")]
    public Transform[] joints;
    public Transform endEffector;
    public Transform target;

    [Header("IK Settings")]
    public InverseForm form = InverseForm.Transpose;
    public IKMethod method = IKMethod.PseudoInverse;

    public int maxIterations = 20;
    public float tolerance = 0.001f;
    public float stepSize = 0.1f;
    public float damping = 0.1f;
    float angleOffset = Mathf.PI / 2f;
    bool isSolving = true;
    private int currentIteration = 0;
    Vector2 lastTargetPos;
    float maxReach;

    private float algorithmTime = 0f;
    private int totalIterations = 0;

    float[] angles;
    float[] lengths;

    void Start()
    {
        int n = joints.Length;
        angles = new float[n];
        lengths = new float[n];

        UpdateLengths();
        ReadAnglesFromTransforms();

        maxReach = 0f;
        for (int i = 0; i < lengths.Length; i++)
            maxReach += lengths[i];
        
        if (method == IKMethod.PseudoInverse)
        {
            for (int i = 0; i < angles.Length; i++)
                angles[i] += Random.Range(-0.01f, 0.01f);
            ApplyAnglesToTransforms();
        }

        print("Initial End Effector: " + ForwardKinematics(angles));
        print("Target Position: " + target.position);
        print("Intial Lengths: " + string.Join(", ", lengths));
        print("Initial Angles (deg): " + string.Join(", ", angles));
    }

    void Update()
    {
        Vector2 currentTarget = target.position;

        if ((currentTarget - lastTargetPos).magnitude > 0.001f)
        {
            isSolving = true;
            currentIteration = 0;
            lastTargetPos = currentTarget;

            algorithmTime = 0f;
            totalIterations = 0;
        }

        if (!isSolving) return;

        ReadAnglesFromTransforms();
        SolveIKStep();
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
    }

    void ExtendTowardsTarget(Vector2 direction)
    {
        float targetAngle = Mathf.Atan2(direction.y, direction.x) - angleOffset;

        angles[0] = Mathf.LerpAngle(angles[0] * Mathf.Rad2Deg, targetAngle * Mathf.Rad2Deg, 0.1f) * Mathf.Deg2Rad;

        for (int i = 1; i < angles.Length; i++)
        {
            angles[i] = Mathf.Lerp(angles[i], 0f, 0.1f);
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

    float[,] ComputeJacobian(float[] theta)
    {
        int n = theta.Length;
        float[,] J = new float[2, n];

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

        return J;
    }

    void SolveIKStep()
    {
        Vector2 current = ForwardKinematics(angles);
        Vector2 rootPos = joints[0].position;
        Vector2 goal = target.position;

        Vector2 error = goal - current;
        Vector2 toTarget = goal - rootPos;
        float roottoTargetDistance = toTarget.magnitude;

        if (roottoTargetDistance > maxReach)
        {
            ExtendTowardsTarget(toTarget.normalized);
            return;
        }

        if (error.magnitude < tolerance)
        {
            isSolving = false;
            UnityEngine.Debug.Log("IK Converged in " + totalIterations + " iterations, algorithm time: " + algorithmTime + "s");
            return;
        }

        if (currentIteration >= maxIterations)
        {
            isSolving = false;
            UnityEngine.Debug.Log("Max iterations reached (" + maxIterations + "), no convergent solution. Algorithm time: " + algorithmTime + "s");
            return;
        }

        Stopwatch sw = Stopwatch.StartNew();

        float[,] J = ComputeJacobian(angles);

        float[] delta;

        if (form == InverseForm.SVD)
        {
            if (method == IKMethod.DampedLeastSquares)
                delta = SolveDLS_SVD(J, error);
            else
                delta = SolvePseudoInverse_SVD(J, error);
        }
        else
        {
            if (method == IKMethod.DampedLeastSquares)
                delta = SolveDLS(J, error);
            else
                delta = SolvePseudoInverse(J, error);  
        }
        
        sw.Stop();
        algorithmTime += (float)sw.Elapsed.TotalSeconds;
        totalIterations++;

        for (int i = 0; i < angles.Length; i++)
            angles[i] += stepSize * delta[i];

        currentIteration++;
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