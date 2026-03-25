using UnityEngine;
using System.Diagnostics;

public class JacobianIK : MonoBehaviour
{
    public enum InverseForm {Transpose, SVD}
    public enum IKMethod {PseudoInverse, DampedLeastSquares}

    [Header("Scene References")]
    public Transform[] joints;   // for 3 joints
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

        Vector3 delta;

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

    Vector3 SolveDLS(float[,] J, Vector2 error)
    {
        float j11 = J[0,0], j12 = J[0,1], j13 = J[0,2];
        float j21 = J[1,0], j22 = J[1,1], j23 = J[1,2];

        float a = j11*j11 + j12*j12 + j13*j13;
        float b = j11*j21 + j12*j22 + j13*j23;
        float c = j21*j21 + j22*j22 + j23*j23;

        float lambda2 = damping * damping;
        a += lambda2;
        c += lambda2;

        float det = a * c - b * b;
        if (Mathf.Abs(det) < 1e-8f)
            return Vector3.zero;

        float inv00 =  c / det;
        float inv01 = -b / det;
        float inv10 = -b / det;
        float inv11 =  a / det;

        float y0 = inv00 * error.x + inv01 * error.y;
        float y1 = inv10 * error.x + inv11 * error.y;

        return new Vector3(
            j11*y0 + j21*y1,
            j12*y0 + j22*y1,
            j13*y0 + j23*y1
        );
    }

    Vector3 SolveDLS_SVD(float[,] J, Vector2 error)
    {
        float lambda = damping * damping;
        float j11 = J[0,0], j12 = J[0,1], j13 = J[0,2];
        float j21 = J[1,0], j22 = J[1,1], j23 = J[1,2];

        float a = j11*j11 + j12*j12 + j13*j13;
        float b = j11*j21 + j12*j22 + j13*j23;
        float c = j21*j21 + j22*j22 + j23*j23;

        float trace = a + c;
        float det = a*c - b*b;
        float temp = Mathf.Sqrt(Mathf.Max(0f, trace*trace/4 - det));
        float lambda1 = trace/2 + temp;
        float lambda2 = trace/2 - temp;

        float sigma1 = Mathf.Sqrt(Mathf.Max(lambda1, 1e-8f));
        float sigma2 = Mathf.Sqrt(Mathf.Max(lambda2, 1e-8f));

        Vector2 u1, u2;
        if (Mathf.Abs(b) > 1e-8f)
        {
            u1 = new Vector2(lambda1 - c, b).normalized;
            u2 = new Vector2(-b, lambda1 - c).normalized;
        }
        else
        {
            u1 = new Vector2(1,0);
            u2 = new Vector2(0,1);
        }

        Vector3 v1 = new Vector3(
            (j11*u1.x + j21*u1.y)/sigma1,
            (j12*u1.x + j22*u1.y)/sigma1,
            (j13*u1.x + j23*u1.y)/sigma1
        );
        Vector3 v2 = new Vector3(
            (j11*u2.x + j21*u2.y)/sigma2,
            (j12*u2.x + j22*u2.y)/sigma2,
            (j13*u2.x + j23*u2.y)/sigma2
        );

        float factor1 = sigma1 / (sigma1*sigma1 + lambda);
        float factor2 = sigma2 / (sigma2*sigma2 + lambda);

        float y0 = Vector2.Dot(u1, error) * factor1;
        float y1 = Vector2.Dot(u2, error) * factor2;

        Vector3 delta = v1 * y0 + v2 * y1;
        return delta;
    }

    Vector3 SolvePseudoInverse(float[,] J, Vector2 error)
    {
        float j11 = J[0,0], j12 = J[0,1], j13 = J[0,2];
        float j21 = J[1,0], j22 = J[1,1], j23 = J[1,2];

        float a = j11*j11 + j12*j12 + j13*j13;
        float b = j11*j21 + j12*j22 + j13*j23;
        float c = j21*j21 + j22*j22 + j23*j23;

        float det = a*c - b*b;
        if (Mathf.Abs(det) < 1e-8f)
            return Vector3.zero;

        float inv00 =  c / det;
        float inv01 = -b / det;
        float inv10 = -b / det;
        float inv11 =  a / det;

        float y0 = inv00 * error.x + inv01 * error.y;
        float y1 = inv10 * error.x + inv11 * error.y;

        return new Vector3(
            j11*y0 + j21*y1,
            j12*y0 + j22*y1,
            j13*y0 + j23*y1
        );
    }

    Vector3 SolvePseudoInverse_SVD(float[,] J, Vector2 error)
    {
        float j11 = J[0,0], j12 = J[0,1], j13 = J[0,2];
        float j21 = J[1,0], j22 = J[1,1], j23 = J[1,2];

        float a = j11*j11 + j12*j12 + j13*j13;
        float b = j11*j21 + j12*j22 + j13*j23;
        float c = j21*j21 + j22*j22 + j23*j23;

        float trace = a + c;
        float det = a*c - b*b;
        float temp = Mathf.Sqrt(Mathf.Max(0f, trace*trace/4 - det));
        float lambda1 = trace/2 + temp;
        float lambda2 = trace/2 - temp;

        float sigma1 = Mathf.Sqrt(Mathf.Max(lambda1, 1e-8f));
        float sigma2 = Mathf.Sqrt(Mathf.Max(lambda2, 1e-8f));

        Vector2 u1, u2;
        if (Mathf.Abs(b) > 1e-8f)
        {
            u1 = new Vector2(lambda1 - c, b).normalized;
            u2 = new Vector2(-b, lambda1 - c).normalized;
        }
        else
        {
            u1 = new Vector2(1,0);
            u2 = new Vector2(0,1);
        }

        Vector3 v1 = new Vector3(
            (j11*u1.x + j21*u1.y)/sigma1,
            (j12*u1.x + j22*u1.y)/sigma1,
            (j13*u1.x + j23*u1.y)/sigma1
        );
        Vector3 v2 = new Vector3(
            (j11*u2.x + j21*u2.y)/sigma2,
            (j12*u2.x + j22*u2.y)/sigma2,
            (j13*u2.x + j23*u2.y)/sigma2
        );

        float y0 = Vector2.Dot(u1, error) / sigma1;
        float y1 = Vector2.Dot(u2, error) / sigma2;

        Vector3 delta = v1 * y0 + v2 * y1;
        return delta;
    }
}