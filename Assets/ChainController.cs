using UnityEngine;

public class ChainController : MonoBehaviour
{
    public Transform joint1; //first joint starting from the bottom
    public Transform joint2; //second joint
    public Transform joint3; //joint closest to the end effector
    public Transform endEffector; //end of the chain
    public Vector2 jointAngles; // stores angles of the joints
    public Transform Target;
    public float tolerance = 0.01f;
    public int iteration = 1;
    private float[] link_lengths;
    private float total_link_lengths;
    private Vector2[] positions;
    private Transform[] joints;

    void Start()
    {
        joints = new Transform[]{joint1, joint2,joint3, endEffector};
        int jL = joints.Length;
        link_lengths = new float[jL-1];
        positions = new Vector2[jL];
        total_link_lengths = 0f;

        for (int i=0; i<jL - 1; i++){
            link_lengths[i] = Vector2.Distance(joints[i].position,joints[i+1].position); //determining link lengths by measuring the distance from one joint to another
            total_link_lengths += link_lengths[i];
        }

        Debug.Log(total_link_lengths);

    }

    void Update() //gets called every frame to run FABRIK
    {
        if (Target != null) IniFABRIK(Target.position);
        
    }

    void IniFABRIK(Vector2 Targetposition)
    {
        int n = joints.Length;
        //store current joint positions
        for (int i = 0; i < n; i++){
            positions[i] = joints[i].position;
        }

        float dist = Vector2.Distance(positions[0], Targetposition); //distance between root and target
        //check if target is reachable or not
        if (dist > total_link_lengths)//target is unreachable
        {
            for (int i = 0; i < n-1; i++)
            //find the distance r_i between the target and the joint position p_i
            {
                float r = Vector2.Distance(Targetposition, positions[i]);
                if (r == 0) r = 0.0001f;
                float lambda = link_lengths[i]/r;
                //find new joint positions p_i
                positions[i+1] = (1 - lambda)* positions[i] + lambda * Targetposition;
            }
        }
        else //the target is reachable so set base position as the initial position of the joint p_1
        {
            Vector2 arm_base = positions[0];
            // set iteration ABOVE for how many times we want to run FABRIK
            for (int iter = 0; iter < iteration; iter++)
            {
                //forward reaching
                //set the endeffector p_n as the target
                positions[n-1] = Targetposition;

                for (int i = n-2; i >= 0; i--)
                //find the distance r_i between the new joint position p_i and the joint p_i+1
                {
                    float r = Vector2.Distance(positions[i + 1], positions[i]);
                    if (r == 0) r = 0.0001f;
                    float lambda = link_lengths[i]/r;
                    //find the new joint positions p_i
                    positions[i] = (1 - lambda) * positions[i+1] + lambda * positions[i];
                }

                //backward reaching
                //set the root P_1 its initial position
                positions[0] = arm_base;
                //find the distance r_i between the new joint position p_i and the joint p_i+1
                for (int i = 0; i < n-1; i++)
                {
                    float r = Vector2.Distance(positions[i + 1], positions[i]);
                    if (r == 0) r = 0.0001f;
                    float lambda = link_lengths[i]/r;
                    //find the new joint positions p_i
                    positions[i+1] = (1 - lambda) * positions[i] + lambda * positions[i+1];
                }

                float dist_to_target = Vector2.Distance(positions[n-1], Targetposition);
                Debug.Log($"Iteration {iter + 1} | Distance to target: {dist_to_target}");
                //safet net: check whether the distance between the endeffector and the target is greater than our set tolerance
                if (Vector2.Distance(positions[n-1], Targetposition) < tolerance) break;
                
            }

        }
        //move base to the actual starting base
        joints[0].position = positions[0];

        //apply new positions back to the joints
        for (int i = 0; i < n - 1; i++)
        {
            Vector2 direction = positions[i+1] - positions[i];
            float angle = Mathf.Atan2(direction.y, direction.x) * Mathf.Rad2Deg;
            joints[i].rotation = Quaternion.Euler(0, 0, angle - 90f);
        }

        joints[n - 1].position = positions[n-1];

    }

    public Vector2 GetEndEffectorPosition()
    {
        return endEffector.position;
    }  
}


