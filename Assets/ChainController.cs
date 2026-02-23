using UnityEngine;

public class ChainController : MonoBehaviour
{
    public Transform joint1; //first joint starting from the bottom
    public Transform joint2; //second joint
    public Transform joint3; //joint closest to the end effector
    public Transform endEffector; //end of the chain
    public Vector3 jointAngles; // stores angles of the joints

    void Update() //gets called every frame to run ApplyJointAngles
    {
        ApplyJointAngles(jointAngles);
    }

    public void ApplyJointAngles(Vector3 angles) //gives new coordinates to the joints
    {
        joint1.eulerAngles = new Vector3(0f,0f, angles.x);
        joint2.eulerAngles = new Vector3(0f,0f, angles.y);
        joint3.eulerAngles = new Vector3(0f,0f, angles.z);
    }

    public Vector3 GetEndEffectorPosition()
    {
        return endEffector.position;
    }  
}
