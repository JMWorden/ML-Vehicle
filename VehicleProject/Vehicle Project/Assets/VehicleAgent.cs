using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MLAgents;

[System.Serializable]
public class AxleInfo {
    public WheelCollider leftWheel;
    public WheelCollider rightWheel;
    public bool motor;
    public bool steering;
}

public class VehicleAgent : Agent
{
    public List<AxleInfo> axleInfos; 
    public float maxMotorTorque;
    public float maxSteeringAngle;
    RigidBody vehicleBody;

    // Start is called before the first frame update
    void Start()
    {
        vehicleBody = GetComponent<RigidBody>();
    }

    // finds the corresponding visual wheel
    // correctly applies the transform
    public void ApplyLocalPositionToVisuals(WheelCollider collider)
    {
        if (collider.transform.childCount == 0) {
            return;
        }
     
        Transform visualWheel = collider.transform.GetChild(0);
     
        Vector3 position;
        Quaternion rotation;
        collider.GetWorldPose(out position, out rotation);
     
        visualWheel.transform.position = position;
        visualWheel.transform.rotation = rotation;
    }

    public void FixedUpdate(float percentTorque, float percentSteer)
    {
        float motor = maxMotorTorque * percentTorque;
        float steering = maxSteeringAngle * percentSteer;
     
        foreach (AxleInfo axleInfo in axleInfos) {
            if (axleInfo.steering) {
                axleInfo.leftWheel.steerAngle = steering;
                axleInfo.rightWheel.steerAngle = steering;
            }
            if (axleInfo.motor) {
                axleInfo.leftWheel.motorTorque = motor;
                axleInfo.rightWheel.motorTorque = motor;
            }
            ApplyLocalPositionToVisuals(axleInfo.leftWheel);
            ApplyLocalPositionToVisuals(axleInfo.rightWheel);
        }
    }

    public Transform Target;

    public override void AgentReset() {
        // TODO
        if (false) {
            this.transform.position = new Vector3(0, 0, 0);
        }

        Target.position = new Vector3(Random.value * 8 - 4,
                                      0,
                                      Random.value * 8 -4);
    }

    public override void CollectObservations() {
        // TODO : Add proximity

        // Distance to Target
        AddVectorObs(Vector3.Distance(this.transform.position, Target.position));

        // Vehicle velocity
        AddVectorObs(vehicleBody.velocity.x);
        AddVectorObs(vehicleBody.velocity.z);
    }

    public float speed = 10; // What is this?

    public override void AgentAction(float[] vectorAction, string textAction) {
        // vectorAction[0] - accelaration
        // vectorAction[1] - turning angle
        FixedUpdate(vectorAction[0], vectorAction[1]);

        // Rewards
        float distanceToTarget = Vector3.Distance(this.transform.position, Target.position)
        
        // Reached target
        if (distanceToTarget < 1.42f) {
            SetReward(1.0f);
            Done();
        }

        // TODO : Timeout?
    }
}