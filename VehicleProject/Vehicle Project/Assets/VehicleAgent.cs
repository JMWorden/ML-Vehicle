using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MLAgents;

public class VehicleAgent : Agent {

    [SerializeField]
    private Transform Goal;

    [SerializeField]
    // Reward every "RewardInterval" units from "Goal"
    private float RewardInterval = 3f;

    [SerializeField]
    // Acceptable distance from "Goal" that is still a solution.
    private float DistanceTolerance = 2;

    [SerializeField]
    // Acceptable speed (from 0) that is still a solution.
    private float SpeedTolerance = 5f;

    [SerializeField]
    private Bounds WorldBounds;

    private float prevDistance;

    private VehiclePhysics vehiclePhys;

    public void Start() {
        vehiclePhys = GetComponent<VehiclePhysics>();
        prevDistance = Vector3.Distance(this.transform.position, Goal.transform.position);
    }

    public override void CollectObservations() {
        base.CollectObservations();

        // Agent position, y rotation, and velocity
        Vector3 normalizedVehiclePos = NormalizePosition(this.transform.position);
        AddVectorObs(vehiclePhys.vehicleBody.velocity.magnitude); 
        AddVectorObs(normalizedVehiclePos.x);
        AddVectorObs(normalizedVehiclePos.z);

        Vector3 normalizedVehicleRot = NormalizeRotation(this.transform.rotation);
        // Rotatation.y specifies direction
        AddVectorObs(normalizedVehicleRot.y);

        // Rotation.x, Rotation.z will specify if car is flipped over
        AddVectorObs(normalizedVehicleRot.z);
        AddVectorObs(normalizedVehicleRot.x);

        // TODO : Add
        /*
        foreach (ProximitySensor sensor in proxSensors) {
            sensor.UpdateSensor();
            AddVectorObs(sensor.NormalizedDistance);
        }
        */

        foreach (ProximitySensor sensor in vehiclePhys.proxSensors) {
            AddVectorObs(sensor.UpdateSensor(this.transform));
        }
    }

    public override void AgentAction(float[] vectorAction, string textAction) {
        base.AgentAction(vectorAction, textAction);

        if (IsDone())
            return;

        VehiclePhysics vehiclePhys = GetComponent<VehiclePhysics>();

        // vectorAction[2]:
        //  0: Motor torque
        //  1: Steering
        // TODO : Normalize to [-1, 1]
        vehiclePhys.ApplyMotor(vectorAction[0], vectorAction[1]);

        // Check for incremental reward
        float distanceFromGoal = Vector3.Distance(this.transform.position, Goal.transform.position);
        if (distanceFromGoal < prevDistance) {
            // If vehicle is closer to goal by a meaningful amount (defined by RewardInterval)
            if ((int)(distanceFromGoal / (RewardInterval * 2)) < (int)(prevDistance / (RewardInterval * 2))) {
                AddReward(0.02f); // Reward
                prevDistance = distanceFromGoal;
            }
        }
        else {
            // If vehicle is father from goal by a meaningful amount (defined by RewardInterval)
            if ((int)(distanceFromGoal / (RewardInterval * 2)) > (int)(prevDistance / (RewardInterval * 2))) {
                AddReward(-0.04f); // Penalize
                prevDistance = distanceFromGoal;
            }
        }

        // Check if goal state reached
        if (distanceFromGoal <= DistanceTolerance) {
            if (vehiclePhys.vehicleBody.velocity.magnitude <= SpeedTolerance) {
                AddReward(1f);
                Done();

                return;
            }
        }

        // Check if vehicle left world bounds
        if (!WorldBounds.Contains(new Vector3Int((int)transform.position.x, (int)transform.position.y, (int)transform.position.z))) {
            AddReward(-1f);
            Done();
            return;
        }
    }

    void OnCollisionEnter(Collision collision) {
        if (collision.collider.gameObject.CompareTag("Obstacle")) {
            AddReward(-0.12f);
        }
    }

    private Vector3 NormalizePosition(in Vector3 pos) {
        float x = NormalizeValue(pos.x, WorldBounds.min.x, WorldBounds.max.x);
        float y = NormalizeValue(pos.y, WorldBounds.min.y, WorldBounds.max.y);
        float z = NormalizeValue(pos.z, WorldBounds.min.z, WorldBounds.max.z);

        return new Vector3(x, y, z);
    }

    private Vector3 NormalizeRotation(in Quaternion rot) {
        float x = NormalizeValue(rot.eulerAngles.x, 0, 360);
        float y = NormalizeValue(rot.eulerAngles.y, 0, 360);
        float z = NormalizeValue(rot.eulerAngles.z, 0, 360);

        return new Vector3(x, y, z);
    }

    private float NormalizeValue(float val, float min, float max) {
        return (val - min)/(max - min);
    }
}

/* 
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
        float distanceToTarget = Vector3.Distance(this.transform.position, Target.position);
        
        // Reached target
        if (distanceToTarget < 1.42f) {
            SetReward(1.0f);
            Done();
        }

        // TODO : Timeout?
    }
}
*/