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

        foreach (ProximitySensor sensor in vehiclePhys.proxSensors) {
            // Distance from object normalized to [0, 1]
            AddVectorObs(NormalizeValue(sensor.UpdateSensor(this.transform), 0, sensor.getMaxDistance()));
        }
    }

    public override void AgentAction(float[] vectorAction, string textAction) {
        base.AgentAction(vectorAction, textAction);

        if (IsDone())
            return;

        // vectorAction[2]:
        //  0: Motor torque
        //  1: Steering
        vehiclePhys.ApplyMotor(Mathf.Clamp(vectorAction[0], -1, 1), Mathf.Clamp(vectorAction[1], -1, 1));
        //Debug.Log("[" + vectorAction[0] + ", " + vectorAction[1] + "] [" + Mathf.Clamp(vectorAction[0], -1, 1) + ", " + Mathf.Clamp(vectorAction[1], -1, 1) + "]");

        // Compute distance from goal
        float distanceFromGoal = Vector3.Distance(this.transform.position, Goal.transform.position);

        // Check for incremental reward/punishment
        if (distanceFromGoal < prevDistance) {
            // If vehicle is closer to goal by a meaningful amount (defined by RewardInterval)
            if ((int)(distanceFromGoal / (RewardInterval * 2)) < (int)(prevDistance / (RewardInterval * 2))) {
                AddReward(0.02f); // Reward
                prevDistance = distanceFromGoal;
            }
        }
        else {
            // If vehicle is farther from goal by a meaningful amount (defined by RewardInterval)
            if ((int)(distanceFromGoal / (RewardInterval * 2)) > (int)(prevDistance / (RewardInterval * 2))) {
                AddReward(-0.04f); // Penalize
                prevDistance = distanceFromGoal;
            }
        }

        // Check if goal state reached
        if (distanceFromGoal <= DistanceTolerance) {
            if (vehiclePhys.vehicleBody.velocity.magnitude <= SpeedTolerance) {
                // TODO : Reward for speed
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

    public override void AgentReset() {
        this.transform.position = new Vector3(0, 0, 0);
        this.vehiclePhys.vehicleBody.velocity = Vector3.zero;
        this.vehiclePhys.vehicleBody.angularVelocity = Vector3.zero;
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

    // Normalizes val to [0, 1]
    private float NormalizeValue(float val, float min, float max) {
        return (val - min)/(max - min);
    }
}