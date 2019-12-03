using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using MLAgents;

public class VehicleAgent : Agent {

    [SerializeField]
    private Transform Goal;

    [SerializeField] 
    private SpawnAreaManager SpawnManager;

    [SerializeField]
    // Reward every "RewardInterval" units from "Goal"
    private float RewardInterval = 0.5f;

    [SerializeField]
    // Acceptable distance from "Goal" that is still a solution.
    private float DistanceTolerance = 2;

    [SerializeField]
    // Acceptable speed (from 0) that is still a solution.
    private float SpeedTolerance = 5f;

    [SerializeField]
    private float StepPenaltyInterval = 100f;

    [SerializeField]
    private Bounds WorldBounds;

    private float prevDistance;
    private float steps;
    private float distanceReward;

    private VehiclePhysics vehiclePhys;

    public override void InitializeAgent() {
        vehiclePhys = GetComponent<VehiclePhysics>();
        vehiclePhys.vehicleBody.centerOfMass = new Vector3(0f, 0.2f, 0.25f);
        this.distanceReward = 0f;
        vehiclePhys.distFromGoalX = 0f;
        vehiclePhys.distFromGoalZ = 0f;
    }

    public override void CollectObservations() {
        base.CollectObservations();

        // Agent position, y rotation, and velocity
        Vector3 normalizedVehiclePos = NormalizePosition(this.transform.position);
        AddVectorObs(vehiclePhys.vehicleBody.velocity.magnitude); 
        //AddVectorObs(normalizedVehiclePos.x); //enable for 2.0
        //AddVectorObs(normalizedVehiclePos.z); //enable for 2.0

        // Distances from goal
        Vector3 normalizedGoalPos = NormalizePosition(this.Goal.position);
        vehiclePhys.distFromGoalX = normalizedGoalPos.x - normalizedVehiclePos.x;
        vehiclePhys.distFromGoalZ = normalizedGoalPos.z - normalizedVehiclePos.z;
        AddVectorObs(normalizedGoalPos.x - normalizedVehiclePos.x);
        AddVectorObs(normalizedGoalPos.z - normalizedVehiclePos.z);

        Vector3 normalizedVehicleRot = NormalizeRotation(this.transform.rotation);
        // Rotatation.y specifies direction
        AddVectorObs(normalizedVehicleRot.y);

        foreach (ProximitySensor sensor in vehiclePhys.proxSensors) {
            // Distance from object normalized to [0, 1]
            AddVectorObs(NormalizeValue(sensor.UpdateSensor(this.transform), 0, sensor.distance));
        }
    }

    public override void AgentAction(float[] vectorAction, string textAction) {
        base.AgentAction(vectorAction, textAction);

        if (IsDone())
            return;

        // vectorAction[3]:
        //  0: Motor torque
        //  1: Steering
        //  2: Brake
        vehiclePhys.ApplyMotor(Mathf.Clamp(vectorAction[0], -1, 1), Mathf.Clamp(vectorAction[1], -1, 1), vectorAction[2]);
        //vehiclePhys.FixedUpdate();

        // Penalize at interval of steps
        if (++steps % StepPenaltyInterval == 0) {
            AddReward(-0.001f);
        }

        // Compute distance from goal
        float distanceFromGoal = Vector3.Distance(this.transform.position, Goal.transform.position);

        // Check for incremental reward/punishment
        if (distanceFromGoal < prevDistance) {
            // If vehicle is closer to goal by a meaningful amount (defined by RewardInterval)
            if ((int)(distanceFromGoal / (RewardInterval * 2)) < (int)(prevDistance / (RewardInterval * 2))) {
                AddReward(0.0035f); // Reward
                distanceReward += 0.0035f;
                prevDistance = distanceFromGoal;
            }
        }
        else {
            // If vehicle is farther from goal by a meaningful amount (defined by RewardInterval)
            if ((int)(distanceFromGoal / (RewardInterval * 2)) > (int)(prevDistance / (RewardInterval * 2))) {
                AddReward(-0.0035f); // Penalize
                distanceReward += -0.0035f;
                prevDistance = distanceFromGoal;
            }
        }

        // Check if goal state reached
        if (distanceFromGoal <= DistanceTolerance) {
            float reward = 1f;
            if (vehiclePhys.vehicleBody.velocity.magnitude > SpeedTolerance) {
                reward -= NormalizeValue(vehiclePhys.vehicleBody.velocity.magnitude, SpeedTolerance, 300);
            }
            Debug.Log("Vehicle reached goal! Rewarded with " + reward);
            AddReward(reward);
            Done();
            return;
        }

        // Check if vehicle left world bounds
        if (!WorldBounds.Contains(new Vector3Int((int)transform.position.x, (int)transform.position.y, (int)transform.position.z))) {
            Debug.Log("Vehicle escaped world bounds");
            AddReward(-1f);
            Done();
            return;
        }
    }

    public override void AgentReset() {
        Debug.Log("Done. Agent distance reward: " + distanceReward);
        // Reset vehicle and move to random location and rotation
        this.vehiclePhys.vehicleBody.velocity = Vector3.zero;
        this.vehiclePhys.vehicleBody.angularVelocity = Vector3.zero;
        this.transform.position = SpawnManager.getSpawnPoint();
        this.transform.rotation = Quaternion.Euler(0, Random.Range(0f, 360f), 0);
        this.vehiclePhys.Start();

        this.Goal.transform.position = SpawnManager.getSpawnPoint();
        Debug.Log("Spawned vehicle at " + this.transform.position + ". goal at " + this.Goal.transform.position);
        
        this.prevDistance = Vector3.Distance(this.transform.position, Goal.transform.position);
        this.steps = 0;

        if (this.Goal.position != this.Goal.transform.position) {
            Debug.Log("Logic error");
        }

        this.distanceReward = 0f;
        Vector3 normalizedVehiclePos = NormalizePosition(this.transform.position);
        Vector3 normalizedGoalPos = NormalizePosition(this.Goal.position);
        vehiclePhys.distFromGoalX = normalizedGoalPos.x - normalizedVehiclePos.x;
        vehiclePhys.distFromGoalZ = normalizedGoalPos.z - normalizedVehiclePos.z;
    }

    void OnCollisionEnter(Collision collision) {
        if (collision.collider.gameObject.CompareTag("Obstacle")) {
            Debug.Log("Vehicle collided with obstacle");
            AddReward(-0.5f);
            Done();
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

    void OnDrawGizmos() {
        Gizmos.color = Color.blue;
        Gizmos.DrawWireCube(WorldBounds.center, WorldBounds.size);
    }
}