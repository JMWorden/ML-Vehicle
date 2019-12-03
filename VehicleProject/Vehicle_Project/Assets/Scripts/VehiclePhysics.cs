using UnityEngine;
using System.Collections;
using System.Collections.Generic;

[System.Serializable]
public class AxleInfo {
    public WheelCollider leftWheel;
    public WheelCollider rightWheel;
    public bool motor;
    public bool steering;
}

[System.Serializable]
public class ProximitySensor {
    public float distance;
    public Vector3 direction;

    public float UpdateSensor(Transform origin) {
        RaycastHit hit;
        Vector3 relativeDirection = origin.TransformVector(direction);
        relativeDirection.y = direction.y;
        if (Physics.Raycast(origin.position, relativeDirection.normalized, out hit, distance)) {
            Debug.DrawRay(origin.position, relativeDirection.normalized * hit.distance, Color.red);
        }
        else {
            Debug.DrawRay(origin.position, relativeDirection.normalized * distance, Color.green);
            return distance;
        }
        return hit.distance;
    }
}
     
public class VehiclePhysics : MonoBehaviour {
    public List<AxleInfo> axleInfos;
    public List<ProximitySensor> proxSensors;

    public float maxMotorTorque;
    public float maxSteeringAngle;
    public float maxBrakeTorque;
    public Rigidbody vehicleBody;

    private float motorToApply;
    private float brakeToApply;
    private float steeringToApply;

    public float distFromGoalX;
    public float distFromGoalZ;

    public void Start() {
        vehicleBody = this.GetComponent<Rigidbody>();
        motorToApply = 0;
        steeringToApply = 0;
        brakeToApply = 0;
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

    public void ApplyMotor(float applyTorque, float applySteering, float applyBrake) {
        if (applyBrake > 0f) {
            brakeToApply = maxBrakeTorque;
        }
        else {
            brakeToApply = 0;
        }

        // Vehicle moves slower in reverse
        if (applyTorque >= 0f) {
            motorToApply = maxMotorTorque * applyTorque;
        }
        else {
            motorToApply = -1500;
        }
        steeringToApply = maxSteeringAngle * applySteering;
    }

    public void FixedUpdate() {
        foreach (AxleInfo axleInfo in axleInfos) {
            if (axleInfo.steering) {
                axleInfo.leftWheel.steerAngle = steeringToApply;
                axleInfo.rightWheel.steerAngle = steeringToApply;
            }
            if (axleInfo.motor) {
                axleInfo.leftWheel.motorTorque = motorToApply;
                axleInfo.rightWheel.motorTorque = motorToApply;
            }
            axleInfo.leftWheel.brakeTorque = brakeToApply;
            axleInfo.rightWheel.brakeTorque = brakeToApply;
            ApplyLocalPositionToVisuals(axleInfo.leftWheel);
            ApplyLocalPositionToVisuals(axleInfo.rightWheel);
        }
    }

    void OnGUI() {
        GUI.Label(new Rect(0,0,Screen.width,Screen.height),"Motor: " + motorToApply
            + "\nSteering: " + steeringToApply + "\nBrake: " + brakeToApply + "\n\nD(x): " + distFromGoalX + "\nD(z): " + distFromGoalZ);
    }
}