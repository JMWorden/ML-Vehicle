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
        if (Physics.Raycast(origin.position, direction.normalized, out hit, distance)) {
            Debug.DrawRay(origin.position, direction.normalized * hit.distance, Color.red);
        }
        else {
            Debug.DrawRay(origin.position, direction.normalized * distance, Color.green);
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
    public Rigidbody vehicleBody;
     
    public void Start() {
        vehicleBody = this.GetComponent<Rigidbody>();
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

    public void ApplyMotor(float applyTorque, float applySteering) {
        float motor = maxMotorTorque * applyTorque;
        float steering = maxSteeringAngle * applySteering;
     
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

    public void Update() {
        foreach (ProximitySensor sensor in proxSensors) {
            Debug.Log(sensor.UpdateSensor(this.transform));
        }
    }
     
    /* 
    public void FixedUpdate()
    {
        float motor = maxMotorTorque * Input.GetAxis("Vertical");
        float steering = maxSteeringAngle * Input.GetAxis("Horizontal");
     
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
    */
}