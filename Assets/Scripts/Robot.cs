using System;
using UnityEngine;
using UnityEngine.InputSystem;


[Serializable]
public class WheelProperties
{
    public int wheelState = 1;  // 1 = steerable wheel, 0 = free wheel
    [HideInInspector] public float biDirectional = 0; // optional advanced usage
    public Vector3 localPosition;        // wheel anchor in the car's local space
    public float turnAngle = 30f;        // max steer angle for this wheel

    [HideInInspector] public float lastSuspensionLength = 0.0f;
    [HideInInspector] public Vector3 localSlipDirection;
    [HideInInspector] public Vector3 worldSlipDirection;
    [HideInInspector] public Vector3 suspensionForceDirection;
    [HideInInspector] public Vector3 wheelWorldPosition;
    [HideInInspector] public float wheelCircumference;
    [HideInInspector] public float torque = 0.0f;
    [HideInInspector] public Rigidbody parentRigidbody;
    [HideInInspector] public GameObject wheelObject;
    [HideInInspector] public float hitPointForce;
    [HideInInspector] public Vector3 localVelocity;
}

public class Robot : MonoBehaviour
{
    [Header("Wheel Setup")]
    public GameObject wheelPrefab;
    public WheelProperties[] wheels;
    public float wheelSize = 0.53f;        // radius of the wheel
    public float maxTorque = 450f;         // maximum engine torque
    public float wheelGrip = 12f;          // how strongly it resists sideways slip
    public float maxGrip = 12f;          // how strongly it resists sideways slip
    public float frictionCoWheel = 0.022f; // rolling friction

    [Header("Suspension")]
    public float suspensionForce = 90f;       // spring constant
    public float dampAmount = 2.5f;           // damping constant
    public float suspensionForceClamp = 200f; // cap on total suspension force

    [Header("Robot Mass")]
    public float massInKg = 100f; // (not strictly used, but you might incorporate it)

    // These are updated each frame
    [HideInInspector] public Vector2 input = Vector2.zero;  // horizontal=steering, vertical=gas/brake
    [HideInInspector] public bool Forwards = false;

    private Rigidbody rb;

    [Header("Control")]
    public bool usePlayerInput = true;

    [Header("Differential Steering")]
    public float turnSpeed = 1.5f;
    public float minTurnSpeed = 0.25f;


    public Vector3 Velocity => rb.linearVelocity;
    public Vector3 AngularVelocity => rb.angularVelocity;

    [Header("Sensors")]
    public Transform[] sensors;
    public Transform[] Sensors => sensors;
    public float rayDistance = 40f;

    void Start()
    {
        // Grab or add a Rigidbody
        rb = GetComponent<Rigidbody>();
        if (!rb) rb = gameObject.AddComponent<Rigidbody>();

        // Slight tweak to inertia if desired
        rb.inertiaTensor = 1.0f * rb.inertiaTensor;

        // Create each wheel
        if (wheels != null)
        {
            for (int i = 0; i < wheels.Length; i++)
            {
                WheelProperties w = wheels[i];

                // Convert localPosition consistently
                Vector3 parentRelativePosition = transform.InverseTransformPoint(transform.TransformPoint(w.localPosition));
                w.localPosition = parentRelativePosition;

                // Instantiate the visual wheel
                w.wheelObject = Instantiate(wheelPrefab, transform);
                w.wheelObject.transform.localPosition = w.localPosition;
                w.wheelObject.transform.eulerAngles   = transform.eulerAngles;
                w.wheelObject.transform.localScale    = 2f * new Vector3(wheelSize, wheelSize, wheelSize);

                // Calculate wheel circumference for rotation logic
                w.wheelCircumference = 2f * Mathf.PI * wheelSize;

                w.parentRigidbody = rb;
            }
        }

        // Get raycast origin points
        Transform raycastsParent = transform.Find("raycasts");

        if (raycastsParent != null)
        {
            sensors = new Transform[raycastsParent.childCount];

            for (int i = 0; i < raycastsParent.childCount; i++)
            {
                sensors[i] = raycastsParent.GetChild(i);
            }
        }
        else
        {
            sensors = new Transform[0];
        }

    }

    void Update()
    {
        // Gather inputs
        if (usePlayerInput)
        {
            input = InputSystem.actions["Move"].ReadValue<Vector2>();
        }
    }

    void FixedUpdate()
    {
        if (wheels == null || wheels.Length == 0) return;

        float steering = Mathf.Clamp(input.x, -1f, 1f);
        float throttle = Mathf.Clamp(input.y, -1f, 1f);

        // Differential steering
        float turn = -steering * turnSpeed;

        float leftTorque = throttle - turn;
        float rightTorque = throttle + turn;

        foreach (var wheel in wheels)
        {
            if (!wheel.wheelObject) continue;

            Transform wheelObj = wheel.wheelObject.transform;
            Transform wheelVisual = wheelObj.GetChild(0);

            // Determine wheel side using local X position
            float sideTorque = wheel.localPosition.x < 0f
                ? leftTorque
                : rightTorque;

            // Keep wheels facing forward
            if (wheel.wheelState == 1)
            {
                Quaternion targetRot = Quaternion.identity;

                wheelObj.localRotation = Quaternion.Lerp(
                    wheelObj.localRotation,
                    targetRot,
                    Time.fixedDeltaTime * 100f
                );
            }
            else if (wheel.wheelState == 0 &&
                    rb.linearVelocity.magnitude > 0.04f)
            {
                RaycastHit tmpHit;

                if (Physics.Raycast(
                    transform.TransformPoint(wheel.localPosition),
                    -transform.up,
                    out tmpHit,
                    wheelSize * 2f))
                {
                    Quaternion aim = Quaternion.LookRotation(
                        rb.GetPointVelocity(tmpHit.point),
                        transform.up
                    );

                    wheelObj.rotation = Quaternion.Lerp(
                        wheelObj.rotation,
                        aim,
                        Time.fixedDeltaTime * 100f
                    );
                }
            }

            wheel.wheelWorldPosition =
                transform.TransformPoint(wheel.localPosition);

            Vector3 velocityAtWheel =
                rb.GetPointVelocity(wheel.wheelWorldPosition);

            wheel.localVelocity =
                wheelObj.InverseTransformDirection(velocityAtWheel);

            // Differential torque
            wheel.torque = Mathf.Clamp(
                sideTorque,
                -1f,
                1f
            ) * maxTorque / massInKg;

            // Rolling friction
            float rollingFrictionForce =
                -frictionCoWheel * wheel.localVelocity.z;

            // Lateral friction
            float lateralFriction =
                -wheelGrip * wheel.localVelocity.x;

            lateralFriction = Mathf.Clamp(
                lateralFriction,
                -maxGrip,
                maxGrip
            );

            // Engine force
            float engineForce = wheel.torque / wheelSize;

            Vector3 totalLocalForce = new Vector3(
                lateralFriction,
                0f,
                rollingFrictionForce + engineForce
            );

            wheel.localSlipDirection = totalLocalForce;

            Vector3 totalWorldForce =
                wheelObj.TransformDirection(totalLocalForce);

            wheel.worldSlipDirection = totalWorldForce;

            Forwards = wheel.localVelocity.z > 0f;

            // Suspension
            RaycastHit hit;

            if (Physics.Raycast(
                wheel.wheelWorldPosition,
                -transform.up,
                out hit,
                wheelSize * 2f))
            {
                float rayLen = wheelSize * 2f;
                float compression = rayLen - hit.distance;

                float damping =
                    (wheel.lastSuspensionLength - hit.distance)
                    * dampAmount;

                float springForce =
                    (compression + damping) * suspensionForce;

                springForce = Mathf.Clamp(
                    springForce,
                    0f,
                    suspensionForceClamp
                );

                Vector3 springDir = hit.normal * springForce;

                wheel.suspensionForceDirection = springDir;

                rb.AddForceAtPosition(
                    springDir + totalWorldForce,
                    hit.point
                );

                wheelObj.position =
                    hit.point + transform.up * wheelSize;

                wheel.lastSuspensionLength = hit.distance;
            }
            else
            {
                wheelObj.position =
                    wheel.wheelWorldPosition - transform.up * wheelSize;
            }

            // Visual wheel rotation
            Vector3 forwardInWheelSpace =
                wheelObj.InverseTransformDirection(
                    rb.GetPointVelocity(wheel.wheelWorldPosition)
                );

            float wheelRotationSpeed =
                forwardInWheelSpace.z * 360f / wheel.wheelCircumference;

            wheelVisual.Rotate(
                Vector3.right,
                wheelRotationSpeed * Time.fixedDeltaTime,
                Space.Self
            );
        }
    }

    void OnDrawGizmos()
    {
        if (wheels == null) return;

        foreach (var wheel in wheels)
        {
            // Mark the wheel center
            Gizmos.color = Color.green;
            Gizmos.DrawSphere(wheel.wheelWorldPosition, 0.08f);

            // Suspension force
            if (wheel.suspensionForceDirection != Vector3.zero)
            {
                Gizmos.color = Color.blue;
                Gizmos.DrawLine(
                    wheel.wheelWorldPosition,
                    wheel.wheelWorldPosition + wheel.suspensionForceDirection * 0.01f
                );
            }

            // Slip/friction force
            if (wheel.worldSlipDirection != Vector3.zero)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawLine(
                    wheel.wheelWorldPosition,
                    wheel.wheelWorldPosition + wheel.worldSlipDirection * 0.01f
                );
            }
        }

        if (sensors == null) return;

        foreach (Transform sensor in sensors)
        {
            if (sensor == null) continue;

            Vector3 origin = sensor.position;
            Vector3 direction = sensor.forward;

            if (Physics.Raycast(origin, direction, out RaycastHit hit, rayDistance))
            {
                Gizmos.color = Color.red;
                Gizmos.DrawLine(origin, hit.point);
                Gizmos.DrawSphere(hit.point, 0.08f);
            }
            else
            {
                Gizmos.color = Color.green;
                Gizmos.DrawLine(origin, origin + direction * rayDistance);
            }
        }

    }

    public void SetAgentInput(float steering, float throttle)
    {
        input = new Vector2(
            Mathf.Clamp(steering, -1f, 1f),
            Mathf.Clamp(throttle, -1f, 1f)
        );
    }

    public void Reset()
    {
        rb.position = transform.position;
        rb.rotation = transform.rotation;

        rb.linearVelocity = Vector3.zero;
        rb.angularVelocity = Vector3.zero;
    }

}