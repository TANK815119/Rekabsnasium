using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;
/*
 * Simplified version of Drone_3D target that gives ideal pitches, rolls, yaws, and thrusts
 * instead of the raw inputs which may be difficult for the networkt to navigate.
 * A Proportional-Integral-Derrivative(PID) will be used to reach those ideal values. 
 * Remember Unity has a left-handed coordinate system/
 */

public class Drone_3DTarget_Functional : Agent
{
    [SerializeField] private Transform target;
    //[SerializeField] private float maxDistance = 25f;

    [SerializeField] private Transform FRProp;
    [SerializeField] private Transform BRProp;
    [SerializeField] private Transform FLProp;
    [SerializeField] private Transform BLProp;
    private Transform[] propTrans;

    [SerializeField] private float maxThrust = 15f;
    [SerializeField] private float damper = 0.15f;
    [SerializeField] private float maxYawOmega = 7f;
    [SerializeField] private float axesThrustAlotment = 5f;

    private float idealLift;
    private float idealPitch;
    private float idealRoll;
    private float idealYawOmega;

    private Rigidbody droneBody;
    private float time = 0f;

    public void Awake()
    {
        droneBody = this.GetComponent<Rigidbody>();
        droneBody.maxAngularVelocity = 999f;
        propTrans = new Transform[] { FRProp, BRProp, FLProp, BLProp };
    }

    public void ResetDrone()
    {
        time = 0f; //reset clock
        idealLift = 0f; //reset lift
        idealPitch = 0f; //reset pitch
        idealRoll = 0f; //reset roll
        idealYawOmega = 0f; //reset yaw;

        ResetPhysics();
    }

    private void ResetPhysics()
    {
        // 1. Reset position and rotation
        transform.localPosition = Vector3.zero;
        transform.localRotation = Quaternion.identity;

        // 2. Reset Rigidbody's linear and angular velocity
        droneBody.velocity = Vector3.zero;
        droneBody.angularVelocity = Vector3.zero;

        // 3. Force sleep after resetting velocities (prevents unexpected waking)
        droneBody.Sleep();

        // 4. Disable and re-enable the Rigidbody to force re-initialization (Optional, but powerful)
        droneBody.isKinematic = true;  // Disable physics for this frame
    }

    public override void CollectObservations(VectorSensor sensor) //TOTAL: 17
    {
        // 1️ Relative position to target (local)
        Vector3 relativePosition = transform.InverseTransformPoint(target.position);
        sensor.AddObservation(relativePosition); // 3 values

        // 2 Alternative for yaw (use sin/cos instead)
        float relativeYaw = Mathf.DeltaAngle(transform.eulerAngles.y, target.eulerAngles.y) * Mathf.Deg2Rad;
        sensor.AddObservation(Mathf.Sin(relativeYaw)); // 1 value
        sensor.AddObservation(Mathf.Cos(relativeYaw)); // 1 value

        // 3️ Local velocity
        Vector3 localVelocity = transform.InverseTransformDirection(droneBody.velocity);
        sensor.AddObservation(localVelocity); // 3 values

        // 4️ Forward and Up vectors for orientation
        Vector3 forward = transform.forward;
        Vector3 up = transform.up;
        sensor.AddObservation(forward); // 3 values
        sensor.AddObservation(up); // 3 values

        // 5️ Local angular velocity
        Vector3 localAngularVelocity = transform.InverseTransformDirection(droneBody.angularVelocity);
        sensor.AddObservation(localAngularVelocity); // 3 values
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        idealLift = actions.ContinuousActions[0]; //ideaLift
        idealPitch = actions.ContinuousActions[1]; //idealPitch
        idealRoll = actions.ContinuousActions[2]; //idealRoll
        idealYawOmega = actions.ContinuousActions[3]; //ideaLaw

        //modulate idealLift between 0 and 15f with f(0) = 9.81
        if (idealLift <= 0)
        {
            idealLift = Mathf.Lerp(0, 9.81f, idealLift + 1); // gravity compensation for negative thrust
        }
        else
        {
            idealLift = Mathf.Lerp(9.81f, maxThrust, idealLift); // regular thrust for positive values
        }

        //modulate ideal yaw for ptich and roll to be within -90f and 90f so the drone doesnt flip
        idealPitch = idealPitch * 90f;
        idealRoll = idealRoll * 90f;

        idealYawOmega = idealYawOmega * maxYawOmega;
    }

    public void FixedUpdate()
    {
        if (time >= 0.1f && droneBody.isKinematic)
        {
            droneBody.isKinematic = false; // Re-enable physics
        }

        //turn the outputs intoactual thrust values
        float[] thrusts = ThrustPID();

        //apply thrusts
        for (int i = 0; i < propTrans.Length; i++)
        {
            //droneBody.AddForce(Vector3.up * thrust * Time.fixedDeltaTime, ForceMode.Impulse);
            Vector3 localThrust = Vector3.up * thrusts[i];
            Vector3 globalThrust = propTrans[i].TransformDirection(localThrust);
            droneBody.AddForceAtPosition(globalThrust * Time.fixedDeltaTime, propTrans[i].position, ForceMode.Impulse);
        }

        //yaw torque added artificially
        float torque = CalculateTorque(thrusts);
        droneBody.AddRelativeTorque(Vector3.up * torque * Time.fixedDeltaTime, ForceMode.Impulse);
    }

    private float[] ThrustPID()
    {
        float[] thrusts = new float[] { 0f, 0f, 0f, 0f }; //FR, BR, FL, BL, torque
        Vector3 localAngularVelocity = transform.InverseTransformDirection(droneBody.angularVelocity);

        //first, calculate the proportion for pitch and roll
        float pitchError = AngularError(droneBody.rotation.eulerAngles.x, idealPitch);
        float pitchDerr = -localAngularVelocity.x * Mathf.Rad2Deg;
        float pitchProp = (pitchError - pitchDerr * damper) / 90f;

        float rollError = AngularError(droneBody.rotation.eulerAngles.z, idealRoll);
        float rollDerr = -localAngularVelocity.z * Mathf.Rad2Deg;
        float rollProp = (rollError - rollDerr * damper) / 90f;

        //second, modulate yaw-ing on top of the simple pitch and roll values
        float yawError = -localAngularVelocity.y - idealYawOmega; // Error in yaw rate 
        float yawProp = -yawError / 4f; //-yawError / 90f;

        //set the thrust of all props equal for correct lift
        for (int i = 0; i < thrusts.Length; i++)
        {
            thrusts[i] = idealLift / 4f;
        }

        //make sure the total never excedes 1f and is proportional to error, not other forces
        float pitchAdj = pitchProp * axesThrustAlotment;
        float rollAdj = rollProp * axesThrustAlotment;
        float yawAdj = yawProp * axesThrustAlotment;

        thrusts[0] += pitchAdj + -rollAdj + -yawAdj; //FR
        thrusts[1] += -pitchAdj + -rollAdj + yawAdj; //BR
        thrusts[2] += pitchAdj + rollAdj + yawAdj; //FL
        thrusts[3] += -pitchAdj + rollAdj + -yawAdj; //BL

        //normalize thrusts
        for (int i = 0; i < thrusts.Length; i++)
        {
            thrusts[i] = Mathf.Clamp(thrusts[i], 0f, maxThrust / 4f);
        }
        return thrusts;
    }

    private float AngularError(float realAngle, float idealAngle)
    {
        float angleError = 0f;
        if (realAngle <= 180f)
        {
            angleError = realAngle - idealAngle;
        }
        else
        {
            angleError = (realAngle - 360f) - idealAngle;
        }
        return angleError;
    }

    private float CalculateTorque(float[] thrusts)
    {
        //torque addition cause why not; i have no other good place to put it
        float propDist = Mathf.Sqrt(Mathf.Pow(propTrans[0].localPosition.x, 2f) + Mathf.Pow(propTrans[0].localPosition.z, 2f));
        float torque = propDist * ((thrusts[0] + thrusts[3]) - (thrusts[2] + thrusts[1])); //(FR+BL) - (FL+BR) for left hand rule(not rihgt)

        return torque;
    }

    public void Update()
    {
        //little clock
        time += Time.deltaTime;
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continousActions = actionsOut.ContinuousActions;
        continousActions[0] = 0f;
        continousActions[1] = 0f;
        continousActions[2] = 0f;
        continousActions[3] = 0f;
        if (Input.GetKey(KeyCode.W)) //front
        {
            continousActions[1] += 0.2f;
        }
        if (Input.GetKey(KeyCode.S)) //back
        {
            continousActions[1] += -0.2f;
        }
        if (Input.GetKey(KeyCode.A)) //left
        {
            continousActions[2] += 0.2f;
        }
        if (Input.GetKey(KeyCode.D)) //right
        {
            continousActions[2] += -0.2f;
        }
        if (Input.GetKey(KeyCode.E)) //turn right
        {
            continousActions[3] += -1.0f;
        }
        if (Input.GetKey(KeyCode.Q)) //turn left
        {
            continousActions[3] += 1.0f;
        }
        if (Input.GetKey(KeyCode.Space)) //up
        {
            continousActions[0] += 0.5f;
        }
        if (Input.GetKey(KeyCode.LeftShift)) //down
        {
            continousActions[0] += -0.5f;
        }
    }

    private string ArrayToString(float[] arr)
    {
        string str = "";
        for (int i = 0; i < arr.Length; i++)
        {
            str += arr[i] + ", ";
        }
        return str;
    }
}