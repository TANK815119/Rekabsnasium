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

public class Drone_3DTarget_Smooth : Agent
{
    [SerializeField] private Transform target;
    [SerializeField] private float episodeLength = 15f;

    [SerializeField] private Transform FRProp;
    [SerializeField] private Transform BRProp;
    [SerializeField] private Transform FLProp;
    [SerializeField] private Transform BLProp;
    private Transform[] propTrans;

    [SerializeField] private float maxThrust = 15f;
    [SerializeField] private float damper = 0.15f;
    [SerializeField] private float maxYawOmega = 7f;
    [SerializeField] private float axesThrustAlotment = 1f;

    //private float[] thrusts;
    //private float torque;

    private float idealLift;
    private float idealPitch;
    private float idealRoll;
    private float idealYawOmega;

    //for derrivatives
    //private float lastPitch;
    //private float lastRoll;
    //private float lastYaw;

    private Rigidbody droneBody;
    private float time = 0f;
    public void Awake()
    {
        droneBody = this.GetComponent<Rigidbody>();
        droneBody.maxAngularVelocity = 999f;
        propTrans = new Transform[] { FRProp, BRProp, FLProp, BLProp };
        //thrusts = new float[] { 0f, 0f, 0f, 0f }; //FR, BR, FL, BL
    }

    public override void OnEpisodeBegin()
    {
        time = 0f; //reset clock
        idealLift = 0f; //reset lift
        idealPitch = 0f; //reset pitch
        idealRoll = 0f; //reset roll
        idealYawOmega = 0f; //reset yaw;

        ResetDrone();

        //make new target position
        target.localPosition = new Vector3(Random.Range(-25f, 25f), Random.Range(-25f, 25f), Random.Range(-25f, 25f));
        target.rotation = Quaternion.Euler(0f, Random.Range(-360f, 360f), 0f);
    }

    private void ResetDrone()
    {
        // 1. Reset position and rotation
        transform.localPosition = Vector3.zero;
        transform.localRotation = Quaternion.identity;

        // 2. Reset Rigidbody's linear and angular velocity
        droneBody.linearVelocity = Vector3.zero;
        droneBody.angularVelocity = Vector3.zero;

        // 3. Force sleep after resetting velocities (prevents unexpected waking)
        droneBody.Sleep();

        // 4. Disable and re-enable the Rigidbody to force re-initialization (Optional, but powerful)
        droneBody.isKinematic = true;  // Disable physics for this frame
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.position); //3 inputs
        sensor.AddObservation(target.position); //3 inputs
        sensor.AddObservation(target.eulerAngles * Mathf.Deg2Rad); //3 inputs
        sensor.AddObservation(droneBody.linearVelocity); //3 inputs
        sensor.AddObservation(transform.eulerAngles * Mathf.Deg2Rad); //3 inputs
        sensor.AddObservation(droneBody.angularVelocity); //3 inputs
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        idealLift = actions.ContinuousActions[0]; //ideaLift
        idealPitch = actions.ContinuousActions[1]; //idealPitch
        idealRoll = actions.ContinuousActions[2]; //idealRoll
        idealYawOmega = actions.ContinuousActions[3]; //ideaLaw

        //Debug.Log($"{idealLift} {idealPitch} {idealRoll} {idealYaw}");

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

        //yaw is a little different since it needs to be 360 degrees
        //simply multiplying by 180f is a good start, but it may be
        //difficult for the drone to fully turn around
        idealYawOmega = idealYawOmega * maxYawOmega;

        //Debug.Log($"{idealLift} {idealPitch} {idealRoll} {idealYaw}");
    }

    private void OnCollisionEnter(Collision collision)
    {
        AddReward(-2f * (episodeLength - time));
        EndEpisode();
    }

    public void FixedUpdate()
    {
        if(time >= 0.1f && droneBody.isKinematic)
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

        //calculate reward from distance
        float distance = Vector3.Distance(this.transform.position, target.transform.position);
        float sigmoid = 1f / (1f + Mathf.Exp(-distance / 5f)); //dividing by 5f makes the functions slope less steep(good for large distnace)
        float adjustedSig = (-4f * (sigmoid - 0.5f)) + 1f; // 1 value at zero distance and -1 at max distance
        //float timeAdjustedSig = adjustedSig / episodeLength;
        AddReward(adjustedSig * Time.fixedDeltaTime);

        //calculate reward from rotation
        float yawError = Mathf.Abs(Mathf.DeltaAngle(transform.eulerAngles.y, target.eulerAngles.y)); //value: 0-180
        float adjustedError = ((180 - yawError) - 90f) / 180f; //values: -0.5 to 0.5, mathches dron rotation to target
        float yawOmegaPenalty = -1f * Mathf.Abs(idealYawOmega / (maxYawOmega * 4f)); //values -0.25 to 0, penalizes too much yaw-ing
        AddReward((adjustedError + yawOmegaPenalty) * Time.fixedDeltaTime);
    }

    private float[] ThrustPID()
    {
        float[] thrusts = new float[] { 0f, 0f, 0f, 0f}; //FR, BR, FL, BL, torque
        Vector3 localAngularVelocity = transform.InverseTransformDirection(droneBody.angularVelocity);

        //first, calculate the proportion for pitch and roll
        //float pitchError = Mathf.DeltaAngle(droneBody.rotation.eulerAngles.x, idealPitch); //Mathf.DeltaAngle can also be used here if range expanded
        //Quaternion realPitchQuat = Quaternion.Euler(droneBody.rotation.eulerAngles.x, 0f, 0f);
        //Quaternion idealPitchQuat = Quaternion.Euler(idealPitch, 0f, 0f);
        //float pitchError = Quaternion.Angle(realPitchQuat, idealPitchQuat);
        float pitchError = AngularError(droneBody.rotation.eulerAngles.x, idealPitch);
        //float pitchDerr = ErrorDerrivative(droneBody.rotation.eulerAngles.x, lastPitch, Time.fixedDeltaTime) * Mathf.Rad2Deg;
        float pitchDerr = -localAngularVelocity.x * Mathf.Rad2Deg;
        float pitchProp = (pitchError - pitchDerr * damper) / 90f;

        //float rollError = Mathf.DeltaAngle(droneBody.rotation.eulerAngles.z, idealRoll + 180f); //Mathf.DeltaAngle can also be used here if range expanded
        float rollError = AngularError(droneBody.rotation.eulerAngles.z, idealRoll);
        //float rollDerr = ErrorDerrivative(droneBody.rotation.eulerAngles.z, lastRoll, Time.fixedDeltaTime) * Mathf.Rad2Deg;
        float rollDerr = -localAngularVelocity.z * Mathf.Rad2Deg;
        float rollProp = (rollError - rollDerr * damper) / 90f;

        //second, modulate yaw-ing on top of the simple pitch and roll values
        //float yawError = Mathf.DeltaAngle(droneBody.rotation.eulerAngles.y, idealYaw);
        //float yawError = AngularError(droneBody.rotation.eulerAngles.y, idealYaw);
        //float yawDerr = -droneBody.angularVelocity.y * Mathf.Rad2Deg;
        //float yawProp = (yawError - yawDerr * damper) / 90f;
        //float yawError = idealYaw; //relative
        //float yawDerr = ErrorDerrivative(droneBody.rotation.eulerAngles.y, lastYaw, Time.fixedDeltaTime) * Mathf.Rad2Deg;
        //float desiredYawRate = (idealYaw / 90f) * maxYawRate; // yawInput ranges from -1 to 1, scaled by maxYawRate
        //float yawError = localAngularVelocity.y  * (desiredYawRate - localAngularVelocity.y) * Mathf.Rad2Deg; // Error in yaw rate
        //float yawProp = idealYaw / 90f;
        //float yawDerr = -localAngularVelocity.y * Mathf.Rad2Deg;
        //float yawProp = (yawError - yawDerr * yawDamper) / 90f;
        float yawError = -localAngularVelocity.y- idealYawOmega; // Error in yaw rate 
        //float yawDerr = ErrorDerrivative(yawError, lastYawError, Time.fixedDeltaTime);
        float yawProp = -yawError / 4f; //-yawError / 90f;
        //droneBody.AddRelativeTorque(0f, (yawError) * Time.fixedDeltaTime, 0f, ForceMode.Impulse);
        //Debug.Log(localAngularVelocity.y * Mathf.Rad2Deg);

        //lastYawError = yawError;

        //set the thrust of all props equal for correct lift
        for (int i = 0; i < thrusts.Length; i++)
        {
            thrusts[i] = idealLift / 4f;
        }

        //make sure the total never excedes 1f and is proportional to error, not other forces
        float pitchAdj = pitchProp * axesThrustAlotment;
        float rollAdj = rollProp * axesThrustAlotment;
        float yawAdj = yawProp * axesThrustAlotment;

        thrusts[0] += pitchAdj + -rollAdj + -yawAdj; //FR //IDK why Yaw is neg here and think it should be opp on all others too
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

    private float ErrorDerrivative(float error, float lastError, float fixedDeltaTime)
    {
        return (error - lastError) / fixedDeltaTime;
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
        if (time >= episodeLength)
        {
            EndEpisode();
        }
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
        if(Input.GetKey(KeyCode.LeftShift)) //down
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