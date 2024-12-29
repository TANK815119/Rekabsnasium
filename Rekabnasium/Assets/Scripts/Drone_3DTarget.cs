using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class Drone_3DTarget : Agent
{
    [SerializeField] private Transform target;
    [SerializeField] private float episodeLength = 15f;

    [SerializeField] private Transform FRProp;
    [SerializeField] private Transform BRProp;
    [SerializeField] private Transform FLProp;
    [SerializeField] private Transform BLProp;
    private Transform[] propTrans;

    private float FRThrust;
    private float BRThrust;
    private float FLThrust;
    private float BLThrust;
    private float[] thrusts;
    private float torque;

    private Rigidbody droneBody;
    private float time = 0f;

    public void Start()
    {
        droneBody = this.GetComponent<Rigidbody>();
        propTrans = new Transform[] { FRProp, BRProp, FLProp, BLProp };
        thrusts = new float[] { FRThrust, BRThrust, FLThrust, BLThrust };
    }

    public override void OnEpisodeBegin()
    {
        time = 0f; //reset clock
        this.transform.localPosition = Vector3.zero; //reset positon
        droneBody.linearVelocity = Vector3.zero; //reset linear velocity
        this.transform.localRotation = Quaternion.identity; //reset rotation
        droneBody.angularVelocity = Vector3.zero; //reset angular velocity

        //make new target position
        target.localPosition = new Vector3(0f, Random.Range(3f, 9f), 0f);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.position); //3 inputs
        sensor.AddObservation(target.position); //3 inputs
        sensor.AddObservation(droneBody.linearVelocity); //3 inputs
        sensor.AddObservation(transform.eulerAngles * Mathf.Deg2Rad); //3 inputs
        sensor.AddObservation(droneBody.angularVelocity); //3 inputs
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        thrusts[0] = actions.ContinuousActions[0];
        thrusts[1] = actions.ContinuousActions[1];
        thrusts[2] = actions.ContinuousActions[2];
        thrusts[3] = actions.ContinuousActions[3];

        //2.4525 = 9.81/4
        //3.71 = 15/4
        for (int i = 0; i < thrusts.Length; i++)
        {
            if (thrusts[i] <= 0)
            {
                thrusts[i] = Mathf.Lerp(0, 2.45f, thrusts[i] + 1); // gravity compensation for negative thrust
            }
            else
            {
                thrusts[i] = Mathf.Lerp(2.45f, 3.71f, thrusts[i]); // regular thrust for positive values
            }
        }

        float propDist = Mathf.Sqrt(Mathf.Pow(propTrans[0].position.x, 2f) + Mathf.Pow(propTrans[0].position.z, 2f));
        torque = propDist * ((thrusts[0] + thrusts[3]) - (thrusts[2] + thrusts[1])); //(FR+BL) - (FL+BR) for left hand rule(not rihgt)
    }

    private void OnCollisionEnter(Collision collision)
    {
        AddReward(-1.1f * (episodeLength - time));
        EndEpisode();
    }

    public void FixedUpdate()
    {
        //apply outputs
        for(int i =0; i < propTrans.Length; i++)
        {
            //droneBody.AddForce(Vector3.up * thrust * Time.fixedDeltaTime, ForceMode.Impulse);
            Vector3 localThrust = Vector3.up * thrusts[i];
            Vector3 globalThrust = propTrans[i].TransformDirection(localThrust);
            droneBody.AddForceAtPosition(globalThrust * Time.fixedDeltaTime, propTrans[i].position, ForceMode.Impulse);
        }

        //yaw torque
        droneBody.AddRelativeTorque(Vector3.up * torque * Time.fixedDeltaTime, ForceMode.Impulse);

        //calculate reward from distance
        float distance = Vector3.Distance(this.transform.position, target.transform.position);
        float sigmoid = 1f / (1f + Mathf.Exp(-distance));
        float adjustedSig = (-4f * (sigmoid - 0.5f)) + 1f; // 1 value at zero distance and -1 at max distance
        //float timeAdjustedSig = adjustedSig / episodeLength;
        AddReward(adjustedSig * Time.fixedDeltaTime);
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
        continousActions[0] = -1f;
        continousActions[1] = -1f;
        continousActions[2] = -1f;
        continousActions[3] = -1f;
        if (Input.GetKey(KeyCode.S)) //front
        {
            continousActions[0] += 0.05f;
            continousActions[2] += 0.05f;
        }
        if (Input.GetKey(KeyCode.W)) //back
        {
            continousActions[1] += 0.05f;
            continousActions[3] += 0.05f;
        }
        if (Input.GetKey(KeyCode.D)) //left
        {
            continousActions[2] += 0.05f;
            continousActions[3] += 0.05f;
        }
        if (Input.GetKey(KeyCode.A)) //right
        {
            continousActions[0] += 0.05f;
            continousActions[1] += 0.05f;
        }
        if (Input.GetKey(KeyCode.Q)) //turn left
        {
            continousActions[2] += 0.05f; //FL
            continousActions[1] += 0.05f; //BR
        }
        if (Input.GetKey(KeyCode.E)) //turn right
        {
            continousActions[0] += 0.05f; //FR
            continousActions[3] += 0.05f; //BR
        }
        if (Input.GetKey(KeyCode.Space)) //up
        {
            continousActions[0] += 1.9f;
            continousActions[1] += 1.9f;
            continousActions[2] += 1.9f;
            continousActions[3] += 1.9f;
        }
    }
}
