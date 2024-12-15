using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class Drone_Simception_1 : Agent
{
    [SerializeField] private Transform target;
    [SerializeField] private float episodeLength = 15f;
    private Rigidbody droneBody;
    private float thrust;
    private float time = 0f;

    public void Start()
    {
        droneBody = this.GetComponent<Rigidbody>();
    }

    public override void OnEpisodeBegin()
    {
        time = 0f; //reset clock
        this.transform.localPosition = Vector3.zero; //reset positon

        //make new target position
        target.localPosition = new Vector3(0f, Random.Range(3f, 9f), 0f);
    }

    public override void CollectObservations(VectorSensor sensor)
    {
        sensor.AddObservation(transform.position.y); //1 input
        sensor.AddObservation(target.position.y); //1 input
        sensor.AddObservation(droneBody.velocity.y); //1 input
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        thrust = actions.ContinuousActions[0];

        if(thrust <= 0)
        {
            thrust = Mathf.Lerp(0, 9.81f, thrust + 1); // gravity compensation for negative thrust
        }
        else
        {
            thrust = Mathf.Lerp(9.81f, 15f, thrust); // regular thrust for positive values
        }
    }

    public void FixedUpdate()
    {
        //Debug.Log("thrust " + thrust);
        //Debug.Log("reward " + GetCumulativeReward());

        //input do drone
        droneBody.AddForce(Vector3.up * thrust * Time.fixedDeltaTime, ForceMode.Impulse);

        //calculate reward from distance
        float distance = Mathf.Abs(this.transform.position.y - target.transform.position.y);
        float sigmoid = 1f / (1f + Mathf.Exp(-distance));
        float adjustedSig = (-4f * (sigmoid - 0.5f)) + 1f; // 1 value at zero distance and -1 at max distance
        //float timeAdjustedSig = adjustedSig / episodeLength;
        AddReward(adjustedSig * Time.fixedDeltaTime);
    }

    public void Update()
    {
        //little clock
        time += Time.deltaTime;
        if(time >= episodeLength)
        {
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continousActions = actionsOut.ContinuousActions;
        if(Input.GetKey(KeyCode.W))
        {
            continousActions[0] = 1f;
        }
        else
        {
            continousActions[0] = -1f;
        }
    }
}
