﻿using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.MLAgents;
using Unity.MLAgents.Actuators;
using Unity.MLAgents.Sensors;

public class Drone_Overlord_FPV : Agent
{
    [SerializeField] private Transform POI; //point of interest
    [SerializeField] private Transform target;
    [SerializeField] private Transform drone;
    [SerializeField] private Camera FPVCamera;
   
    [SerializeField] private float episodeLength = 15f;
    [SerializeField] private float maxDistance = 25f;
    [SerializeField] private float randRange = 25f;
    [SerializeField] private float decisionRange = 0.5f; //how close to target till range request
    [SerializeField] private float decisionTImeMin = 0.5f; //how much time till another decisiona allowed
    [SerializeField] private float decisionTImeMax = 3f; //how much time till another decisiona made

    private Drone_3DTarget_Functional droneTargeting;
    private DecisionRequesterManual decisionRequester;
    private Rigidbody droneBody;
    private float time = 0f;
    private float decisionTime = 0f;
    //private Vector3 targetPosition;
    private Quaternion targetYaw;
    private Vector3 lastOffset;
    private Vector3 offset;

    public void Awake()
    {
        droneTargeting = drone.GetComponent<Drone_3DTarget_Functional>();
        decisionRequester = GetComponent<DecisionRequesterManual>();
        droneBody = drone.GetComponent<Rigidbody>();
        lastOffset = Vector3.zero;
    }

    public override void OnEpisodeBegin()
    {
        time = 0f; //reset clock
        lastOffset = Vector3.zero;
        droneTargeting.ResetDrone(); //reset drone

        //make new POI
        POI.localPosition = new Vector3(Random.Range(-randRange, randRange), Random.Range(0f, randRange), Random.Range(-randRange, randRange));
    }

    public override void CollectObservations(VectorSensor sensor) //TOTAL: 11
    {
        // velocity akin to accelerometer
        Vector3 velocity = droneBody.linearVelocity;
        sensor.AddObservation(velocity); // 3 values

        // global drone yaw yaw (sin/cos for direction) akin to compass
        float globalYaw = drone.eulerAngles.y;
        sensor.AddObservation(Mathf.Sin(globalYaw)); // 1 value
        sensor.AddObservation(Mathf.Cos(globalYaw)); // 1 value

        // last local(to drone) target offset
        sensor.AddObservation(lastOffset); //3 values

        // POI screenspace position and oncreen tell
        Vector2 screenPos = FPVCamera.WorldToViewportPoint(POI.position);
        bool onScreen = true;
        if (screenPos.x < 0f || screenPos.y < 0f|| screenPos.x > 1f|| screenPos.y > 1f)
        {
            onScreen = false;
            screenPos = Vector3.zero;
        }
        sensor.AddObservation(onScreen); //1 value
        sensor.AddObservation(screenPos); //2 values
        

        // render texture inputs are handled automatically i think
    }

    public override void OnActionReceived(ActionBuffers actions)
    {
        float idealX = actions.ContinuousActions[0]; //ideaL relative X position
        float idealY = actions.ContinuousActions[1]; //ideaL relative y position
        float idealZ = actions.ContinuousActions[2]; //ideaL relative z position
        float idealYaw = actions.ContinuousActions[3]; //ideaL relative yaw position

        //create a relative offset in local space, scaling to max distance
        offset = new Vector3(idealX, idealY, idealZ) * maxDistance;
        offset =  new Vector3(Mathf.Clamp(offset.x, -maxDistance, maxDistance), Mathf.Clamp(offset.y, -maxDistance, maxDistance), Mathf.Clamp(offset.z, -maxDistance, maxDistance));

        // Convert this local offset to world space relative to the drone's position
        //target.position = drone.TransformPoint(localOffset);
        //targetPosition = drone.position + offset;

        //add the idealYaw to the current yaw
        float currentYaw = target.eulerAngles.y; // Get the current yaw in world space
        float yaw = currentYaw + idealYaw * 15f; // Add the yaw offset
        targetYaw = Quaternion.Euler(0f, yaw, 0f); // Yaw is rotation around Y-axis

        //stow for later input
        lastOffset = offset;
        //distIntegral = Vector3.zero; //reser integral
    }

    private void OnCollisionEnter(Collision collision)
    {
        AddReward(-2f * (episodeLength - time));
        EndEpisode();
    }

    public void FixedUpdate()
    {
        ClockTick();

        decisionTime += Time.fixedDeltaTime;
        if (decisionTime > decisionTImeMax || ((target.position - drone.position).magnitude < decisionRange && decisionTime > decisionTImeMin))
        {
            decisionRequester.RequestDecision();
            decisionTime = 0f;
        }

        //integrate velocity into offset to fix position
        offset -= droneBody.linearVelocity * Time.fixedDeltaTime;

        //set target position and rotation
        target.position = drone.position + offset;
        target.rotation = targetYaw;

        //calculate reward
        CalculateReward();

        //Check if real close to give big doe of reward and reset
        CheckPOIHit();
    }

    private void CalculateReward()
    {
        //calculate reward from distance
        float distance = Vector3.Distance(target.position, POI.position);
        float sigmoid = 1f / (1f + Mathf.Exp(-distance / 5f)); //dividing by 5f makes the functions slope less steep(good for large distnace)
        float distanceReward = (-4f * (sigmoid - 0.5f)) + 1f; // 1 value at zero distance and -1 at max distance

        //weight rewards properly
        float combineRewards = distanceReward * 1f;
        AddReward(combineRewards * Time.fixedDeltaTime);
    }

    private void CheckPOIHit()
    {
        if(Vector3.Distance(drone.position, POI.position) < 0.5f)
        {
            float timeRemaining = episodeLength - time;
            AddReward(2f * timeRemaining);
            EndEpisode();
        }
    }

    public void ClockTick()
    {
        //little clock
        time += Time.fixedDeltaTime; //fixed delta is better because thats what mlagents runs on
        if (time >= episodeLength)
        {
            EndEpisode();
        }
    }

    public override void Heuristic(in ActionBuffers actionsOut)
    {
        ActionSegment<float> continousActions = actionsOut.ContinuousActions;
        continousActions[0] = 0f; //x
        continousActions[1] = 0f; //y
        continousActions[2] = 0f; //z
        continousActions[3] = 0f; //yaw

        if (Input.GetKey(KeyCode.W)) //front
        {
            continousActions[2] += 0.2f;
        }
        if (Input.GetKey(KeyCode.S)) //back
        {
            continousActions[2] += -0.2f;
        }
        if (Input.GetKey(KeyCode.A)) //left
        {
            continousActions[0] += -0.2f;
        }
        if (Input.GetKey(KeyCode.D)) //right
        {
            continousActions[0] += +0.2f;
        }
        if (Input.GetKey(KeyCode.E)) //turn right
        {
            continousActions[3] += -0.2f;
        }
        if (Input.GetKey(KeyCode.Q)) //turn left
        {
            continousActions[3] += 0.2f;
        }
        if (Input.GetKey(KeyCode.Space)) //up
        {
            continousActions[1] += 0.2f;
        }
        if (Input.GetKey(KeyCode.LeftShift)) //down
        {
            continousActions[1] += -0.2f;
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