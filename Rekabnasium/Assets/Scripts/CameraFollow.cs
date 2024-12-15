using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraFollow : MonoBehaviour
{
    [SerializeField] private Transform target;
    [SerializeField] private Transform drone;
    [SerializeField] private Vector3 cameraOffset;

    // Update is called once per frame
    void Update()
    {
        //position
        Vector3 droneToTarget = (target.position - drone.position).normalized;
        transform.forward = droneToTarget;
        transform.position = drone.position;
        transform.position += droneToTarget * cameraOffset.z;
        transform.position += Vector3.up * cameraOffset.y;

        //rotation
        Vector3 cameraToTarget = (target.position - transform.position).normalized;
        transform.forward = cameraToTarget;
    }
}
