using UnityEngine;

public class TargetLooker : MonoBehaviour
{
    [SerializeField] private Transform drone;

    // Update is called once per frame
    void Update()
    {
        Vector3 forward = transform.position - drone.position;
        this.transform.forward = forward.normalized;
        this.transform.rotation = Quaternion.Euler(0f, this.transform.eulerAngles.y, 0f);
    }
}
