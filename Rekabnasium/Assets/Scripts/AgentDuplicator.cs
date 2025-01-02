using UnityEngine;

public class AgentDuplicator : MonoBehaviour
{
    [SerializeField] private GameObject agent;
    [SerializeField] private int agentCount = 10;
    [SerializeField] private Vector3 offset;
    // Start is called once before the first execution of Update after the MonoBehaviour is created
    void Start()
    {
        for(int i = 1; i < agentCount; i++)
        {
            GameObject newAgent = Instantiate(agent, agent.transform.position + offset * i, agent.transform.rotation);
            if(newAgent.TryGetComponent(out AgentDuplicator duplicator))
            {
                Destroy(duplicator);
            }
        }
    }
}
