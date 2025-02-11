using System;
using UnityEngine;
using UnityEngine.Serialization;

namespace Unity.MLAgents
{
    /// <summary>
    /// The DecisionRequester component automatically request decisions for an
    /// <see cref="Agent"/> instance at regular intervals.
    /// </summary>
    /// <remarks>
    /// Attach a DecisionRequester component to the same [GameObject] as the
    /// <see cref="Agent"/> component.
    ///
    /// The DecisionRequester component provides a convenient and flexible way to
    /// trigger the agent decision making process. Without a DecisionRequester,
    /// your <see cref="Agent"/> implementation must manually call its
    /// <seealso cref="Agent.RequestDecision"/> function.
    /// </remarks>
    /// 
    [RequireComponent(typeof(Agent))]
    public class DecisionRequesterManual : MonoBehaviour
    {
        [NonSerialized]
        Agent m_Agent;

        private bool decisionRequested = false;

        /// <summary>
        /// Get the Agent attached to the DecisionRequester.
        /// </summary>
        public Agent Agent
        {
            get => m_Agent;
        }

        internal void Awake()
        {
            m_Agent = gameObject.GetComponent<Agent>();
            Debug.Assert(m_Agent != null, "Agent component was not found on this gameObject and is required.");
            Academy.Instance.AgentPreStep += MakeRequests;
        }

        void OnDestroy()
        {
            if (Academy.IsInitialized)
            {
                Academy.Instance.AgentPreStep -= MakeRequests;
            }
        }

        /// <summary>
        /// Information about Academy step used to make decisions about whether to request a decision.
        /// </summary>
        public struct DecisionRequestContext
        {
            /// <summary>
            /// The current step count of the Academy, equivalent to Academy.StepCount.
            /// </summary>
            public int AcademyStepCount;
        }

        public void RequestDecision()
        {
            decisionRequested = true;
        }

        /// <summary>
        /// Method that hooks into the Academy in order inform the Agent on whether or not it should request a
        /// decision, and whether or not it should take actions between decisions.
        /// </summary>
        /// <param name="academyStepCount">The current step count of the academy.</param>
        void MakeRequests(int academyStepCount)
        {
            var context = new DecisionRequestContext
            {
                AcademyStepCount = academyStepCount
            };

            if (ShouldRequestDecision(context))
            {
                m_Agent?.RequestDecision();
            }

            if (ShouldRequestAction(context))
            {
                m_Agent?.RequestAction();
            }
        }

        /// <summary>
        /// Whether Agent.RequestDecision should be called on this update step.
        /// </summary>
        /// <param name="context"></param>
        /// <returns></returns>
        protected virtual bool ShouldRequestDecision(DecisionRequestContext context)
        {
            bool returnState = decisionRequested;
            decisionRequested = false;
            return returnState;
        }

        /// <summary>
        /// Whether Agent.RequestAction should be called on this update step.
        /// </summary>
        /// <param name="context"></param>
        /// <returns></returns>
        protected virtual bool ShouldRequestAction(DecisionRequestContext context)
        {
            return false;
        }
    }
}
