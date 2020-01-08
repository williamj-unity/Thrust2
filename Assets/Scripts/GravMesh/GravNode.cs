using System.Collections;
using System.Collections.Generic;
using System.Numerics;
using UnityEditor.MemoryProfiler;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Burst;
using static Unity.Mathematics.math;
using UnityEngine;
using Vector3 = UnityEngine.Vector3;

public class GravNode
{
    public Transform lineRendererTransform { get; private set; }

    public bool m_Moveable;

    public Vector3 m_Position;
    public Vector3 m_PrevPosition;
    public Vector3 m_Velocity;
    public Vector3 m_TargetPosition;
    public Vector3 m_StartPosition;
    private Vector3 m_Acceleration;
    public GravNodeCollider gravNodeColliderParent { get; private set; }


    public int m_Connections;

    protected float rigidity = 1.0f;
    public GravNode(Vector3 position, float spacing, bool pinned = false)
    {
        lineRendererTransform = new GameObject("LineRendererTransform").transform;
        lineRendererTransform.position = position;

        m_Position = position;
        m_PrevPosition = position;
        m_TargetPosition = m_Position;
        m_StartPosition = m_Position;
        m_Moveable = !pinned;
        m_Acceleration = Vector3.zero;

        gravNodeColliderParent = new GameObject("GravNodeAnchor").AddComponent<GravNodeCollider>();
        gravNodeColliderParent.SetSpacing(spacing);
        gravNodeColliderParent.transform.position = position;
        gravNodeColliderParent.affectorCollisionEnter += AffectorCollisionEnter;
        gravNodeColliderParent.affectorCollisionExit += AffectorCollisionExit;
    }

    public void OffsetPos(Vector3 correctionVectorHalf)
    {
        if (m_Moveable && m_Connections > 0)
        {
            m_Position += (correctionVectorHalf / m_Connections);
        }
    }
    void AffectorCollisionEnter(float mass)
    {
        Vector3 newPose = m_Position;
        newPose.z = mass;
        SetTargetPosition(newPose);
    }

    void AffectorCollisionExit()
    {
        SetTargetPosition(m_StartPosition);
    }

    void SetTargetPosition(Vector3 position)
    {
        m_TargetPosition = position;
    }

    public void ApplyForce(Vector3 force)
    {
        if(m_Moveable)
            m_Acceleration = force;
    }

    void ResetAcceleration()
    {
        m_Acceleration = Vector3.zero;
    }

    public virtual void Update(float damping, float timeStep)
    {
        ApplyForce((m_TargetPosition - m_Position));

        float timeStepSq = timeStep * timeStep;

        m_Velocity = (m_Position - m_PrevPosition) * 0.99f;
        Vector3 next = m_Position + (m_Velocity) + 0.5f * m_Acceleration * timeStep;
        m_PrevPosition = m_Position;
        m_Position = next;
        lineRendererTransform.position = m_Position;
        ResetAcceleration();
    }

    public struct UpdateJob : IJobParallelFor
    {
        public void Execute(int index)
        {
            throw new System.NotImplementedException();
        }
    }
}


