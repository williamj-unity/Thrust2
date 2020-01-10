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
using Unity.Collections;

public class GravNode
{
    public Transform lineRendererTransform { get; private set; }

    public bool m_Moveable;

    public float3 m_Position;
    public float3 m_PrevPosition;
    public float3 m_TargetPosition;
    public float3 m_StartPosition;
    public float3 m_Acceleration;
    public GravNodeCollider gravNodeColliderParent { get; private set; }

    public int m_Connections;
    public int m_Index;

    protected float rigidity = 1.0f;
    public GravNode(float3 position, float spacing, int index)
    {
        lineRendererTransform = new GameObject("LineRendererTransform").transform;
        lineRendererTransform.position = position;
        m_Position = position;
        m_PrevPosition = position;
        m_TargetPosition = m_Position;
        m_StartPosition = m_Position;
        m_Acceleration = float3(0,0,0);
        m_Index = index;
        m_Moveable = true;
        gravNodeColliderParent = new GameObject("GravNodeAnchor").AddComponent<GravNodeCollider>();
        gravNodeColliderParent.SetSpacing(spacing);
        gravNodeColliderParent.transform.position = position;
        gravNodeColliderParent.affectorCollisionEnter += AffectorCollisionEnter;
        gravNodeColliderParent.affectorCollisionExit += AffectorCollisionExit;
    }

    public void OffsetPos(float3 correctionVectorHalf)
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

    void SetTargetPosition(float3 position)
    {
        m_TargetPosition = position;
    }

    public void SetPosition(float3 position)
    {
        if(m_Moveable)
        {
            m_Position = position;
        }
    }

    public void ApplyForce(float3 force)
    {
        if(m_Moveable)
            m_Acceleration += force;
    }

    void ResetAcceleration()
    {
        m_Acceleration = float3(0, 0, 0);
    }

    public void UpdateTransforms(float3 prevPosition, float3 position)
    {
        m_Position = position;
        m_PrevPosition = prevPosition;
        lineRendererTransform.position = m_Position;
        ResetAcceleration();
    }

    [BurstCompile]
    public struct UpdateJob : IJobParallelFor
    {
        [ReadOnly]
        public float damping;
        [ReadOnly]
        public float timeStep;
        [ReadOnly]
        public NativeArray<float3> targetPositions;
        [ReadOnly]
        public NativeArray<float3> accelerations;
        [ReadOnly]
        public NativeArray<bool> moveable;

        public NativeArray<float3> positions;
        public NativeArray<float3> prevPositions;

        public void Execute(int index)
        {
            float3 acceleration = 0;
            if (moveable[index])
                acceleration = accelerations[index] + (targetPositions[index] - positions[index]);

            float timeStepSq = timeStep * timeStep;

            float3 velocity = (positions[index] - prevPositions[index]) * 0.99f;
            float3 next = positions[index] + (velocity) + damping * acceleration * timeStepSq;
            prevPositions[index] = positions[index];
            positions[index] = next;
        }
    }
}


