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
using Unity.Collections.LowLevel.Unsafe;
using System;

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
    public NativeArray<int> neighborIndicies;
    public NativeArray<float> restDistances;
    public NativeArray<float> stiffnesses;

    public List<int> neighborIndiciesList;
    public List<float> restDistancesList;
    public List<float> stiffnessesList;

    public Action<float3, int> SetTargetLocation;
    public Action<float3, int> AddForce;

    public GravNode(float3 position, float spacing, int index, Transform anchorParent)
    {
        m_Position = position;
        m_PrevPosition = position;
        m_TargetPosition = m_Position;
        m_StartPosition = m_Position;
        m_Acceleration = float3(0,0,0);
        m_Index = index;
        m_Moveable = true;
        gravNodeColliderParent = new GameObject("GravNodeAnchor").AddComponent<GravNodeCollider>();
        gravNodeColliderParent.SetSpacing(spacing/2);
        gravNodeColliderParent.affectorCollisionEnter += AffectorCollisionEnter;
        gravNodeColliderParent.affectorCollisionExit += AffectorCollisionExit;
        gravNodeColliderParent.transform.parent = anchorParent;
        gravNodeColliderParent.transform.localPosition = position;
        neighborIndiciesList = new List<int>();
        stiffnessesList = new List<float>();
        restDistancesList = new List<float>();
    }

    public void ConvertNeighborListToNativeArray()
    {
        neighborIndicies = new NativeArray<int>(neighborIndiciesList.ToArray(), Allocator.Persistent);
        restDistances = new NativeArray<float>(restDistancesList.ToArray(), Allocator.Persistent);
        stiffnesses = new NativeArray<float>(stiffnessesList.ToArray(), Allocator.Persistent);
    }

    ~GravNode()
    {
        neighborIndicies.Dispose();
        restDistances.Dispose();
        stiffnesses.Dispose();
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
        Vector3 newPose = m_StartPosition;
        newPose.z = mass;
        SetTargetPosition(newPose);
    }

    void AffectorCollisionExit()
    {
        SetTargetPosition(m_StartPosition);
    }

    void SetTargetPosition(float3 position)
    {
        SetTargetLocation(position, m_Index);
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
        if (m_Moveable)
            AddForce(force, m_Index);
    }

    public void ResetAcceleration()
    {
        m_Acceleration = float3(0, 0, 0);
    }

    public void UpdateTransforms(float3 prevPosition, float3 position)
    {
        m_Position = position;
        m_PrevPosition = prevPosition;
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

    [BurstCompile]
    public struct SolveContraints : IJobParallelFor
    {
        [ReadOnly]
        public NativeArray<int> allneighbors;
        [ReadOnly]
        public NativeArray<int> numConnections;
        [ReadOnly]
        public NativeArray<int> startIndex;
        [ReadOnly]
        public NativeArray<float> allRestDistances;
        [ReadOnly]
        public float linkStiffness;
        [ReadOnly]
        public NativeArray<bool> moveable;
        [ReadOnly]
        public NativeArray<float3> positions;
        [WriteOnly]
        public NativeArray<float3> outPositions;

        public void Execute(int index)
        {
            float3 gn1Pos = positions[index];
            outPositions[index] = gn1Pos;
            if (!moveable[index])
                return;

            float3 gn2Pos = 0;
            float3 accumulator = gn1Pos;

            int numConnection = numConnections[index];
            for (int i = 0; i < numConnection; i++)
            {
                int start = startIndex[index];
                
                gn2Pos = positions[allneighbors[start + i]];
                float restDistance = allRestDistances[start + i];

                float3 gn1Togn2 = gn2Pos - gn1Pos;
                float current_distance = math.distance(gn2Pos, gn1Pos);
                float scalar = linkStiffness;

                float3 correctionVector = (gn1Togn2 * (1.0f - restDistance / current_distance));

                accumulator += (correctionVector * 0.5f * scalar) / numConnection;
            }
            outPositions[index] = accumulator;
        }
    }

    [BurstCompile]
    public struct SwapBuffers : IJobParallelFor
    {
        [WriteOnly]
        public NativeArray<float3> positions;
        [ReadOnly]
        public NativeArray<float3> newPositions;

        public void Execute(int index)
        {
            positions[index] = newPositions[index];
        }
    }

    [BurstCompile]
    public struct ResetAcclerations : IJobParallelFor
    {
        [WriteOnly]
        public NativeArray<float3> accelerations;

        public void Execute(int index)
        {
            accelerations[index] = float3(0,0,0);
        }
    }
}


