#define USE_JOBS

using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Jobs;
using UnityEngine;
using UnityEngine.Assertions;
using Unity.Mathematics;
using Unity.Burst;
using static Unity.Mathematics.math;

public class Link
{
    public float m_RestDistance;
    public float m_Stiffness;
    public GravNode m_GravNode1, m_GravNode2;

    public Link(GravNode gn1, GravNode gn2, bool draw, float stiffness = 1.0f)
    {
        m_GravNode1 = gn1;
        m_GravNode2 = gn2;
        m_RestDistance = Vector3.Distance(gn1.m_Position,
            gn2.m_Position);
        gn1.m_Connections++;
        gn2.m_Connections++;

        gn1.neighborIndiciesList.Add(gn2.m_Index);
        gn2.neighborIndiciesList.Add(gn1.m_Index);

        gn1.restDistancesList.Add(m_RestDistance);
        gn2.restDistancesList.Add(m_RestDistance);

        gn1.stiffnessesList.Add(stiffness);
        gn2.stiffnessesList.Add(stiffness);

        if (draw)
        {
            LineRenderPair lnp = new GameObject("linerenderpair").AddComponent<LineRenderPair>();
            lnp.SetupLineRenderPair(m_GravNode1.lineRendererTransform, m_GravNode2.lineRendererTransform, 0.05f);
        }

        m_Stiffness = stiffness;
    }

    public void OffsetNodeConnection(Vector3 correctionVectorHalf)
    {
        m_GravNode1.OffsetPos(correctionVectorHalf);
        m_GravNode2.OffsetPos(-correctionVectorHalf);
    }
}

[BurstCompile]
public struct SolveConstraintsJob : IJobParallelFor
{
    [ReadOnly]
    public NativeArray<float> restDistances;
    [ReadOnly]
    public NativeArray<float> linkStiffness;
    [ReadOnly]
    public NativeArray<float3> positions;
    [ReadOnly]
    public NativeArray<int> gn1Indicies;
    [ReadOnly]
    public NativeArray<int> gn2Indicies;
    [WriteOnly]
    //half of the correction vector for the connection
    public NativeArray<float3> results;

    public void Execute(int index)
    {
        //float3 gn2Pos = gn2Postions[index];
        int index1 = gn1Indicies[index];
        int index2 = gn2Indicies[index];
        float3 gn1Pos = positions[index1];
        float3 gn2Pos = positions[index2];
        float restDistance = restDistances[index];

        float3 gn1Togn2 = gn2Pos - gn1Pos;
        float current_distance = math.distance(gn2Pos, gn1Pos);
        // TODO:: branching in the hot path here... perhaps separate the distance validation and correction vector calculation steps.
        if (current_distance == 0)
            return;
        float scalar = linkStiffness[index];
        
        float3 correctionVector = (gn1Togn2 * (1.0f - restDistance / current_distance));

        results[index] = correctionVector * 0.5f * scalar;
    }
}

[BurstCompile]
public struct UpdateParticlePositions : IJobParallelFor
{
    [ReadOnly]
    public NativeArray<float3> results;
    [ReadOnly]
    public NativeArray<int> position1Index;
    [ReadOnly]
    public NativeArray<int> position2Index;
    [ReadOnly]
    public NativeArray<int> connections;
    [ReadOnly]
    public NativeArray<bool> moveables;
    [ReadOnly]
    public NativeArray<float3> currentPositions;
    [WriteOnly]
    public NativeArray<float3> positions;

    public void Execute(int index)
    {
        int gn1I = position1Index[index];
        int gn2I = position2Index[index];
        if (moveables[gn1I])
        {
            float3 pos1 = currentPositions[gn1I];
            int con1 = connections[gn1I];
            pos1 += results[index] / con1;
            positions[gn1I] = pos1;
        }

        if (moveables[gn2I])
        {
            float3 pos2 = currentPositions[gn2I];
            int con2 = connections[gn2I];
            pos2 -= results[index] / con2;
            positions[gn2I] = pos2;
        }
    }
}
