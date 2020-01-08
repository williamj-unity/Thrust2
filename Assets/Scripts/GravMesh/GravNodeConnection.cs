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

        if(draw)
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
    public NativeArray<float3> gn1Postions;
    [ReadOnly]
    public NativeArray<float3> gn2Postions;

    [WriteOnly]
    //half of the correction vector for the connection
    public NativeArray<float3> results;

    public void Execute(int index)
    {
        float3 gn2Pos = gn2Postions[index];
        float3 gn1Pos = gn1Postions[index];
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

