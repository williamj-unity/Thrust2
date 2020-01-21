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
    public int m_GravNode1PosIndex, m_GravNode2PosIndex;
    public int m_VertexStartIndex;
    public int m_GravNode1VertexStart, m_GravNode2VertexStart;
    public bool m_Draw;

    public Link(GravNode gn1, GravNode gn2, bool draw, float stiffness, GravMesh gravMesh)
    {
        m_GravNode1PosIndex = gn1.m_Index;
        m_GravNode2PosIndex = gn2.m_Index;
        m_RestDistance = Vector3.Distance(gn1.m_Position,
            gn2.m_Position);

        gn1.neighborIndiciesList.Add(gn2.m_Index);
        gn2.neighborIndiciesList.Add(gn1.m_Index);

        gn1.restDistancesList.Add(m_RestDistance);
        gn2.restDistancesList.Add(m_RestDistance);

        gn1.stiffnessesList.Add(stiffness);
        gn2.stiffnessesList.Add(stiffness);
        m_Draw = draw;

        if (draw)
        {
            m_GravNode1VertexStart = (gn1.m_Connections * 2) + gn1.m_VertexStart;
            m_GravNode2VertexStart = (gn2.m_Connections * 2) + gn2.m_VertexStart;
            gravMesh.AddPair(m_GravNode1VertexStart, m_GravNode2VertexStart);
        }

        gn1.m_Connections++;
        gn2.m_Connections++;

        m_Stiffness = stiffness;
    }

    [BurstCompile]
    public struct UpdateMeshVertices : IJobParallelFor
    {
        [ReadOnly]
        public NativeArray<float3> positions;
        [ReadOnly]
        public NativeArray<int> gn1PositionIndex;
        [ReadOnly]
        public NativeArray<int> gn2PositionIndex;
        [ReadOnly]
        public NativeArray<int> gn1VertexStartIndices;
        [ReadOnly]
        public NativeArray<int> gn2VertexStartIndices;
        [ReadOnly]
        public NativeArray<bool> draw;
        [ReadOnly]
        public float lineWidth;
        [ReadOnly]
        public float3 cameraPos;
        [ReadOnly]
        public float3 cameraUp;
        [ReadOnly]
        public float3 gravGridPos;
        [WriteOnly]
        [NativeDisableParallelForRestriction]
        public NativeArray<float3> vertixPositions;

        public void Execute(int index)
        {
            if (!draw[index])
                return;

            float3 position1 = positions[gn1PositionIndex[index]];
            float3 position2 = positions[gn2PositionIndex[index]];

            float3 pointWorld = (position2 + position1) / 2.0f + gravGridPos;
            float3 look = math.normalize(pointWorld - cameraPos);

            float3 perp = math.cross(math.normalize(position2 - position1) * lineWidth, look);
            int gn1VertexStart = gn1VertexStartIndices[index];
            int gn2VertexStart = gn2VertexStartIndices[index];

            vertixPositions[gn1VertexStart + 1] = position1 - perp;
            vertixPositions[gn1VertexStart] = position1 + perp;
            vertixPositions[gn2VertexStart + 1] = position2 - perp;
            vertixPositions[gn2VertexStart] = position2 + perp;
        }
    }

}

