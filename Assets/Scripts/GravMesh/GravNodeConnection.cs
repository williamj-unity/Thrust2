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
    public bool m_Draw;

    public Link(GravNode gn1, GravNode gn2, bool draw, float stiffness, GravMesh gravMesh)
    {
        m_GravNode1PosIndex = gn1.m_Index;
        m_GravNode2PosIndex = gn2.m_Index;
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
        m_Draw = draw;

        if (draw)
        {
            m_VertexStartIndex = gravMesh.AddPair(gn1.m_Position, gn2.m_Position);
        }

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
        public NativeArray<int> vertexStartIndex;
        [ReadOnly]
        public NativeArray<bool> draw;
        [ReadOnly]
        public float lineWidth;
        [WriteOnly]
        [NativeDisableParallelForRestriction]
        public NativeArray<float3> vertixPositions;

        public void Execute(int index)
        {
            if (!draw[index])
                return;

            float3 position1 = positions[gn1PositionIndex[index]];
            float3 position2 = positions[gn2PositionIndex[index]];
            float3 perp = math.cross(position2 - position1, float3(0,0,-1) * lineWidth);

            vertixPositions[vertexStartIndex[index]] = position1 - perp;
            vertixPositions[vertexStartIndex[index] + 1] = position1 + perp;
            vertixPositions[vertexStartIndex[index] + 2] = position2 - perp;
            vertixPositions[vertexStartIndex[index] + 3] = position2 + perp;
        }
    }

}

