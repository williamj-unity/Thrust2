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
using System;

public class Link
{
    public float m_RestDistance;
    public int m_GravNode1PosIndex, m_GravNode2PosIndex;
    public int m_VertexStartIndex;
    public int m_GravNode1VertexStart, m_GravNode2VertexStart;
    public bool m_Draw;
    public GravNode m_Gn1;
    public GravNode m_Gn2;
    public int m_TrianglesIndex;
    public int m_Index;
    public float m_Thickness;

    public Link(GravNode gn1, GravNode gn2, bool draw, GravMesh gravMesh, float thickness, int index)
    {
        m_Index = index;
        m_Gn1 = gn1;
        m_Gn2 = gn2;
        m_GravNode1PosIndex = gn1.m_Index;
        m_GravNode2PosIndex = gn2.m_Index;
        m_RestDistance = Vector3.Distance(gn1.m_StartPosition,
            gn2.m_StartPosition);

        gn1.m_Links.Add(this);

        gn1.neighborIndiciesList.Add(gn2.m_Index);
        gn2.neighborIndiciesList.Add(gn1.m_Index);

        gn1.restDistancesList.Add(m_RestDistance);
        gn2.restDistancesList.Add(m_RestDistance);

        m_Draw = draw;
        m_Thickness = thickness;

        if (draw)
        {
            if (gn1.availibleVertexPositions.Count == 0)
                Debug.LogError("No more vertices are available for rendering this connection");

            m_GravNode1VertexStart = gn1.availibleVertexPositions.Dequeue();

            if (gn2.availibleVertexPositions.Count == 0)
                Debug.LogError("No more vertices are available for rendering this connection");

            m_GravNode2VertexStart = gn2.availibleVertexPositions.Dequeue();
            m_TrianglesIndex = gravMesh.AddPair(m_GravNode1VertexStart, m_GravNode2VertexStart);
        }

        gn1.m_Connections++;
        gn2.m_Connections++;
    }

    internal void BreakLink(GravMesh gravMesh)
    {
        m_Gn1.m_Links.Remove(this);

        m_Gn1.neighborIndiciesList.Remove(m_Gn2.m_Index);
        m_Gn2.neighborIndiciesList.Remove(m_Gn1.m_Index);

        m_Gn1.restDistancesList.Remove(m_RestDistance);
        m_Gn2.restDistancesList.Remove(m_RestDistance);

        gravMesh.RemovePair(m_TrianglesIndex);
        m_Gn1.availibleVertexPositions.Enqueue(m_GravNode1VertexStart);
        m_Gn2.availibleVertexPositions.Enqueue(m_GravNode2VertexStart);

        m_Gn1.m_Connections--;
        m_Gn2.m_Connections--;
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
        public NativeArray<float> lineWidth;
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

            float3 perp = math.cross(math.normalize(position2 - position1) * lineWidth[index], look);
            int gn1VertexStart = gn1VertexStartIndices[index];
            int gn2VertexStart = gn2VertexStartIndices[index];

            vertixPositions[gn1VertexStart + 1] = position1 - perp;
            vertixPositions[gn1VertexStart] = position1 + perp;
            vertixPositions[gn2VertexStart + 1] = position2 - perp;
            vertixPositions[gn2VertexStart] = position2 + perp;
        }
    }
}

