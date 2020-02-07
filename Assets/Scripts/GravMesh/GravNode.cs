using System.Collections.Generic;
using Unity.Jobs;
using Unity.Mathematics;
using Unity.Burst;
using static Unity.Mathematics.math;
using UnityEngine;
using UnityEngine.Jobs;
using Vector3 = UnityEngine.Vector3;
using Unity.Collections;
using System;

public class GravNode
{
    public Transform lineRendererTransform { get; private set; }

    public bool m_Moveable;
    public float3 m_Position;
    public float3 m_PrevPosition;
    public float3 m_TargetPosition;
    public float3 m_ReturnPosition;
    public float3 m_Acceleration;
    public GravNodeCollider gravNodeColliderParent { get; private set; }

    public int m_Connections;
    public int m_Index;

    protected float rigidity = 1.0f;

    public List<int> neighborIndiciesList;
    public List<float> restDistancesList;
    public List<float> stiffnessesList;

    public Action<float3, int> SetTargetLocation;
    public Action<int> Return;
    public Action<float3, int> AddForce;

    public Func<int, float3> GetNodePosition;
    public int m_VertexStart;
    public Queue<int> availibleVertexPositions;

    public List<Link> m_Links;
    public List<Link.Dir> m_Directions; //0 left, 1 up, 2 right, 3 down, 4 diag

    public GravNode(float3 position, float spacing, int index, Transform anchorParent, int vertexStart, int layer)
    {
        m_Links = new List<Link>();
        m_Position = position;
        m_PrevPosition = position;
        m_TargetPosition = m_Position;
        m_ReturnPosition = m_Position;
        m_Acceleration = float3(0,0,0);
        m_Index = index;
        m_Moveable = true;
        gravNodeColliderParent = new GameObject("GravNodeAnchor").AddComponent<GravNodeCollider>();
        gravNodeColliderParent.gameObject.layer = layer;
        gravNodeColliderParent.SetSpacing(spacing/2);
        gravNodeColliderParent.SetMoveable(true);
        gravNodeColliderParent.affectorCollisionEnter += AffectorCollisionEnter;
        gravNodeColliderParent.affectorCollisionExit += AffectorCollisionExit;
        gravNodeColliderParent.GetNodeTransformFunc = GetNodePositionCallback;
        gravNodeColliderParent.transform.parent = anchorParent;
        gravNodeColliderParent.transform.localPosition = position;
        neighborIndiciesList = new List<int>();

        stiffnessesList = new List<float>();
        restDistancesList = new List<float>();
        m_Directions = new List<Link.Dir>();
        m_Connections = 0;
        m_VertexStart = vertexStart;
        availibleVertexPositions = new Queue<int>();
        // max of 4 pairs of verts availbles for drawing.
        for(int i = 0; i < 4; i++)
        {
            availibleVertexPositions.Enqueue(m_VertexStart + i * 2);
        }
    }

    private float3 GetNodePositionCallback()
    {
        return GetNodePosition.Invoke(m_Index);
    }

    public int FindLinkIndex(GravNode gn2)
    {
        for(int i = 0; i < m_Links.Count; i++)
        {
            if(m_Links[i].m_Gn2.m_Index == gn2.m_Index)
            {
                return m_Links[i].m_Index;
            }
        }
        return -1;
    }

    public void SetRootPos(Vector3 position)
    {
        gravNodeColliderParent.transform.localPosition = position;
    }

    public Vector3 GetRootPos()
    {
        return m_ReturnPosition;
    }

    public void SetHomePosition(Vector3 position)
    {
        m_Position = position;
        m_PrevPosition = position;
        m_TargetPosition = m_Position;
        m_ReturnPosition = m_Position;
        m_Acceleration = float3(0, 0, 0);
    }

    void AffectorCollisionEnter(float mass)
    {
        if (!m_Moveable)
            return;
        Vector3 newPose = m_ReturnPosition;
        newPose.z = mass;
        SetTargetPosition(newPose);
    }

    void AffectorCollisionExit()
    {
        Return(m_Index);
    }

    public void SetMoveable(bool b)
    {
        m_Moveable = b;
        gravNodeColliderParent.SetMoveable(b);
    }

    void SetTargetPosition(float3 position)
    {
        SetTargetLocation(position, m_Index);
    }

    public void ApplyForce(float3 force)
    {
        if (m_Moveable)
            AddForce(force, m_Index);
    }

    public struct MoveColliders : IJobParallelForTransform
    {
        public void Execute(int index, TransformAccess transform)
        {

        }
    }

    [BurstCompile]
    public struct SetOffsets : IJobParallelFor
    {
        [ReadOnly]
        public int2 worldStart;
        [ReadOnly]
        public int2 sectorStart;
        [ReadOnly]
        public int2 dimensions;
        [ReadOnly]
        public NativeArray<int2> coordinate;
        [WriteOnly]
        public NativeArray<float2> sectorOffsets;

        public void Execute(int index)
        {
            int xOffset = (coordinate[index].x + worldStart.x) - sectorStart.x;
            float t = ((float)xOffset / (float)dimensions.x + 1.0f) / 2.0f;
            int yOffset = (coordinate[index].y + worldStart.y) - sectorStart.y;
            float s = ((float)yOffset / (float)dimensions.y + 1.0f) / 2.0f;
            sectorOffsets[index] = new float2(t,s);
        }
    }

    [BurstCompile]
    public struct ShiftPlane2ElectricBoogaloo : IJobParallelFor
    {
        [ReadOnly]
        public NativeArray<float3> nodePoses;
        [ReadOnly]
        public NativeArray<float2> sectorOffsets;
        [ReadOnly]
        public NativeArray<float2> parentNodeOffsets;
        [ReadOnly]
        public int numTriangles;
        [ReadOnly]
        public NativeArray<int> triangles;
        [ReadOnly]
        public NativeArray<bool> affected;
        [WriteOnly]
        public NativeArray<float3> targetPositions;
        [WriteOnly]
        public NativeArray<float3> returnPositions;

        public void Execute(int index)
        {
            float3 coords = 0;
            int goldenTri = 0;
            for(int i = 0; i < numTriangles; i++)
            {
                float2 a = parentNodeOffsets[triangles[i * 3]];
                float2 b = parentNodeOffsets[triangles[i * 3 + 1]];
                float2 c = parentNodeOffsets[triangles[i * 3 + 2]];

                coords = Barycentric(sectorOffsets[index], a, b, c);
                if(!(coords.x < 0 || coords.y < 0 || coords.z < 0))
                {
                    goldenTri = i;
                    break;
                }
            }

            float3 aPose = nodePoses[goldenTri];
            float3 bPose = nodePoses[goldenTri];
            float3 cPose = nodePoses[goldenTri];

            float3 pose = aPose * coords.x + bPose * coords.y + cPose * coords.z;
            returnPositions[index] = pose;
            if (!affected[index])
                targetPositions[index] = pose;
        }

        public static float3 Barycentric(float2 p, float2 a, float2 b, float2 c)
        {
            float3 n;
            float2 v0 = b - a;
            float2 v1 = c - a;
            float2 v2 = p - a;
            //can be cached
            float d00 = math.dot(v0, v0);
            float d01 = math.dot(v0, v1);
            float d11 = math.dot(v1, v1);

            //cannot be cached
            float d20 = math.dot(v2, v0);
            float d21 = math.dot(v2, v1);

            //can be cached
            float denom = d00 * d11 - d01 * d01;
            n.x = (d11 * d20 - d01 * d21) / denom;
            n.y = (d00 * d21 - d01 * d20) / denom;
            n.z = 1.0f - n.x - n.y;
            return n;
        }
    }


    [BurstCompile]
    public struct ShiftPlane : IJobParallelFor
    {
        [ReadOnly]
        public float3 A;
        [ReadOnly]
        public float3 B;
        [ReadOnly]
        public float3 C;
        [ReadOnly]
        public float3 D;
        [ReadOnly]
        public float3 E;
        [ReadOnly]
        public NativeArray<float2> sectorOffsets;
        [ReadOnly]
        public NativeArray<bool> affected;
        [WriteOnly]
        public NativeArray<float3> targetPositions;
        [WriteOnly]
        public NativeArray<float3> returnPositions; 

        public void Execute(int index)
        {
            float t = sectorOffsets[index].x;
            float3 AB_interp = (1 - t) * C + t * D;
            float3 CD_interp = (1 - t) * A + t * B;

            float s = sectorOffsets[index].y;
            float3 final = (1 - s) * AB_interp + s * CD_interp;
            float3 pose = final;

            returnPositions[index] = pose;
            if (!affected[index])
                targetPositions[index] = pose;
        }
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

        public NativeArray<float3> positions;
        public NativeArray<float3> prevPositions;

        public void Execute(int index)
        {
            float3 acceleration = accelerations[index] + (targetPositions[index] - positions[index]);

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


