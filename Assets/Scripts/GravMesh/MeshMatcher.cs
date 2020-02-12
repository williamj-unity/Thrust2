using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Mathematics;
using Unity.Collections;
using Unity.Burst;
using Unity.Jobs;
using Unity.Profiling;

public class MeshMatcher 
{
    // Parent Grid Properties
    NativeArray<float3> parentGridPoses;
    NativeArray<float2> parentGridCoords;
    NativeArray<float> parentGridDenoms;

    NativeArray<int> parentGridTris;
    NativeArray<RaycastHit> m_RootNodeHits;
    NativeArray<RaycastCommand> m_RaycastCommands;

    GravNodeCollider[] colliderArray;

    int m_GridDimensions;
    LayerMask m_ParentLayer;
    float m_GridSpacing;
    int m_SectorDimensions;
    int m_RootNodeDimension;
    int m_TriangleCount;
    float m_SectorSpacing;
    int m_RootNodeCount;
    static ProfilerMarker s_SetAdjacentNodes = new ProfilerMarker("SetAdjacentNodes");
    static ProfilerMarker s_UpdateAdjacentNodes = new ProfilerMarker("UpdateAdjacentNodes");

    public MeshMatcher(int numSectors, LayerMask parentLayer, float spacing, int dimensions)
    {
        m_SectorDimensions = numSectors;
        m_SectorSpacing = dimensions/numSectors * spacing;

        m_ParentLayer = parentLayer;
        m_GridSpacing = spacing;
        m_GridDimensions = dimensions;

        m_RootNodeDimension = (numSectors * 2) + 1;
        int indexCount = (m_RootNodeDimension-1) * (m_RootNodeDimension-1) * 6;
        m_RootNodeCount = m_RootNodeDimension * m_RootNodeDimension;

        parentGridCoords = new NativeArray<float2>(m_RootNodeCount, Allocator.Persistent);
        parentGridPoses = new NativeArray<float3>(m_RootNodeCount, Allocator.Persistent);
        parentGridTris = new NativeArray<int>(indexCount, Allocator.Persistent);
        colliderArray = new GravNodeCollider[m_RootNodeCount];
        m_RootNodeHits = new NativeArray<RaycastHit>(m_RootNodeCount, Allocator.Persistent);
        m_RaycastCommands = new NativeArray<RaycastCommand>(m_RootNodeCount, Allocator.Persistent);

        float coordSpacing = 1.0f / (m_RootNodeDimension - 1.0f);
        for (int y = 0, index = 0; y < m_RootNodeDimension; y++)
            for(int x = 0; x < m_RootNodeDimension; x++, index++)
                parentGridCoords[index] = new float2(x * coordSpacing, y * coordSpacing);

        for (int ti = 0, vi = 0, y = 0; y < m_RootNodeDimension-1; y++, vi++)
        {
            for (int x = 0; x < m_RootNodeDimension-1; x++, ti += 6, vi++)
            {
                parentGridTris[ti] = vi;
                parentGridTris[ti + 3] = parentGridTris[ti + 2] = vi + 1;
                parentGridTris[ti + 4] = parentGridTris[ti + 1] = vi + m_RootNodeDimension;
                parentGridTris[ti + 5] = vi + m_RootNodeDimension + 1;
            }
        }

        parentGridDenoms = new NativeArray<float>(parentGridTris.Length / 3, Allocator.Persistent);


        for (int i = 0; i < parentGridTris.Length / 3; i++)
        {
            float2 a = parentGridCoords[parentGridTris[i * 3]];
            float2 b = parentGridCoords[parentGridTris[i * 3 + 1]];
            float2 c = parentGridCoords[parentGridTris[i * 3 + 2]];

            //parentGridDenoms[i] = denom;
        }
    }

    public void UpdateAdjacentNodes(int2 root, 
        NativeArray<float3> targetPositions, 
        NativeArray<float3> returnPositions, 
        NativeArray<bool> affected, 
        NativeArray<bool> forceUpdatePos, 
        NativeArray<float3> positions, 
        NativeArray<float3> prevPositions,
        NativeArray<float3> accelerations,
        NativeArray<float2> childOffsets)
    {
        s_UpdateAdjacentNodes.Begin();

        for (int index = 0; index < m_RootNodeCount; index++)
                parentGridPoses[index] = colliderArray[index] != null ? colliderArray[index].GetNodeTransform() : parentGridPoses[index];

        var UpdateReturnPosesJob = new ShiftPlane2ElectricBoogaloo()
        {
            parentNodeOffsets = parentGridCoords,
            parentNodePoses = parentGridPoses,
            childNodeOffsets = childOffsets,
            triangles = parentGridTris,
            numTriangles = parentGridTris.Length / 3,
            targetPositions = targetPositions,
            returnPositions = returnPositions,
            affected = affected,
            forceUpdatePos = forceUpdatePos,
            positions = positions,
            prevPositions = prevPositions,
            accelerations = accelerations,
            parentGridDenoms = parentGridDenoms
        };

        UpdateReturnPosesJob.Schedule(returnPositions.Length, 32).Complete();
        s_UpdateAdjacentNodes.End();
    }

    // root = the bottom left most sector.
    public void SetAdjacentNodes(int2 root)
    {
        s_SetAdjacentNodes.Begin();
        float3 centerOffset = new float3(root.x * m_GridSpacing, root.y * m_GridSpacing, 0.0f) - new float3(m_GridDimensions * m_GridSpacing, m_GridDimensions * m_GridSpacing, 0.0f);

        for (int y = 0, index = 0; y < m_RootNodeDimension; y++)
        {
            for (int x = 0; x < m_RootNodeDimension; x++, index++)
            {
                m_RaycastCommands[index] = new RaycastCommand(centerOffset + new float3(x * m_SectorSpacing, y * m_SectorSpacing, -1.0f), Vector3.forward, 1.5f, m_ParentLayer, 1);
            }
        }

        RaycastCommand.ScheduleBatch(m_RaycastCommands, m_RootNodeHits, 8).Complete();

        for (int y = 0, index = 0; y < m_RootNodeDimension; y++)
        {
            for (int x = 0; x < m_RootNodeDimension; x++, index++)
            {
                colliderArray[index] = (m_RootNodeHits[index].collider != null) ? m_RootNodeHits[index].collider.GetComponent<GravNodeCollider>() : null;
                parentGridPoses[index] = (colliderArray[index] != null) ? colliderArray[index].GetNodeTransform() : centerOffset + new float3(x * m_SectorSpacing, y * m_SectorSpacing, 0.0f);
            }
        }
        s_SetAdjacentNodes.End();
    }

    public float3 GetNodePoseFromBarycentrics(float2 offset, int2 root)
    {
        float3 coords = 0;
        int goldenTri = -1;
        int triCount = parentGridTris.Length / 3;
        for (int i = 0; i < triCount; i++)
        {
            float2 a = parentGridCoords[parentGridTris[i * 3]];
            float2 b = parentGridCoords[parentGridTris[i * 3 + 1]];
            float2 c = parentGridCoords[parentGridTris[i * 3 + 2]];

            coords = ShiftPlane2ElectricBoogaloo.Barycentric(offset, a, b, c);
            if (!(coords.x < 0 || coords.y < 0 || coords.z < 0))
            {
                goldenTri = i;
                break;
            }
        }

        if(goldenTri == -1)
        {
            Debug.Log("could not triangulate. using default pos");
            return new float3(offset.x * m_GridSpacing + root.x * m_GridSpacing, offset.y * m_GridSpacing + root.y * m_GridSpacing, 0.0f);
        }

        float3 aPose = parentGridPoses[parentGridTris[goldenTri * 3]];
        float3 bPose = parentGridPoses[parentGridTris[goldenTri * 3 + 1]];
        float3 cPose = parentGridPoses[parentGridTris[goldenTri * 3 + 2]];

        float3 pose = aPose * coords.z + bPose * coords.x + cPose * coords.y;
        return pose;
    }

    public void DebugSpheres()
    {
        if (!parentGridPoses.IsCreated)
            return;
        Gizmos.color = Color.red;

        for (int i = 0; i < m_RaycastCommands.Length; i++)
        {
            if (m_RootNodeHits[i].collider == null)
                Gizmos.color = Color.white;
            else
                Gizmos.color = Color.red;
            Gizmos.DrawSphere(m_RaycastCommands[i].from, 0.1f);
            Gizmos.DrawRay(m_RaycastCommands[i].from, m_RaycastCommands[i].direction * m_RaycastCommands[i].distance);
        }
    }

    public void Cleanup()
    {
        parentGridPoses.Dispose();
        parentGridCoords.Dispose();
        parentGridTris.Dispose();
        m_RootNodeHits.Dispose();
        m_RaycastCommands.Dispose();
        parentGridDenoms.Dispose();
    }
}

[BurstCompile]
public struct ShiftPlane2ElectricBoogaloo : IJobParallelFor
{
    [ReadOnly]
    public NativeArray<float3> parentNodePoses;
    [ReadOnly]
    public NativeArray<float2> childNodeOffsets;
    [ReadOnly]
    public NativeArray<float2> parentNodeOffsets;
    [ReadOnly]
    public int numTriangles;
    [ReadOnly]
    public NativeArray<int> triangles;
    [ReadOnly]
    public NativeArray<bool> affected;
    [ReadOnly]
    public NativeArray<float> parentGridDenoms;
    [ReadOnly]
    public NativeArray<bool> forceUpdatePos;
    [WriteOnly]
    public NativeArray<float3> targetPositions;
    [WriteOnly]
    public NativeArray<float3> returnPositions;
    [WriteOnly]
    public NativeArray<float3> positions;
    [WriteOnly]
    public NativeArray<float3> prevPositions;
    [WriteOnly]
    public NativeArray<float3> accelerations;


    public void Execute(int index)
    {
        float3 coords = 0;
        int goldenTri = -1;
        for (int i = 0; i < numTriangles; i++)
        {
            float2 a = parentNodeOffsets[triangles[i * 3]];
            float2 b = parentNodeOffsets[triangles[i * 3 + 1]];
            float2 c = parentNodeOffsets[triangles[i * 3 + 2]];

            coords = Barycentric(childNodeOffsets[index], a, b, c);
            if (!(coords.x < 0 || coords.y < 0 || coords.z < 0))
            {
                goldenTri = i;
                break;
            }
        }

        float3 aPose = parentNodePoses[triangles[goldenTri * 3]];
        float3 bPose = parentNodePoses[triangles[goldenTri * 3 + 1]];
        float3 cPose = parentNodePoses[triangles[goldenTri * 3 + 2]];

        float3 pose = aPose * coords.z + bPose * coords.x + cPose * coords.y;
        returnPositions[index] = pose;
        if (!affected[index])
            targetPositions[index] = pose;
        if(forceUpdatePos[index])
        {
            targetPositions[index] = pose;
            positions[index] = pose;
            prevPositions[index] = pose;
            accelerations[index] = 0;
        }
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

        //can be cached
        n.x = (d11 * d20 - d01 * d21) / denom;
        n.y = (d00 * d21 - d01 * d20) / denom;
        n.z = 1.0f - n.x - n.y;
        return n;
    }
}
