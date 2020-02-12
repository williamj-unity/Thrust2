using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using System;
using Unity.Mathematics;
using Unity.Profiling;
using UnityEngine.Jobs;

public class GravGridBuilder : MonoBehaviour
{
    static ProfilerMarker s_SeOffsets = new ProfilerMarker("SetOffsets");
    static ProfilerMarker s_UpdateMeshVerticies = new ProfilerMarker("UpdateMeshVertices");

    static ProfilerMarker s_TotalSolverTime = new ProfilerMarker("JobsSolverTime");

    static ProfilerMarker s_PrepareSolver = new ProfilerMarker("PrepareConstraintSolver");
    static ProfilerMarker s_PreparePhsycisStep = new ProfilerMarker("PreparePhysicsStep");
    static ProfilerMarker s_ResetAccelerations = new ProfilerMarker("ResetAccelertions");

    static ProfilerMarker s_SetSolverTransforms = new ProfilerMarker("SetSolverTransforms");
    static ProfilerMarker s_Accumlate = new ProfilerMarker("Accumulate");

    static ProfilerMarker s_UpdateNodePosesOnShift = new ProfilerMarker("UpdateNodePosesOnShift");
    static ProfilerMarker s_UpdateJobs = new ProfilerMarker("UpdateJobs");

    static ProfilerMarker s_ResetConnections = new ProfilerMarker("ResetConnections");
    static ProfilerMarker s_AddConnections = new ProfilerMarker("AddConnections");
    static ProfilerMarker s_SetConnectionProperties = new ProfilerMarker("SetConnectionProperties");
    static ProfilerMarker s_SetGravNodeConnectionProperties = new ProfilerMarker("SetGravNodeConnectionProperties");
    static ProfilerMarker s_ConstructuMesh = new ProfilerMarker("ConstructMesh");
    static ProfilerMarker s_UpdateMesh = new ProfilerMarker("UpdateMesh");
    static ProfilerMarker s_CreateNewLink = new ProfilerMarker("CreateNewLink");

    private List<Link> connections = new List<Link>();
    private GravNode[,] gravGrid;
    private List<GravNode> gravNodes;
    public int SolverIterations = 15;

    public float dampingCoefficient = 2.0f;

    public float fixedDeltaTime = 16.0f;
    private float fixedDeltaTimeSeconds;


    [Header("Build time settings")]
    public int gravGridDimensions;
    public Vector2Int startingRootCoordinate;
    public float spacing;
    public float planeLinkStiffness = 1.0f;
    public float lineWidth = 0.1f;
    public LayerMask parentLayer;

    private float mLeftOverTime = 0.0f;

    NativeArray<int> startConnectionsIndex;
    NativeArray<int> nodeNumConnections;
    NativeArray<int> allNodeConnections;
    NativeArray<float> allRestsDistances;

    NativeArray<float2> offsets;
    NativeArray<float3> targetPositions;
    NativeArray<float3> returnPositions;
    NativeArray<bool> affected;
    NativeArray<bool> forceUpdatePosition;

    NativeArray<float3> prevPositions;
    NativeArray<float3> positions;
    NativeArray<float3> newPositions;
    TransformAccessArray colliderTransformArray;

    NativeArray<float3> accelerations;
    NativeArray<bool> moveables;

    NativeArray<int> gn1Index;
    NativeArray<int> gn2Index;
    NativeArray<int2> coordinates; 
    NativeArray<bool> drawables;
    NativeArray<int> gn1VertexStartIndices;
    NativeArray<int> gn2VertexStartIndices;
    NativeArray<float> lineWidths;
    NativeArray<float> baseLineWidths;

    Queue<int> availableConnectionIndex;

    GravMesh m_GravMesh;
    MeshMatcher m_MeshMatcher;
    Camera gravGridCam;

    int MAX_CONNECTIONS = 16;
    int mStartWorldX = 0;
    int mStartWorldY = 0;
    int2 rootCoord = 0;
    public int sectorSize = 10; 
    
    private float lineWidthScalar = 1.0f;

    void Start()
    {
        gravGridCam = Camera.main;
        fixedDeltaTimeSeconds = (float)fixedDeltaTime / 1000.0f;
        gravGrid = new GravNode[gravGridDimensions, gravGridDimensions];
        gravNodes = new List<GravNode>(gravGridDimensions * gravGridDimensions);
        float gridSizeWorld = gravGridDimensions * spacing;
        m_GravMesh = new GravMesh(new Vector3(gridSizeWorld, gridSizeWorld, 0.5f), gridSizeWorld, GetComponent<MeshFilter>());

        m_MeshMatcher = new MeshMatcher(gravGridDimensions/sectorSize, parentLayer, spacing, gravGridDimensions);
        m_MeshMatcher.SetAdjacentNodes(rootCoord);

        availableConnectionIndex = new Queue<int>();
        offsets = new NativeArray<float2>(gravGridDimensions * gravGridDimensions, Allocator.Persistent);
        coordinates = new NativeArray<int2>(gravGridDimensions * gravGridDimensions, Allocator.Persistent);
        rootCoord = new int2(startingRootCoordinate.x, startingRootCoordinate.y);
        colliderTransformArray = new TransformAccessArray(gravGridDimensions * gravGridDimensions);

        for (int j = 0, index = 0; j < gravGridDimensions; ++j)
        {
            int yOffset = (j + mStartWorldY) - rootCoord.y;
            float s = ((float)yOffset / (float)gravGridDimensions + 1.0f) / 2.0f;


            for (int i = 0; i < gravGridDimensions; ++i, index++)
            {
                int xOffset = (i + mStartWorldX) - rootCoord.x;
                float t = ((float)xOffset / (float)gravGridDimensions + 1.0f) / 2.0f;

                Vector3 position = new Vector3(i * spacing, j * spacing, 0);
                int vertexStartIndex = m_GravMesh.AddNode();
                offsets[index] = new float2(t, s);
                coordinates[index] = new int2(i, j);

                var gn = new GravNode(position, spacing, index, transform, vertexStartIndex, gameObject.layer);
                colliderTransformArray.Add(gn.gravNodeColliderParent.transform);
                gn.AddForce = ApplyForce;
                gn.SetTargetLocation = SetTargetLocation;
                gn.Return = SetTargetToReturnPosition;
                gn.GetNodePosition = GetNodePosition;
                gn.GetReturnPosition = GetReturnPosition;
                gravGrid[i, j] = gn;
                gravNodes.Add(gn);
            }
        }

        nodeNumConnections = new NativeArray<int>(gravNodes.Count, Allocator.Persistent);
        startConnectionsIndex = new NativeArray<int>(gravNodes.Count, Allocator.Persistent);
        allNodeConnections = new NativeArray<int>(gravNodes.Count * MAX_CONNECTIONS, Allocator.Persistent);
        allRestsDistances = new NativeArray<float>(gravNodes.Count * MAX_CONNECTIONS, Allocator.Persistent);

        targetPositions = new NativeArray<float3>(gravNodes.Count, Allocator.Persistent);
        returnPositions = new NativeArray<float3>(gravNodes.Count, Allocator.Persistent);
        affected = new NativeArray<bool>(gravNodes.Count, Allocator.Persistent);
        prevPositions = new NativeArray<float3>(gravNodes.Count, Allocator.Persistent);
        positions = new NativeArray<float3>(gravNodes.Count, Allocator.Persistent);
        newPositions = new NativeArray<float3>(gravNodes.Count, Allocator.Persistent);
        accelerations = new NativeArray<float3>(gravNodes.Count, Allocator.Persistent);
        moveables = new NativeArray<bool>(gravNodes.Count, Allocator.Persistent);
        forceUpdatePosition = new NativeArray<bool>(gravNodes.Count, Allocator.Persistent);

        for (int j = 0; j < gravNodes.Count; j++)
        {
            accelerations[j] = gravNodes[j].m_Acceleration;
            prevPositions[j] = gravNodes[j].m_PrevPosition;
            targetPositions[j] = gravNodes[j].m_TargetPosition;
            returnPositions[j] = gravNodes[j].m_ReturnPosition;
            affected[j] = false;
            moveables[j] = gravNodes[j].m_Moveable;
            positions[j] = gravNodes[j].m_Position;
        }

        ResetConnections();
    }

    internal void SetOffsets()
    {
        s_SeOffsets.Auto();
        for (int j = 0; j < gravGridDimensions; j++)
        {
            int yOffset = (j + mStartWorldY) - rootCoord.y;
            float s = ((float)yOffset / (float)gravGridDimensions + 1.0f) / 2.0f;
            for (int i = 0; i < gravGridDimensions; i++)
            {
                int x = mod((mStartWorldX + i), gravGridDimensions);
                int y = mod((mStartWorldY + j), gravGridDimensions);
                GravNode gn = gravGrid[x, y];
                int index = gn.m_Index;
                int xOffset = (i + mStartWorldX) - rootCoord.x;
                float t = ((float)xOffset / (float)gravGridDimensions + 1.0f) / 2.0f;

                offsets[index] = new float2(t, s);
            }
        }
        //var SetOffsetsJob = new GravNode.SetOffsets()
        //{
        //    coordinate = coordinates,
        //    sectorOffsets = offsets,
        //    dimensions = new int2(mHorizontalParticles, mVerticalParticles),
        //    worldStart = new int2(mStartWorldX, mStartWorldY),
        //    sectorStart = sector
        //};
        //SetOffsetsJob.Schedule(coordinates.Length, 64).Complete();
    }

    internal void SetLineWidthScalar(float newScale)
    {
        lineWidthScalar = newScale;
    }

    internal void SetScaleRange(float min, float max)
    {
        m_GravMesh.SetRange(min, max);
    }

    internal void SetCurrentZoom(float currentScale)
    {
        m_GravMesh.SetCurrentZoom(currentScale);
    }

    public void ResetConnections()
    {
        s_ResetConnections.Begin();
        connections.Clear();

        for(int i = 0; i < gravNodes.Count; i++)
        {
            gravNodes[i].m_Connections = 0;
            gravNodes[i].neighborIndiciesList.Clear();
            gravNodes[i].restDistancesList.Clear();
            gravNodes[i].stiffnessesList.Clear();
        }

        s_AddConnections.Begin();
        for (int j = 0; j < gravGridDimensions; j++)
        {
            int y = mod((mStartWorldY + j), gravGridDimensions);
            for (int i = 0; i < gravGridDimensions; i++)
            {
                int x = mod((mStartWorldX + i), gravGridDimensions);
                float thickcessX = lineWidth;
                float thickcessY = lineWidth;
                if (y % sectorSize == 0)
                    thickcessX *= 3;
                if (x % sectorSize == 0)
                    thickcessY *= 3;

                if (i < gravGridDimensions - 1) AddConnection(gravGrid[x, y], gravGrid[mod((x + 1), gravGridDimensions), y],  true, thickcessX, m_GravMesh);
                if(j < gravGridDimensions - 1) AddConnection(gravGrid[x, y], gravGrid[x, mod((y + 1), gravGridDimensions)], true, thickcessY, m_GravMesh);
                //if (x < mHorizontalParticles - 1 && y < mVerticalParticles - 1) AddConnection(gravGrid[x, y], gravGrid[x + 1, y + 1], drawDiagonals, planeLinkStiffness, m_GravMesh);
                //if (x < mHorizontalParticles - 1 && y < mVerticalParticles - 1) AddConnection(gravGrid[x + 1, y], gravGrid[x, y + 1], drawDiagonals, planeLinkStiffness, m_GravMesh);
                //configure extended connections
                //if (x < mHorizontalParticles - 2) AddConnection(gravGrid[x, y], gravGrid[x + 2, y], drawDiagonals, planeLinkStiffness, m_GravMesh);
                //if (y < mVerticalParticles - 2) AddConnection(gravGrid[x, y], gravGrid[x, y + 2], drawDiagonals, planeLinkStiffness, m_GravMesh);
                //if (x < mHorizontalParticles - 2 && y < mVerticalParticles - 2) AddConnection(gravGrid[x, y], gravGrid[x + 2, y + 2], drawDiagonals, planeLinkStiffness, m_GravMesh);
                //if (x < mHorizontalParticles - 2 && y < mVerticalParticles - 2) AddConnection(gravGrid[x + 2, y], gravGrid[x, y + 2], drawDiagonals, planeLinkStiffness, m_GravMesh);

                if (x == mStartWorldX || y == mStartWorldY || x == mStartWorldX + gravGridDimensions - 1 || y == mStartWorldY + gravGridDimensions - 1) gravGrid[x, y].SetMoveable(false);
                else gravGrid[x, y].SetMoveable(true);
            }
        }
        s_AddConnections.End();

        if (!gn1Index.IsCreated)
            gn1Index = new NativeArray<int>(connections.Count, Allocator.Persistent);
        if (!gn2Index.IsCreated)
            gn2Index = new NativeArray<int>(connections.Count, Allocator.Persistent);
        if (!gn1VertexStartIndices.IsCreated)
            gn1VertexStartIndices = new NativeArray<int>(connections.Count, Allocator.Persistent);
        if (!gn2VertexStartIndices.IsCreated)
            gn2VertexStartIndices = new NativeArray<int>(connections.Count, Allocator.Persistent);
        if (!drawables.IsCreated)
            drawables = new NativeArray<bool>(connections.Count, Allocator.Persistent);
        if (!lineWidths.IsCreated)
            lineWidths = new NativeArray<float>(connections.Count, Allocator.Persistent);
        if (!baseLineWidths.IsCreated)
            baseLineWidths = new NativeArray<float>(connections.Count, Allocator.Persistent);


        s_ConstructuMesh.Begin();
        m_GravMesh.ConstructMesh();
        s_ConstructuMesh.End();

        s_SetConnectionProperties.Begin();
        for (int j = 0; j < connections.Count; j++)
        {
            gn1Index[j] = connections[j].m_GravNode1PosIndex;
            gn2Index[j] = connections[j].m_GravNode2PosIndex;
            drawables[j] = connections[j].m_Draw;
            gn1VertexStartIndices[j] = connections[j].m_GravNode1VertexStart;
            gn2VertexStartIndices[j] = connections[j].m_GravNode2VertexStart;
            lineWidths[j] = connections[j].m_Thickness;
            baseLineWidths[j] = connections[j].m_Thickness;
        }
        s_SetConnectionProperties.End();


        s_SetGravNodeConnectionProperties.Begin();
        int startIndex = 0;
        for (int j = 0; j < gravNodes.Count; j++)
        {
            GravNode n = gravNodes[j];
            nodeNumConnections[j] = n.m_Connections;
            startConnectionsIndex[j] = startIndex;
            for (int i = 0; i < n.m_Connections; i++)
            {
                allNodeConnections[i + startIndex] = n.neighborIndiciesList[i];
                allRestsDistances[i + startIndex] = n.restDistancesList[i];
            }
            startIndex += MAX_CONNECTIONS;
            moveables[j] = n.m_Moveable;
        }
        s_SetGravNodeConnectionProperties.End();
        s_ResetConnections.End();
    }

    public void AddConnection(GravNode gn1, GravNode gn2, bool draw, float thickness, GravMesh gravMesh)
    {
        Link l = new Link(gn1, gn2, draw, gravMesh, thickness, connections.Count, spacing);
        connections.Add(l);
    }
   
    float3 GetNodePosition(int index)
    {
        return new float3(transform.position) + positions[index];
    }

    float3 GetReturnPosition(int index)
    {
        return returnPositions[index];
    }

    void OnDrawGizmosSelected()
    {
        if(m_MeshMatcher != null)
            m_MeshMatcher.DebugSpheres();
    }

    void SetTargetLocation(float3 target, int index)
    {
        targetPositions[index] = target;
        affected[index] = true;
    }

    void SetTargetToReturnPosition(int index)
    {
        targetPositions[index] = returnPositions[index];
        affected[index] = false;
    }

    void ApplyForce(float3 force, int index)
    {
        float3 acc = accelerations[index];
        accelerations[index] = acc + force;
    }

    public enum GridShiftDirection
    {
        Up,
        Down,
        Left,
        Right
    }

    public void SectorShift(int2 shiftBy)
    {
        rootCoord += shiftBy;
        SetOffsets();
        m_MeshMatcher.SetAdjacentNodes(rootCoord);
    }

    public float3 GetNodePoseFromBarycentrics(float2 offset)
    {
        return m_MeshMatcher.GetNodePoseFromBarycentrics(offset, rootCoord);
    }

    public void ShiftGrid(GridShiftDirection gridShiftDirection)
    {
        switch (gridShiftDirection)
        {
            case GridShiftDirection.Right:
                {
                    int newStart = mStartWorldX + 1;
                    int oldEnd = mod(mStartWorldX + gravGridDimensions - 1, gravGridDimensions);
                    int newStartX = mod(newStart, gravGridDimensions);
                    int oldStart = mod(mStartWorldX, gravGridDimensions);
                    int newWorldXPos = mStartWorldX + gravGridDimensions;

                    for (int j = 0; j < gravGridDimensions; j++)
                    {
                        int y = mod((mStartWorldY + j), gravGridDimensions);


                        s_UpdateNodePosesOnShift.Begin();

                        int yOffset = (mStartWorldY + j) - rootCoord.y;
                        int xOffset = newWorldXPos - rootCoord.x;
                        float t = ((float)xOffset / (float)gravGridDimensions + 1.0f) / 2.0f;
                        float s = ((float)yOffset / (float)gravGridDimensions + 1.0f) / 2.0f;


                        GravNode newStartGN = gravGrid[newStartX, y];
                        GravNode oldStartNewEndGN = gravGrid[oldStart, y];
                        GravNode oldEndGN = gravGrid[oldEnd, y];
                        offsets[oldStartNewEndGN.m_Index] = new float2(t, s);
                        forceUpdatePosition[oldStartNewEndGN.m_Index] = true;

                        ConfigureNewStartAndEndNodes(
                            oldStartNewEndGN, //break
                            newStartGN, //break
                            oldEndGN,   // connect
                            oldStartNewEndGN, //connect
                            newStartGN, //start
                            oldStartNewEndGN, //end
                            oldEndGN, //middle
                            y, mStartWorldY, gravGridDimensions);
                        s_UpdateNodePosesOnShift.End();
                    }
                    mStartWorldX = newStart;
                    if (oldStart == startingRootCoordinate.x) // sector is moving!!
                        SectorShift(new int2(gravGridDimensions, 0));
                    break;
                }

            case GridShiftDirection.Up:
                {
                    int newStart = mStartWorldY + 1;
                    int oldEnd = mod(mStartWorldY + gravGridDimensions - 1, gravGridDimensions);
                    int newStartY = mod(newStart, gravGridDimensions);
                    int oldStart = mod(mStartWorldY, gravGridDimensions);
                    int newWorldYPos = mStartWorldY + gravGridDimensions;

                    for (int j = 0; j < gravGridDimensions; j++)
                    {
                        s_UpdateNodePosesOnShift.Begin();

                        int x = mod((mStartWorldX + j), gravGridDimensions);

                        int yOffset = newWorldYPos - rootCoord.y;
                        int xOffset = (j + mStartWorldX) - rootCoord.x;

                        float t = ((float)xOffset / (float)gravGridDimensions + 1.0f) / 2.0f;
                        float s = ((float)yOffset / (float)gravGridDimensions + 1.0f) / 2.0f;

                        GravNode newStartGN = gravGrid[x, newStartY];
                        GravNode oldStartNewEndGN = gravGrid[x, oldStart];
                        GravNode oldEndGN = gravGrid[x, oldEnd];
                        offsets[oldStartNewEndGN.m_Index] = new float2(t, s);
                        forceUpdatePosition[oldStartNewEndGN.m_Index] = true;

                        ConfigureNewStartAndEndNodes(
                            oldStartNewEndGN, //break
                            newStartGN, //break
                            oldEndGN,   // connect
                            oldStartNewEndGN, //connect
                            newStartGN, //start
                            oldStartNewEndGN, //end
                            oldEndGN, //middle
                            x, mStartWorldX, gravGridDimensions);
                        s_UpdateNodePosesOnShift.End();
                    }
                    mStartWorldY = newStart;
                    if (oldStart == startingRootCoordinate.y) // sector is moving!!
                        SectorShift(new int2(0, gravGridDimensions));

                    break;
                }
            case GridShiftDirection.Left:
                {
                    int newStart = mStartWorldX - 1;
                    int oldStart = mod(mStartWorldX, gravGridDimensions);
                    int oldEnd = mod(mStartWorldX + gravGridDimensions - 1, gravGridDimensions);
                    int newEnd = mod(mStartWorldX + gravGridDimensions - 2, gravGridDimensions);

                    for (int j = 0; j < gravGridDimensions; j++)
                    {
                        s_UpdateNodePosesOnShift.Begin();

                        int y = mod((mStartWorldY + j), gravGridDimensions);

                        int yOffset = (j + mStartWorldY) - rootCoord.y;
                        int xOffset = newStart - rootCoord.x;
                        float t = ((float)xOffset / (float)gravGridDimensions + 1.0f) / 2.0f;
                        float s = ((float)yOffset / (float)gravGridDimensions + 1.0f) / 2.0f;


                        GravNode oldStartGN = gravGrid[oldStart, y];
                        GravNode newEndGN = gravGrid[newEnd, y];
                        GravNode oldEndNewStartGN = gravGrid[oldEnd, y];
                        offsets[oldEndNewStartGN.m_Index] = new float2(t, s);
                        forceUpdatePosition[oldEndNewStartGN.m_Index] = true;


                        ConfigureNewStartAndEndNodes(
                            newEndGN, //break
                            oldEndNewStartGN, //break
                            oldEndNewStartGN,   // connect
                            oldStartGN,  //connect
                            oldEndNewStartGN, //start
                            newEndGN, //end
                            oldStartGN, //middle
                            y, mStartWorldY, gravGridDimensions);
                        s_UpdateNodePosesOnShift.End();
                    }
                    mStartWorldX = newStart;
                    if (oldEnd == startingRootCoordinate.x) // sector is moving!!
                        SectorShift(new int2(-gravGridDimensions, 0));
                    break;
                }
            case GridShiftDirection.Down:
                {
                    int newStart = mStartWorldY - 1;
                    int oldStart = mod(mStartWorldY, gravGridDimensions);
                    int oldEnd = mod(mStartWorldY + gravGridDimensions - 1, gravGridDimensions);
                    int newEnd = mod(mStartWorldY + gravGridDimensions - 2, gravGridDimensions);

                    for (int j = 0; j < gravGridDimensions; j++)
                    {
                        s_UpdateNodePosesOnShift.Begin();

                        int x = mod((mStartWorldX + j), gravGridDimensions);

                        int yOffset = newStart - rootCoord.y;
                        int xOffset = (j + mStartWorldX) - rootCoord.x;
                        float t = ((float)xOffset / (float)gravGridDimensions + 1.0f) / 2.0f;
                        float s = ((float)yOffset / (float)gravGridDimensions + 1.0f) / 2.0f;

                        GravNode oldStartGN = gravGrid[x, oldStart];
                        GravNode newEndGN = gravGrid[x, newEnd];
                        GravNode oldEndNewStartGN = gravGrid[x, oldEnd];
                        offsets[oldEndNewStartGN.m_Index] = new float2(t, s);
                        forceUpdatePosition[oldEndNewStartGN.m_Index] = true;

                        ConfigureNewStartAndEndNodes(
                            newEndGN, //break
                            oldEndNewStartGN, //break
                            oldEndNewStartGN,   // connect
                            oldStartGN,  //connect
                            oldEndNewStartGN, //start
                            newEndGN, //end
                            oldStartGN, //middle
                            x, mStartWorldX, gravGridDimensions);
                        s_UpdateNodePosesOnShift.End();
                    }
                    mStartWorldY = newStart;
                    if (oldEnd == startingRootCoordinate.y) // sector is moving!!
                        SectorShift(new int2(0, -gravGridDimensions));
                    break;
                }
            default:
                break;
        }
    }

    public void ConfigureNewStartAndEndNodes(
        GravNode break1, 
        GravNode break2, 
        GravNode connect1, 
        GravNode connect2,
        GravNode newStart,
        GravNode newEnd,
        GravNode newMid,
        int row, int start, int max)
    {
        s_AddConnections.Begin();
        RemoveConnection(break1, break2);
        AddAndSetupConnectionAtNextIndex(connect1, connect2, true, m_GravMesh);
        s_AddConnections.End();

        s_SetGravNodeConnectionProperties.Begin();
        if (row != start && row != start + max - 1)
        {
            int midIndex = newMid.m_Index;
            newMid.SetMoveable(true);
            moveables[midIndex] = true;
        }

        int newStartXIndex = newStart.m_Index;
        affected[newStartXIndex] = false;
        moveables[newStartXIndex] = false;
        newStart.SetMoveable(false);

        int newEndXINdex = newEnd.m_Index;
        affected[newEndXINdex] = false;
        moveables[newEndXINdex] = false;
        newEnd.SetMoveable(false);

        s_SetGravNodeConnectionProperties.End();
    }

    static ProfilerMarker s_RemoveLinkCreateIP = new ProfilerMarker("RemoveLinkeCreateIP");

    public void AddAndSetupConnectionAtNextIndex(GravNode gn1, GravNode gn2, bool draw, GravMesh gravMesh)
    {
        if (availableConnectionIndex.Count == 0)
            Debug.LogError("There are no available connections.");
        s_CreateNewLink.Begin();
        int index = availableConnectionIndex.Dequeue();
        //We're not going to update the thickness data as to preserve the sector location.
        connections[index].MakeLink(gn1, gn2, draw, gravMesh, 0, spacing);
        s_CreateNewLink.End();

        gn1Index[index] = connections[index].m_GravNode1PosIndex;
        gn2Index[index] = connections[index].m_GravNode2PosIndex;
        drawables[index] = connections[index].m_Draw;
        gn1VertexStartIndices[index] = connections[index].m_GravNode1VertexStart;
        gn2VertexStartIndices[index] = connections[index].m_GravNode2VertexStart;

        int gn1I = gn1.m_Index;
        int gn2I = gn2.m_Index;

        nodeNumConnections[gn1I] = gn1.m_Connections;
        nodeNumConnections[gn2I] = gn2.m_Connections;
        int connectionsStart = startConnectionsIndex[gn1I];
        for (int i = 0; i < gn1.m_Connections; i++)
        {
            allNodeConnections[i + connectionsStart] = gn1.neighborIndiciesList[i];
            allRestsDistances[i + connectionsStart] = gn1.restDistancesList[i];
        }

        connectionsStart = startConnectionsIndex[gn2I];
        for (int i = 0; i < gn2.m_Connections; i++)
        {
            allNodeConnections[i + connectionsStart] = gn2.neighborIndiciesList[i];
            allRestsDistances[i + connectionsStart] = gn2.restDistancesList[i];
        }
    }

    private void RemoveConnection(GravNode gn1, GravNode gn2)
    {
        s_RemoveLinkCreateIP.Begin();
        int indexResult = gn1.FindLinkIndex(gn2);
        if(indexResult == -1)
        {
            Debug.LogError("Link not found");
            return;
        }
        s_RemoveLinkCreateIP.End();

        connections[indexResult].BreakLink(m_GravMesh);
        availableConnectionIndex.Enqueue(indexResult);
    }

    public Vector3 GetMeshCenterWorld()
    {
        if (!positions.IsCreated)
            return Vector3.zero;
        int x = mod(mStartWorldX + gravGridDimensions / 2, gravGridDimensions);
        int y = mod(mStartWorldY + gravGridDimensions / 2, gravGridDimensions);

        float3 p = positions[gravGrid[x, y].m_Index];
        Vector4 v = new Vector4(p.x,p.y, p.x, 1.0f);
        v = transform.localToWorldMatrix * v;
        v.z = 0;

        return v;
    }

    int mod(int x, int m)
    {
        return (x % m + m) % m;
    }

    void Update()
    {
        s_PreparePhsycisStep.Begin();
        float elapsedTime = Time.deltaTime;
        elapsedTime += mLeftOverTime;
        int timeSteps = Mathf.FloorToInt(elapsedTime / fixedDeltaTimeSeconds);
        timeSteps = Math.Min(timeSteps, 5);
        mLeftOverTime = elapsedTime - timeSteps * fixedDeltaTimeSeconds;
        float4x4 cameraWolrdMatrix = gravGridCam.cameraToWorldMatrix;
        float3 cameraPos = gravGridCam.transform.position;
        s_PreparePhsycisStep.End();

        m_MeshMatcher.UpdateAdjacentNodes(
            rootCoord, 
            targetPositions, 
            returnPositions, 
            affected,
            forceUpdatePosition, 
            positions, 
            prevPositions, 
            accelerations, 
            offsets);

        var updateColliderJob = new GravNode.MoveColliders
        {
            positions = positions,
            forceUpdatePosition = forceUpdatePosition
        };

        var ColliderUpdateHandle = updateColliderJob.Schedule(colliderTransformArray);
        ColliderUpdateHandle.Complete();

        for (int z = 0; z < timeSteps; z++)
        {
            s_TotalSolverTime.Begin();
            for (int i = 0; i < SolverIterations; i++)
            {
                var jobData = new GravNode.SolveContraints
                {
                    allneighbors = allNodeConnections,
                    allRestDistances = allRestsDistances,
                    startIndex = startConnectionsIndex,
                    linkStiffness = planeLinkStiffness,
                    numConnections = nodeNumConnections,
                    moveable = moveables,
                    positions = positions,
                    outPositions = newPositions
                };
                JobHandle handle = jobData.Schedule(gravNodes.Count, 64);
                handle.Complete();

                var swapJobData = new GravNode.SwapBuffers
                {
                    newPositions = newPositions,
                    positions = positions
                };

                handle = swapJobData.Schedule(gravNodes.Count, 64, ColliderUpdateHandle);
                handle.Complete();
            }
            s_TotalSolverTime.End();

            s_UpdateJobs.Begin();
            var updateJobData = new GravNode.UpdateJob()
            {
                damping = dampingCoefficient,
                timeStep = fixedDeltaTimeSeconds,
                positions = positions,
                accelerations = accelerations,
                prevPositions = prevPositions,
                targetPositions = targetPositions
            };
            JobHandle updateJobHandle = updateJobData.Schedule(gravNodes.Count, 128);
            updateJobHandle.Complete();
            s_UpdateJobs.End();

            //TODO:: don't always need to update this.
            var updateLineWidths = new Link.UpdateLineThickness()
            {
                lineWidthScalar = lineWidthScalar,
                baseWidth = baseLineWidths,
                lineWidths = lineWidths
            };
            updateLineWidths.Schedule(connections.Count, 128).Complete();

            s_UpdateMeshVerticies.Begin();
            var updateMeshVertices = new Link.UpdateMeshVertices()
            {
                positions = positions,
                gn1PositionIndex = gn1Index,
                gn2PositionIndex = gn2Index,
                gn1VertexStartIndices = gn1VertexStartIndices,
                gn2VertexStartIndices = gn2VertexStartIndices,
                draw = drawables,
                lineWidth = lineWidths,
                vertixPositions = m_GravMesh.vertices,
                cameraPos = gravGridCam.transform.position,
                cameraUp = gravGridCam.transform.up,
                gravGridPos = transform.position
            };

            updateMeshVertices.Schedule(connections.Count, 128).Complete();
            s_UpdateMeshVerticies.End();
        }

        s_ResetAccelerations.Begin();
        var resetAccelerations = new GravNode.ResetAcclerations
        {
            accelerations = accelerations
        };
        resetAccelerations.Schedule(gravNodes.Count, 128).Complete();
        s_ResetAccelerations.End();

        s_UpdateMesh.Begin();
        m_GravMesh.UpdateMesh(GetMeshCenterWorld());
        s_UpdateMesh.End();
    }

    private void OnDestroy()
    {
        startConnectionsIndex.Dispose();
        nodeNumConnections.Dispose();
        allNodeConnections.Dispose();
        allRestsDistances.Dispose();

        gn1Index.Dispose();
        gn2Index.Dispose();
        gn1VertexStartIndices.Dispose();
        gn2VertexStartIndices.Dispose();
        lineWidths.Dispose();
        baseLineWidths.Dispose();
        coordinates.Dispose();

        drawables.Dispose();

        offsets.Dispose();
        prevPositions.Dispose();
        positions.Dispose();
        newPositions.Dispose();
        affected.Dispose();
        accelerations.Dispose();
        targetPositions.Dispose();
        returnPositions.Dispose();
        moveables.Dispose();
        m_GravMesh.vertices.Dispose();
        m_MeshMatcher.Cleanup();
        forceUpdatePosition.Dispose();
        colliderTransformArray.Dispose();
    }
}
