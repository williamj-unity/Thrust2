using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using System;
using Unity.Mathematics;
using Unity.Profiling;
public class GravGridBuilder : MonoBehaviour
{
    static ProfilerMarker s_UpdateGraNodePoses = new ProfilerMarker("UpdateGravNodePoses");
    static ProfilerMarker s_UpdateMeshVerticies = new ProfilerMarker("UpdateMeshVertices");

    static ProfilerMarker s_TotalSolverTime = new ProfilerMarker("JobsSolverTime");


    static ProfilerMarker s_PrepareSolver = new ProfilerMarker("PrepareConstraintSolver");
    static ProfilerMarker s_PreparePhsycisStep = new ProfilerMarker("PreparePhysicsStep");
    static ProfilerMarker s_ResetAccelerations = new ProfilerMarker("ResetAccelertions");

    static ProfilerMarker s_SetSolverTransforms = new ProfilerMarker("SetSolverTransforms");
    static ProfilerMarker s_Accumlate = new ProfilerMarker("Accumulate");

    static ProfilerMarker s_PrepareUpdateJobs = new ProfilerMarker("PrepareUpdateJobs");
    static ProfilerMarker s_UpdateJobs = new ProfilerMarker("UpdateJobs");

    static ProfilerMarker s_ResetConnections = new ProfilerMarker("ResetConnections");
    static ProfilerMarker s_AddConnections = new ProfilerMarker("AddConnections");
    static ProfilerMarker s_SetConnectionProperties = new ProfilerMarker("SetConnectionProperties");
    static ProfilerMarker s_SetGravNodeConnectionProperties = new ProfilerMarker("SetGravNodeConnectionProperties");
    static ProfilerMarker s_ConstructuMesh = new ProfilerMarker("ConstructMesh");
    static ProfilerMarker s_UpdateMesh = new ProfilerMarker("UpdateMesh");

    private List<Link> connections = new List<Link>();
    private GravNode[,] gravGrid;
    private List<GravNode> gravNodes;
    public int SolverIterations = 15;

    public float dampingCoefficient = 2.0f;

    public float fixedDeltaTime = 16.0f;
    private float fixedDeltaTimeSeconds;


    [Header("Build time settings")]
    public int mHorizontalParticles;
    public int mVerticalParticles;
    public float spacing;
    public bool drawDiagonals = false;
    public float planeLinkStiffness = 1.0f;
    public float lineWidth = 0.1f;

    private float mLeftOverTime = 0.0f;

    NativeArray<int> startConnectionsIndex;
    NativeArray<int> nodeNumConnections;
    NativeArray<int> allNodeConnections;
    NativeArray<float> allRestsDistances;

    NativeArray<float3> targetPositions;
    NativeArray<float3> prevPositions;
    NativeArray<float3> positions;
    NativeArray<float3> newPositions;

    NativeArray<float3> accelerations;
    NativeArray<bool> moveables;

    NativeArray<int> gn1Index;
    NativeArray<int> gn2Index;
    NativeArray<bool> drawables;
    NativeArray<int> gn1VertexStartIndices;
    NativeArray<int> gn2VertexStartIndices;
    NativeArray<float> lineWidths;
    NativeArray<float> baseLineWidths;

    Queue<int> availableConnectionIndex;

    GravMesh m_GravMesh;
    Camera gravGridCam;

    int MAX_CONNECTIONS = 16;
    int mStartX = 0;
    int mStartY = 0;

    bool needsConnectionReset = false;

    private float lineWidthScalar = 1.0f;

    void Start()
    {
        gravGridCam = Camera.main;
        fixedDeltaTimeSeconds = (float)fixedDeltaTime / 1000.0f;
        gravGrid = new GravNode[mHorizontalParticles, mVerticalParticles];
        gravNodes = new List<GravNode>(mHorizontalParticles * mVerticalParticles);
        int index = 0;
        float meshShaderRadius = mHorizontalParticles * spacing;
        m_GravMesh = new GravMesh(meshShaderRadius, GetComponent<MeshFilter>());

        availableConnectionIndex = new Queue<int>();

        for (int i = 0; i < mHorizontalParticles; ++i)
        {
            for (int j = 0; j < mVerticalParticles; ++j)
            {
                Vector3 position = new Vector3(i * spacing, j * spacing, 0);
                int vertexStartIndex = m_GravMesh.AddNode();
                var gn = new GravNode(position, spacing, index++, transform, vertexStartIndex, gameObject.layer);
                gn.AddForce = ApplyForce;
                gn.SetTargetLocation = SetTargetLocation;
                gravGrid[i, j] = gn;
                gravNodes.Add(gn);
            }
        }

        nodeNumConnections = new NativeArray<int>(gravNodes.Count, Allocator.Persistent);
        startConnectionsIndex = new NativeArray<int>(gravNodes.Count, Allocator.Persistent);
        allNodeConnections = new NativeArray<int>(gravNodes.Count * MAX_CONNECTIONS, Allocator.Persistent);
        allRestsDistances = new NativeArray<float>(gravNodes.Count * MAX_CONNECTIONS, Allocator.Persistent);

        targetPositions = new NativeArray<float3>(gravNodes.Count, Allocator.Persistent);
        prevPositions = new NativeArray<float3>(gravNodes.Count, Allocator.Persistent);
        positions = new NativeArray<float3>(gravNodes.Count, Allocator.Persistent);
        newPositions = new NativeArray<float3>(gravNodes.Count, Allocator.Persistent);
        accelerations = new NativeArray<float3>(gravNodes.Count, Allocator.Persistent);
        moveables = new NativeArray<bool>(gravNodes.Count, Allocator.Persistent);

        for (int j = 0; j < gravNodes.Count; j++)
        {
            accelerations[j] = gravNodes[j].m_Acceleration;
            prevPositions[j] = gravNodes[j].m_PrevPosition;
            targetPositions[j] = gravNodes[j].m_TargetPosition;
            moveables[j] = gravNodes[j].m_Moveable;
            positions[j] = gravNodes[j].m_Position;
        }

        ResetConnections();
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
        for (int i = 0; i < mHorizontalParticles; i++)
        {
            for (int j = 0; j < mVerticalParticles; j++)
            {
                int x = mod((mStartX + i), mHorizontalParticles);
                int y = mod((mStartY + j), mVerticalParticles);
                float thickcessX = lineWidth;
                float thickcessY = lineWidth;
                if (y == mStartY)
                    thickcessX *= 3;
                if (x == mStartX)
                    thickcessY *= 3;

                if (i < mHorizontalParticles - 1) AddConnection(gravGrid[x, y], gravGrid[mod((x + 1), mHorizontalParticles), y],  true, thickcessX, m_GravMesh);
                if(j < mVerticalParticles - 1) AddConnection(gravGrid[x, y], gravGrid[x, mod((y + 1), mVerticalParticles)], true, thickcessY, m_GravMesh);
                //if (x < mHorizontalParticles - 1 && y < mVerticalParticles - 1) AddConnection(gravGrid[x, y], gravGrid[x + 1, y + 1], drawDiagonals, planeLinkStiffness, m_GravMesh);
                //if (x < mHorizontalParticles - 1 && y < mVerticalParticles - 1) AddConnection(gravGrid[x + 1, y], gravGrid[x, y + 1], drawDiagonals, planeLinkStiffness, m_GravMesh);
                //configure extended connections
                //if (x < mHorizontalParticles - 2) AddConnection(gravGrid[x, y], gravGrid[x + 2, y], drawDiagonals, planeLinkStiffness, m_GravMesh);
                //if (y < mVerticalParticles - 2) AddConnection(gravGrid[x, y], gravGrid[x, y + 2], drawDiagonals, planeLinkStiffness, m_GravMesh);
                //if (x < mHorizontalParticles - 2 && y < mVerticalParticles - 2) AddConnection(gravGrid[x, y], gravGrid[x + 2, y + 2], drawDiagonals, planeLinkStiffness, m_GravMesh);
                //if (x < mHorizontalParticles - 2 && y < mVerticalParticles - 2) AddConnection(gravGrid[x + 2, y], gravGrid[x, y + 2], drawDiagonals, planeLinkStiffness, m_GravMesh);

                if (x == mStartX) gravGrid[x, y].m_Moveable = false;
                else if (y == mStartY) gravGrid[x, y].m_Moveable = false;
                else if (x == mStartX + mHorizontalParticles - 1) gravGrid[x, y].m_Moveable = false;
                else if (y == mStartY + mVerticalParticles - 1) gravGrid[x, y].m_Moveable = false;
                else gravGrid[x, y].m_Moveable = true;
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

        needsConnectionReset = false;
    }

    public void AddConnection(GravNode gn1, GravNode gn2, bool draw, float thickness,  GravMesh gravMesh)
    {
        Link l = new Link(gn1, gn2, draw, gravMesh, thickness, connections.Count);
        connections.Add(l);
    }

    static ProfilerMarker s_CreateNewLink = new ProfilerMarker("CreateNewLink");


    void SetTargetLocation(float3 target, int index)
    {
        targetPositions[index] = target;
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

    public void ShiftGrid(GridShiftDirection gridShiftDirection, int steps)
    {
        if (steps == 0)
            return;
        switch (gridShiftDirection)
        {
            case GridShiftDirection.Right:
                {
                    if(steps > mHorizontalParticles)
                    {
                        steps = mHorizontalParticles - 1;
                    }
                    int newStart = mStartX + steps;
                    int currentEndIndex = mod(mStartX + mHorizontalParticles - 1, mHorizontalParticles);
                    for (int j = 0; j < mVerticalParticles; j++)
                    {
                        int y = mod((mStartY + j), mVerticalParticles);

                        int currX = 0;
                        int startStep = mod(mStartX, mHorizontalParticles);
                        s_SetConnectionProperties.Begin();
                        for (int i = 0; i < steps; i++)
                        {
                            currX = mod(i + mStartX, mHorizontalParticles);
                            Vector3 newPos = new Vector3(((i + mStartX) + mHorizontalParticles) * spacing, (mStartY + j) * spacing, 0.0f);
                            gravGrid[currX, y].SetHomePosition(newPos);
                            int index = gravGrid[currX, y].m_Index;
                            accelerations[index] = newPos;
                            prevPositions[index] = newPos;
                            targetPositions[index] = newPos;
                            positions[index] = newPos;
                            if (y != mStartY && y != mStartY + mVerticalParticles - 1)
                                moveables[index] = true;
                        }
                        s_SetConnectionProperties.End();

                        GravNode newStartGN = gravGrid[mod(newStart, mHorizontalParticles), y];
                        GravNode oldStartGN = gravGrid[startStep, y];
                        GravNode newEndGN = gravGrid[currX, y];
                        GravNode oldEndGN = gravGrid[currentEndIndex, y];

                        s_AddConnections.Begin();
                        RemoveConnection(newEndGN, newStartGN);
                        AddAndSetupConnectionAtNextIndex(oldEndGN, oldStartGN, true,  m_GravMesh);
                        s_AddConnections.End();

                        s_SetGravNodeConnectionProperties.Begin();
                        if (y != mStartY && y != mStartY + mVerticalParticles - 1)
                        {
                            int oldEndGNIndex = oldEndGN.m_Index;
                            int oldStartGNIndex = oldStartGN.m_Index;
                            moveables[oldEndGNIndex] = true;
                            moveables[oldStartGNIndex] = true;
                        }

                        int newStartXIndex = newStartGN.m_Index;
                        prevPositions[newStartXIndex] = newStartGN.m_StartPosition;
                        targetPositions[newStartXIndex] = newStartGN.m_StartPosition;
                        positions[newStartXIndex] = newStartGN.m_StartPosition;
                        moveables[newStartXIndex] = false;

                        int newEndXINdex = newEndGN.m_Index;
                        prevPositions[newEndXINdex] = newEndGN.m_StartPosition;
                        targetPositions[newEndXINdex] = newEndGN.m_StartPosition;
                        positions[newEndXINdex] = newEndGN.m_StartPosition;
                        moveables[newEndXINdex] = false;
                        s_SetGravNodeConnectionProperties.End();
                    }
                    mStartX = newStart;
                    break;
                }
            case GridShiftDirection.Left:
                {
                    if (steps > mHorizontalParticles)
                    {
                        steps = mHorizontalParticles - 1;
                    }
                    int newStart = mStartX - steps;
                    int currentEndIndex = mod(mStartX + mHorizontalParticles - 1, mHorizontalParticles);
                    int newEnd = mod(mStartX - steps + mHorizontalParticles - 1, mHorizontalParticles);
                    for (int j = 0; j < mVerticalParticles; j++)
                    {
                        int y = mod((mStartY + j), mVerticalParticles);

                        int startStep = mod(mStartX, mHorizontalParticles);
                        int currX = 0;
                        for (int i = 1; i <= steps; i++)
                        {
                            currX = mod((mStartX - i), mHorizontalParticles);
                            Vector3 newPos = new Vector3((mStartX - i) * spacing, (mStartY + j) * spacing, 0.0f);
                            gravGrid[currX, y].SetHomePosition(newPos);
                            int index = gravGrid[currX, y].m_Index;
                            accelerations[index] = newPos;
                            prevPositions[index] = newPos;
                            targetPositions[index] = newPos;
                            positions[index] = newPos;
                            if (y != mStartY && y != mStartY + mVerticalParticles - 1)
                                moveables[index] = true;
                        }
                        GravNode newStartGN = gravGrid[currX, y];
                        GravNode oldStartGN = gravGrid[startStep, y];
                        GravNode newEndGN = gravGrid[newEnd, y];
                        GravNode oldEndGN = gravGrid[currentEndIndex, y];

                        s_AddConnections.Begin();
                        RemoveConnection(newEndGN, newStartGN);
                        AddAndSetupConnectionAtNextIndex(oldEndGN, oldStartGN, true,  m_GravMesh);
                        s_AddConnections.End();

                        s_SetGravNodeConnectionProperties.Begin();
                        if (y != mStartY && y != mStartY + mVerticalParticles - 1)
                        {
                            int oldEndGNIndex = oldEndGN.m_Index;
                            int oldStartGNIndex = oldStartGN.m_Index;
                            moveables[oldEndGNIndex] = true;
                            moveables[oldStartGNIndex] = true;
                        }

                        int newStartXIndex = newStartGN.m_Index;
                        prevPositions[newStartXIndex] = newStartGN.m_StartPosition;
                        targetPositions[newStartXIndex] = newStartGN.m_StartPosition;
                        positions[newStartXIndex] = newStartGN.m_StartPosition;
                        moveables[newStartXIndex] = false;

                        int newEndXINdex = newEndGN.m_Index;
                        prevPositions[newEndXINdex] = newEndGN.m_StartPosition;
                        targetPositions[newEndXINdex] = newEndGN.m_StartPosition;
                        positions[newEndXINdex] = newEndGN.m_StartPosition;
                        moveables[newEndXINdex] = false;
                        s_SetGravNodeConnectionProperties.End();
                    }
                    mStartX -= steps;
                    break;
                }
            case GridShiftDirection.Up:
                {
                    if (steps > mVerticalParticles)
                    {
                        steps = mVerticalParticles - 1;
                    }
                    int currentEndIndex = mod(mStartY + mVerticalParticles - 1, mVerticalParticles);
                    int newStart = mStartY + steps;
                    for (int j = 0; j < mHorizontalParticles; j++)
                    {
                        int x = mod((mStartX + j), mHorizontalParticles);
                        int startStep = mod(mStartY, mVerticalParticles);
                        int currY = 0;
                        for (int i = 0; i < steps; i++)
                        {
                            currY = mod(i + mStartY, mVerticalParticles);
                            Vector3 newPos = new Vector3( (mStartX + j) * spacing, ((i + mStartY) + mVerticalParticles) * spacing, 0.0f);
                            gravGrid[x, currY].SetHomePosition(newPos);
                            int index = gravGrid[x, currY].m_Index;
                            accelerations[index] = newPos;
                            prevPositions[index] = newPos;
                            targetPositions[index] = newPos;
                            positions[index] = newPos;
                            if (x != mStartY && x != mStartX + mHorizontalParticles - 1)
                                moveables[index] = true;
                        }

                        GravNode newStartGN = gravGrid[x, mod(newStart, mHorizontalParticles)];
                        GravNode oldStartGN = gravGrid[x, startStep];
                        GravNode newEndGN = gravGrid[x, currY];
                        GravNode oldEndGN = gravGrid[x, currentEndIndex];

                        s_AddConnections.Begin();
                        RemoveConnection(newEndGN, newStartGN);
                        AddAndSetupConnectionAtNextIndex(oldEndGN, oldStartGN, true,  m_GravMesh);
                        s_AddConnections.End();

                        s_SetGravNodeConnectionProperties.Begin();
                        if (x != mStartX && x != mStartX + mHorizontalParticles - 1)
                        {
                            int oldEndGNIndex = oldEndGN.m_Index;
                            int oldStartGNIndex = oldStartGN.m_Index;
                            moveables[oldEndGNIndex] = true;
                            moveables[oldStartGNIndex] = true;
                        }

                        int newStartXIndex = newStartGN.m_Index;
                        prevPositions[newStartXIndex] = newStartGN.m_StartPosition;
                        targetPositions[newStartXIndex] = newStartGN.m_StartPosition;
                        positions[newStartXIndex] = newStartGN.m_StartPosition;
                        moveables[newStartXIndex] = false;

                        int newEndXINdex = newEndGN.m_Index;
                        prevPositions[newEndXINdex] = newEndGN.m_StartPosition;
                        targetPositions[newEndXINdex] = newEndGN.m_StartPosition;
                        positions[newEndXINdex] = newEndGN.m_StartPosition;
                        moveables[newEndXINdex] = false;
                        s_SetGravNodeConnectionProperties.End();
                    }
                    mStartY = newStart;
                    break;
                }
            case GridShiftDirection.Down:
                {
                    if (steps > mVerticalParticles)
                    {
                        steps = mVerticalParticles - 1;
                    }
                    int currentEndIndex = mod(mStartY + mVerticalParticles - 1, mVerticalParticles);
                    int newEnd = mod(mStartY - steps - 1, mVerticalParticles);
                    for (int j = 0; j < mHorizontalParticles; j++)
                    {
                        int x = mod((mStartX + j), mHorizontalParticles);
                        int currY = 0;
                        int startStep = mod(mStartY, mVerticalParticles);

                        for (int i = 1; i <= steps; i++)
                        {
                            currY = mod((mStartY - i), mVerticalParticles);
                            Vector3 newPos = new Vector3((mStartX + j) * spacing, (mStartY - i) * spacing,  0.0f);
                            gravGrid[x, currY].SetHomePosition(newPos);
                            int index = gravGrid[x, currY].m_Index;
                            accelerations[index] = newPos;
                            prevPositions[index] = newPos;
                            targetPositions[index] = newPos;
                            positions[index] = newPos;
                            if (x != mStartX && x != mStartX + mHorizontalParticles - 1)
                                moveables[index] = true;
                        }
                        GravNode newStartGN = gravGrid[x, currY];
                        GravNode oldStartGN = gravGrid[x,startStep];
                        GravNode newEndGN = gravGrid[x, newEnd];
                        GravNode oldEndGN = gravGrid[x, currentEndIndex];

                        s_AddConnections.Begin();
                        RemoveConnection(newEndGN, newStartGN);
                        AddAndSetupConnectionAtNextIndex(oldEndGN, oldStartGN, true,  m_GravMesh);
                        s_AddConnections.End();

                        s_SetGravNodeConnectionProperties.Begin();
                        if (x != mStartX && x != mStartX + mHorizontalParticles - 1)
                        {
                            int oldEndGNIndex = oldEndGN.m_Index;
                            int oldStartGNIndex = oldStartGN.m_Index;
                            moveables[oldEndGNIndex] = true;
                            moveables[oldStartGNIndex] = true;
                        }

                        int newStartXIndex = newStartGN.m_Index;
                        prevPositions[newStartXIndex] = newStartGN.m_StartPosition;
                        targetPositions[newStartXIndex] = newStartGN.m_StartPosition;
                        positions[newStartXIndex] = newStartGN.m_StartPosition;
                        moveables[newStartXIndex] = false;

                        int newEndXINdex = newEndGN.m_Index;
                        prevPositions[newEndXINdex] = newEndGN.m_StartPosition;
                        targetPositions[newEndXINdex] = newEndGN.m_StartPosition;
                        positions[newEndXINdex] = newEndGN.m_StartPosition;
                        moveables[newEndXINdex] = false;
                        s_SetGravNodeConnectionProperties.End();
                    }
                    mStartY -= steps;
                    break;
                }
            default:
                break;
        }

        needsConnectionReset = true;
    }

    static ProfilerMarker s_RemoveLinkCreateIP = new ProfilerMarker("RemoveLinkeCreateIP");

    public void AddAndSetupConnectionAtNextIndex(GravNode gn1, GravNode gn2, bool draw, GravMesh gravMesh)
    {
        if (availableConnectionIndex.Count == 0)
            Debug.LogError("There are no available slots for a connection is already full");
        s_CreateNewLink.Begin();
        int index = availableConnectionIndex.Dequeue();
        //We're not going to update the thickness data as to preserve the sector location.
        Link l = new Link(gn1, gn2, draw, gravMesh, 0, index);
        connections[index] = l;
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
        connections[indexResult] = null;
        availableConnectionIndex.Enqueue(indexResult);
    }

    public Vector3 GetMeshCenterWorld()
    {
        Vector3 localMeshMiddle = new Vector3(mStartX * spacing +  (mHorizontalParticles * spacing) / 2.0f, mStartY * spacing + (mVerticalParticles * spacing) / 2.0f, 0.0f);
        return transform.position + localMeshMiddle;
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

                handle = swapJobData.Schedule(gravNodes.Count, 64);
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
                targetPositions = targetPositions,
                moveable = moveables
            };
            JobHandle updateJobHandle = updateJobData.Schedule(gravNodes.Count, 128);
            updateJobHandle.Complete();
            s_UpdateJobs.End();

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

        drawables.Dispose();

        prevPositions.Dispose();
        positions.Dispose();
        newPositions.Dispose();
        accelerations.Dispose();
        targetPositions.Dispose();
        moveables.Dispose();
        m_GravMesh.vertices.Dispose();
       
    }
}
