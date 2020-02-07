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
    public Vector2Int startingSectorCoordinate;
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

    NativeArray<float3> prevPositions;
    NativeArray<float3> positions;
    NativeArray<float3> newPositions;

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

    // Parent Grid Properties
    int numRootCoords = 9; // TODO:: generate tris programmatically
    NativeArray<float3> parentGridPoses;
    NativeArray<float2> parentGridCoords;
    NativeArray<int> parentGridTris;

    Queue<int> availableConnectionIndex;

    GravMesh m_GravMesh;
    Camera gravGridCam;

    int MAX_CONNECTIONS = 16;
    int mStartWorldX = 0;
    int mStartWorldY = 0;
    int2 sector = 0;

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
        offsets = new NativeArray<float2>(mHorizontalParticles * mVerticalParticles, Allocator.Persistent);
        coordinates = new NativeArray<int2>(mHorizontalParticles * mVerticalParticles, Allocator.Persistent);
        sector = new int2(startingSectorCoordinate.x, startingSectorCoordinate.y);
        InitParentGridProperties();
        SetAdjacentNodes();
        for (int i = 0; i < mHorizontalParticles; ++i)
        {
            int xOffset = (i + mStartWorldX) - sector.x;
            float t = ((float)xOffset / (float)mHorizontalParticles + 1.0f) / 2.0f;

            for (int j = 0; j < mVerticalParticles; ++j)
            {

                int yOffset = (j + mStartWorldY) - sector.y;
                float s = ((float)yOffset / (float)mVerticalParticles + 1.0f) / 2.0f;

                Vector3 position = new Vector3(i * spacing, j * spacing, 0);
                int vertexStartIndex = m_GravMesh.AddNode();
                offsets[index] = new float2(t, s);
                coordinates[index] = new int2(i, j);

                var gn = new GravNode(position, spacing, index++, transform, vertexStartIndex, gameObject.layer);
                gn.AddForce = ApplyForce;
                gn.SetTargetLocation = SetTargetLocation;
                gn.Return = SetTargetToReturnPosition;
                gn.GetNodePosition = GetNodePosition;
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

    void InitParentGridProperties()
    {
        //TODO:: generate this based on "numRootCoords"
        float2[] rootPosCoords = new float2[]
        {
            new float2(0.0f, 0.0f), // Bottom left
            new float2(0.5f, 0.0f), // center bottom
            new float2(1.0f, 0.0f), // bottom right

            new float2(0.0f, 0.5f), //centerLeft
            new float2(0.5f, 0.5f), //center
            new float2(1.0f, 0.5f), //centerRight

            new float2(0.0f, 1.0f), //topLeft
            new float2(0.5f, 1.0f), //centerTop
            new float2(1.0f, 1.0f), //topRight

        };
        //TODO:: generate this based on "numRootCoords"
        int[] parentGridTriArray = new int[]
        {
            0,3,1,
            1,3,4,
            1,4,2,
            2,4,5,
            3,6,4,
            4,6,7,
            4,7,5,
            5,7,8
        };

        parentGridPoses = new NativeArray<float3>(numRootCoords, Allocator.Persistent);
        parentGridCoords = new NativeArray<float2>(rootPosCoords, Allocator.Persistent);
        parentGridTris = new NativeArray<int>(parentGridTriArray, Allocator.Persistent);
    }

    internal void SetOffsets()
    {
        for(int i = 0; i < mHorizontalParticles; i++)
        {
            for(int j = 0; j < mVerticalParticles; j++)
            {
                int x = mod((mStartWorldX + i), mHorizontalParticles);
                int y = mod((mStartWorldY + j), mVerticalParticles);
                GravNode gn = gravGrid[x, y];
                int index = gn.m_Index;
                int xOffset = (i + mStartWorldX) - sector.x;
                float t = ((float)xOffset / (float)mHorizontalParticles + 1.0f) / 2.0f;
                int yOffset = (j + mStartWorldY) - sector.y;
                float s = ((float)yOffset / (float)mVerticalParticles + 1.0f) / 2.0f;
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

    internal void ShiftPlane()
    {
        //var ShiftGridPlaneJob = new GravNode.ShiftPlane()
        //{
        //    A = topLeft,
        //    B = topRight,
        //    C = bottomLeft,
        //    D = bottomRight,
        //    sectorOffsets = offsets,
        //    affected = affected,
        //    targetPositions = targetPositions,
        //    returnPositions = returnPositions
        //};

        //ShiftGridPlaneJob.Schedule(returnPositions.Length, 64).Complete();
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
                int x = mod((mStartWorldX + i), mHorizontalParticles);
                int y = mod((mStartWorldY + j), mVerticalParticles);
                float thickcessX = lineWidth;
                float thickcessY = lineWidth;
                if (y == sector.y)
                    thickcessX *= 3;
                if (x == sector.x)
                    thickcessY *= 3;

                if (i < mHorizontalParticles - 1) AddConnection(gravGrid[x, y], gravGrid[mod((x + 1), mHorizontalParticles), y],  true, thickcessX, m_GravMesh, Link.Dir.Right);
                if(j < mVerticalParticles - 1) AddConnection(gravGrid[x, y], gravGrid[x, mod((y + 1), mVerticalParticles)], true, thickcessY, m_GravMesh, Link.Dir.Up);
                //if (x < mHorizontalParticles - 1 && y < mVerticalParticles - 1) AddConnection(gravGrid[x, y], gravGrid[x + 1, y + 1], drawDiagonals, planeLinkStiffness, m_GravMesh);
                //if (x < mHorizontalParticles - 1 && y < mVerticalParticles - 1) AddConnection(gravGrid[x + 1, y], gravGrid[x, y + 1], drawDiagonals, planeLinkStiffness, m_GravMesh);
                //configure extended connections
                //if (x < mHorizontalParticles - 2) AddConnection(gravGrid[x, y], gravGrid[x + 2, y], drawDiagonals, planeLinkStiffness, m_GravMesh);
                //if (y < mVerticalParticles - 2) AddConnection(gravGrid[x, y], gravGrid[x, y + 2], drawDiagonals, planeLinkStiffness, m_GravMesh);
                //if (x < mHorizontalParticles - 2 && y < mVerticalParticles - 2) AddConnection(gravGrid[x, y], gravGrid[x + 2, y + 2], drawDiagonals, planeLinkStiffness, m_GravMesh);
                //if (x < mHorizontalParticles - 2 && y < mVerticalParticles - 2) AddConnection(gravGrid[x + 2, y], gravGrid[x, y + 2], drawDiagonals, planeLinkStiffness, m_GravMesh);

                if (x == mStartWorldX || y == mStartWorldY || x == mStartWorldX + mHorizontalParticles - 1 || y == mStartWorldY + mVerticalParticles - 1) gravGrid[x, y].SetMoveable(false);
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

    public void AddConnection(GravNode gn1, GravNode gn2, bool draw, float thickness, GravMesh gravMesh, Link.Dir dir)
    {
        Link l = new Link(gn1, gn2, draw, gravMesh, thickness, connections.Count, dir, spacing);
        connections.Add(l);
    }

    static ProfilerMarker s_CreateNewLink = new ProfilerMarker("CreateNewLink");

   
    float3 GetNodePosition(int index)
    {
        return new float3(transform.position) + positions[index];
    }

    void OnDrawGizmosSelected()
    {
        if (!parentGridPoses.IsCreated)
            return;
        Gizmos.color = Color.red;

        for (int i = 0; i < numRootCoords; i ++)
        {
            Gizmos.DrawSphere(parentGridPoses[i], 0.1f);
        }

        Gizmos.DrawSphere(parentGridPoses[4], 0.2f);

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
        sector += shiftBy;
        SetOffsets();
        SetAdjacentNodes();
        ShiftPlane();
    }
    public void SetAdjacentNodes()
    {
        float3 center = new float3(sector.x * spacing, sector.y * spacing, 0.0f);
        float width = mHorizontalParticles * spacing;
        float height = mVerticalParticles * spacing;
        //TODO:: generate  from 'numRootPoses

        parentGridPoses[0] = QueryAdjacentNode(center + new float3(-width, -height, 0)); //bottomLeft
        parentGridPoses[1] = QueryAdjacentNode(center + new float3(0, -height, 0)); //centerBottom
        parentGridPoses[2] = QueryAdjacentNode(center + new float3(width, -height, 0)); //bottomRight

        parentGridPoses[3] = QueryAdjacentNode(center + new float3(-width, 0, 0)); // centerLeft
        parentGridPoses[4] = QueryAdjacentNode(center); // center
        parentGridPoses[5] = QueryAdjacentNode(center + new float3(width, 0, 0)); //centerRight


        parentGridPoses[6] = QueryAdjacentNode(center + new float3(-width, height, 0)); //topLeft
        parentGridPoses[7] = QueryAdjacentNode(center + new float3(0, height, 0)); //centerTop
        parentGridPoses[8] = QueryAdjacentNode(center + new float3(width, height, 0)); //topRight


    }

    public float3 Midpoint(float3 a, float3 b)
    {
        return new float3((a.x + b.x) / 2.0f, (a.y + b.y) / 2.0f, (a.z + b.z) / 2.0f);
    }

    Collider[] hits = new Collider[1];
    Vector3 QueryAdjacentNode(Vector3 queryPos)
    {
        if (parentLayer.value == 0)
            return queryPos;
        int numHits = Physics.OverlapSphereNonAlloc(queryPos, spacing / 2.0f, hits, parentLayer.value, QueryTriggerInteraction.Collide);
        if (numHits == 1)
        {
            if (hits[0].TryGetComponent<GravNodeCollider>(out var collider))
            {
                return collider.GetNodeTransform();
            }
        }
        return queryPos;
    }

    public float3 BilinearInterpolation(float t, float s, float3 A, float3 B, float3 C, float3 D)
    {
        float3 AB_interp = (1 - t) * C + t * D;
        float3 CD_interp = (1 - t) * A + t * B;

        float3 final = (1 - s) * AB_interp + s * CD_interp;
        float3 pose = final;
        return pose;
    }

    public float3 GetNodePoseFromBarycentrics(float2 offset)
    {
        float3 coords = 0;
        int goldenTri = 0;
        for (int i = 0; i < 9; i++)
        {
            float2 a = parentGridCoords[parentGridTris[i * 3]];
            float2 b = parentGridCoords[parentGridTris[i * 3 + 1]];
            float2 c = parentGridCoords[parentGridTris[i * 3 + 2]];

            coords = GravNode.ShiftPlane2ElectricBoogaloo.Barycentric(offset, a, b, c);
            if (!(coords.x < 0 || coords.y < 0 || coords.z < 0))
            {
                goldenTri = i;
                break;
            }
        }

        float3 aPose = parentGridPoses[parentGridTris[goldenTri * 3]];
        float3 bPose = parentGridPoses[parentGridTris[goldenTri * 3 + 1]];
        float3 cPose = parentGridPoses[parentGridTris[goldenTri * 3 + 2]];

        float3 pose = aPose * coords.z + bPose * coords.x + cPose * coords.y;
        return pose;
    }

    public void ShiftGrid(GridShiftDirection gridShiftDirection)
    {
        switch (gridShiftDirection)
        {
            case GridShiftDirection.Right:
                {
                    int newStart = mStartWorldX + 1;
                    int oldEnd = mod(mStartWorldX + mHorizontalParticles - 1, mHorizontalParticles);
                    int newStartX = mod(newStart, mHorizontalParticles);
                    int oldStart = mod(mStartWorldX, mHorizontalParticles);
                    int newWorldXPos = mStartWorldX + mHorizontalParticles;

                    for (int j = 0; j < mVerticalParticles; j++)
                    {
                        int y = mod((mStartWorldY + j), mVerticalParticles);

                        int yOffset = (mStartWorldY + j) - sector.y;
                        int xOffset = newWorldXPos - sector.x;
                        float t = ((float)xOffset / (float)mHorizontalParticles + 1.0f) / 2.0f;
                        float s = ((float)yOffset / (float)mVerticalParticles + 1.0f) / 2.0f;

                        Vector3 pose = GetNodePoseFromBarycentrics(new float2(t,s));
                        Vector3 colliderRootPos = pose;
                        colliderRootPos.z = 0;
                        
                        ConfigureNewNodePosition(oldStart, y, colliderRootPos, pose);

                        GravNode newStartGN = gravGrid[newStartX, y];
                        GravNode oldStartNewEndGN = gravGrid[oldStart, y];
                        GravNode oldEndGN = gravGrid[oldEnd, y];
                        offsets[oldStartNewEndGN.m_Index] = new float2(t, s);

                        ConfigureNewStartAndEndNodes(
                            oldStartNewEndGN, //break
                            newStartGN, //break
                            oldEndGN,   // connect
                            oldStartNewEndGN, //connect
                            newStartGN, //start
                            oldStartNewEndGN, //end
                            oldEndGN, //middle
                            y, mStartWorldY, mVerticalParticles, Link.Dir.Right);
                    }
                    mStartWorldX = newStart;
                    if (oldStart == startingSectorCoordinate.x) // sector is moving!!
                        SectorShift(new int2(mHorizontalParticles, 0));
                    break;
                }

            case GridShiftDirection.Up:
                {
                    int newStart = mStartWorldY + 1;
                    int oldEnd = mod(mStartWorldY + mVerticalParticles - 1, mVerticalParticles);
                    int newStartY = mod(newStart, mVerticalParticles);
                    int oldStart = mod(mStartWorldY, mVerticalParticles);
                    int newWorldYPos = mStartWorldY + mVerticalParticles;

                    for (int j = 0; j < mHorizontalParticles; j++)
                    {
                        int x = mod((mStartWorldX + j), mHorizontalParticles);

                        int yOffset = newWorldYPos - sector.y;
                        int xOffset = (j + mStartWorldX) - sector.x;

                        float t = ((float)xOffset / (float)mHorizontalParticles + 1.0f) / 2.0f;
                        float s = ((float)yOffset / (float)mVerticalParticles + 1.0f) / 2.0f;

                        Vector3 pose = GetNodePoseFromBarycentrics(new float2(t, s));
                        Vector3 colliderRootPos = pose;
                        colliderRootPos.z = 0;

                        ConfigureNewNodePosition(x, oldStart, colliderRootPos, pose);

                        GravNode newStartGN = gravGrid[x, newStartY];
                        GravNode oldStartNewEndGN = gravGrid[x, oldStart];
                        GravNode oldEndGN = gravGrid[x, oldEnd];
                        offsets[oldStartNewEndGN.m_Index] = new float2(t, s);

                        ConfigureNewStartAndEndNodes(
                            oldStartNewEndGN, //break
                            newStartGN, //break
                            oldEndGN,   // connect
                            oldStartNewEndGN, //connect
                            newStartGN, //start
                            oldStartNewEndGN, //end
                            oldEndGN, //middle
                            x, mStartWorldX, mHorizontalParticles, Link.Dir.Up);
                    }
                    mStartWorldY = newStart;
                    if (oldStart == startingSectorCoordinate.y) // sector is moving!!
                        SectorShift(new int2(0, mVerticalParticles));

                    break;
                }
            case GridShiftDirection.Left:
                {
                    int newStart = mStartWorldX - 1;
                    int oldStart = mod(mStartWorldX, mHorizontalParticles);
                    int oldEnd = mod(mStartWorldX + mHorizontalParticles - 1, mHorizontalParticles);
                    int newEnd = mod(mStartWorldX + mHorizontalParticles - 2, mHorizontalParticles);

                    for (int j = 0; j < mVerticalParticles; j++)
                    {
                        int y = mod((mStartWorldY + j), mVerticalParticles);

                        int yOffset = (j + mStartWorldY) - sector.y;
                        int xOffset = newStart - sector.x;
                        float t = ((float)xOffset / (float)mHorizontalParticles + 1.0f) / 2.0f;
                        float s = ((float)yOffset / (float)mVerticalParticles + 1.0f) / 2.0f;

                        Vector3 pose = GetNodePoseFromBarycentrics(new float2(t, s));
                        Vector3 colliderRootPos = pose;
                        colliderRootPos.z = 0;

                        ConfigureNewNodePosition(oldEnd, y, colliderRootPos, pose);

                        GravNode oldStartGN = gravGrid[oldStart, y];
                        GravNode newEndGN = gravGrid[newEnd, y];
                        GravNode oldEndNewStartGN = gravGrid[oldEnd, y];
                        offsets[oldEndNewStartGN.m_Index] = new float2(t, s);

                        ConfigureNewStartAndEndNodes(
                            newEndGN, //break
                            oldEndNewStartGN, //break
                            oldEndNewStartGN,   // connect
                            oldStartGN,  //connect
                            oldEndNewStartGN, //start
                            newEndGN, //end
                            oldStartGN, //middle
                            y, mStartWorldY, mVerticalParticles, Link.Dir.Right);
                    }
                    mStartWorldX = newStart;
                    if (oldEnd == startingSectorCoordinate.x) // sector is moving!!
                        SectorShift(new int2(-mHorizontalParticles, 0));
                    break;
                }
            case GridShiftDirection.Down:
                {
                    int newStart = mStartWorldY - 1;
                    int oldStart = mod(mStartWorldY, mVerticalParticles);
                    int oldEnd = mod(mStartWorldY + mVerticalParticles - 1, mVerticalParticles);
                    int newEnd = mod(mStartWorldY + mHorizontalParticles - 2, mHorizontalParticles);

                    for (int j = 0; j < mHorizontalParticles; j++)
                    {
                        int x = mod((mStartWorldX + j), mHorizontalParticles);

                        int yOffset = newStart - sector.y;
                        int xOffset = (j + mStartWorldX) - sector.x;
                        float t = ((float)xOffset / (float)mHorizontalParticles + 1.0f) / 2.0f;
                        float s = ((float)yOffset / (float)mVerticalParticles + 1.0f) / 2.0f;

                        Vector3 pose = GetNodePoseFromBarycentrics(new float2(t, s));
                        Vector3 colliderRootPos = pose;
                        colliderRootPos.z = 0;

                        ConfigureNewNodePosition(x, oldEnd, colliderRootPos, pose);

                        GravNode oldStartGN = gravGrid[x, oldStart];
                        GravNode newEndGN = gravGrid[x, newEnd];
                        GravNode oldEndNewStartGN = gravGrid[x, oldEnd];
                        offsets[oldEndNewStartGN.m_Index] = new float2(t, s);

                        ConfigureNewStartAndEndNodes(
                            newEndGN, //break
                            oldEndNewStartGN, //break
                            oldEndNewStartGN,   // connect
                            oldStartGN,  //connect
                            oldEndNewStartGN, //start
                            newEndGN, //end
                            oldStartGN, //middle
                            x, mStartWorldX, mHorizontalParticles, Link.Dir.Up);
                    }
                    mStartWorldY = newStart;
                    if (oldEnd == startingSectorCoordinate.y) // sector is moving!!
                        SectorShift(new int2(0, -mVerticalParticles));
                    break;
                }
            default:
                break;
        }
    }

    public void ConfigureNewNodePosition(int x, int y, Vector3 rootPos, Vector3 relativePos)
    {
        GravNode gn = gravGrid[x, y];
        gn.SetHomePosition(relativePos);
        gn.SetRootPos(rootPos);
        int index = gn.m_Index;
        accelerations[index] = 0;
        prevPositions[index] = relativePos;
        targetPositions[index] = relativePos;
        returnPositions[index] = relativePos;
        positions[index] = relativePos;
        gn.SetMoveable(false);
    }

    public void ConfigureNewStartAndEndNodes(
        GravNode break1, 
        GravNode break2, 
        GravNode connect1, 
        GravNode connect2,
        GravNode newStart,
        GravNode newEnd,
        GravNode newMid,
        int row, int start, int max, Link.Dir dir)
    {
        s_AddConnections.Begin();
        RemoveConnection(break1, break2);
        AddAndSetupConnectionAtNextIndex(connect1, connect2, true, m_GravMesh, dir);
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

    public void AddAndSetupConnectionAtNextIndex(GravNode gn1, GravNode gn2, bool draw, GravMesh gravMesh, Link.Dir dir)
    {
        if (availableConnectionIndex.Count == 0)
            Debug.LogError("There are no available connections.");
        s_CreateNewLink.Begin();
        int index = availableConnectionIndex.Dequeue();
        //We're not going to update the thickness data as to preserve the sector location.
        connections[index].MakeLink(gn1, gn2, draw, gravMesh, 0, dir, spacing);
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
        int x = mod(mStartWorldX + mHorizontalParticles / 2, mHorizontalParticles);
        int y = mod(mStartWorldY + mVerticalParticles / 2, mVerticalParticles);

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

        parentGridPoses.Dispose();
        parentGridCoords.Dispose();
        parentGridTris.Dispose();

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
    }
}
