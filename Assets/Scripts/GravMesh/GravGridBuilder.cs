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
    static ProfilerMarker s_TransformUpdate = new ProfilerMarker("TransformUpdate");
    static ProfilerMarker s_TotalSolverTime = new ProfilerMarker("JobsSolverTime");
    static ProfilerMarker s_SingleSolverIterations = new ProfilerMarker("SingleJobIteration");
    static ProfilerMarker s_PrepareSolver = new ProfilerMarker("PrepareConstraintSolver");
    static ProfilerMarker s_SetSolverTransforms = new ProfilerMarker("SetSolverTransforms");

    static ProfilerMarker s_PrepareUpdateJobs = new ProfilerMarker("PrepareUpdateJobs");
    static ProfilerMarker s_UpdateJobs = new ProfilerMarker("UpdateJobs");


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

    private float mLeftOverTime = 0.0f;

    //NativeArray<float3> gn1Positions;
    //NativeArray<float3> gn2Positions;
    NativeArray<float> restDistances;
    NativeArray<float> stiffnesses;
    NativeArray<int> gn1Index;
    NativeArray<int> gn2Index;
    NativeArray<int> gnconnections;
    NativeArray<float3> results;

    NativeArray<float3> accumulator;

    NativeArray<float3> targetPositions;
    NativeArray<float3> prevPositions;
    NativeArray<float3> positions;
    NativeArray<float3> accelerations;
    NativeArray<bool> moveables;

    void Start()
    {
        fixedDeltaTimeSeconds = (float)fixedDeltaTime / 1000.0f;
        gravGrid = new GravNode[mHorizontalParticles, mVerticalParticles];
        gravNodes = new List<GravNode>(mHorizontalParticles * mVerticalParticles);
        int index = 0;
        for (int i = 0; i < mHorizontalParticles; ++i)
        {
            for (int j = 0; j < mVerticalParticles; ++j)
            {
                Vector3 position = new Vector3(i * spacing, j * spacing, 0) + transform.position;
                gravGrid[i,j] = new GravNode(position, spacing, index++);
                gravNodes.Add(gravGrid[i, j]);
            }
        }

        for(int x = 0; x < mHorizontalParticles; x++)
        {
            for(int y = 0; y < mVerticalParticles; y++)
            {
                if (x < mHorizontalParticles-1) AddConnection(gravGrid[x,y],gravGrid[x+1,y], true, planeLinkStiffness);
                if (y < mVerticalParticles-1) AddConnection(gravGrid[x,y],gravGrid[x,y+1], true, planeLinkStiffness);
                if (x < mHorizontalParticles-1 && y < mVerticalParticles-1) AddConnection(gravGrid[x,y],gravGrid[x+1,y+1], drawDiagonals, planeLinkStiffness);
                if (x < mHorizontalParticles-1 && y < mVerticalParticles-1) AddConnection(gravGrid[x+1,y],gravGrid[x,y+1], drawDiagonals, planeLinkStiffness);
                // configure extended connections
                if (x < mHorizontalParticles - 2) AddConnection(gravGrid[x, y], gravGrid[x + 2, y], drawDiagonals);
                if (y < mVerticalParticles - 2) AddConnection(gravGrid[x, y], gravGrid[x, y + 2], drawDiagonals);
                if (x < mHorizontalParticles - 2 && y < mVerticalParticles - 2) AddConnection(gravGrid[x, y], gravGrid[x + 2, y + 2], drawDiagonals, planeLinkStiffness);
                if (x < mHorizontalParticles - 2 && y < mVerticalParticles - 2) AddConnection(gravGrid[x + 2, y], gravGrid[x, y + 2], drawDiagonals, planeLinkStiffness);

                if (x == 0) gravGrid[x, y].m_Moveable = false;
                if (y == 0) gravGrid[x, y].m_Moveable = false;
                if (x == mHorizontalParticles - 1) gravGrid[x, y].m_Moveable = false;
                if (y == mVerticalParticles - 1) gravGrid[x, y].m_Moveable = false;
            }
        }

        restDistances = new NativeArray<float>(connections.Count, Allocator.Persistent);
        stiffnesses = new NativeArray<float>(connections.Count, Allocator.Persistent);
        gn1Index = new NativeArray<int>(connections.Count, Allocator.Persistent);
        gn2Index = new NativeArray<int>(connections.Count, Allocator.Persistent);

        results = new NativeArray<float3>(connections.Count, Allocator.Persistent);
        accumulator = new NativeArray<float3>(gravNodes.Count, Allocator.Persistent);

        for (int j = 0; j < connections.Count; j++)
        {
            stiffnesses[j] = connections[j].m_Stiffness;
            restDistances[j] = connections[j].m_RestDistance;
            gn1Index[j] = connections[j].m_GravNode1.m_Index;
            gn2Index[j] = connections[j].m_GravNode2.m_Index;
        }

        gnconnections = new NativeArray<int>(gravNodes.Count, Allocator.Persistent);

        for(int j = 0; j < gravNodes.Count; j++)
        {
            gnconnections[j] = gravNodes[j].m_Connections;
        }

        targetPositions = new NativeArray<float3>(gravNodes.Count, Allocator.Persistent);
        prevPositions = new NativeArray<float3>(gravNodes.Count, Allocator.Persistent);
        positions = new NativeArray<float3>(gravNodes.Count, Allocator.Persistent);
        accelerations = new NativeArray<float3>(gravNodes.Count, Allocator.Persistent);
        moveables = new NativeArray<bool>(gravNodes.Count, Allocator.Persistent);
    }

    public void AddConnection(GravNode gn1, GravNode gn2, bool draw, float stiffness = 1.0f)
    {
        connections.Add(new Link(gn1, gn2, draw, stiffness));
    }

    void Update()
    {
        float elapsedTime = Time.deltaTime;
        elapsedTime += mLeftOverTime;
        int timeSteps = Mathf.FloorToInt(elapsedTime / fixedDeltaTimeSeconds);
        timeSteps = Math.Min(timeSteps, 5);
        mLeftOverTime = elapsedTime - timeSteps * fixedDeltaTimeSeconds;



        for (int z = 0; z < timeSteps; z++)
        {
            s_TotalSolverTime.Begin();

            for (int i = 0; i < SolverIterations; i++)
            {
                s_PrepareSolver.Begin();
                for (int j = 0; j < gravNodes.Count; j++)
                {
                    positions[j] = gravNodes[j].m_Position;
                }
                s_PrepareSolver.End();

                var jobData = new SolveConstraintsJob()
                {
                    positions = positions,
                    gn1Indicies = gn1Index,
                    gn2Indicies = gn2Index,
                    restDistances = restDistances,
                    linkStiffness = stiffnesses,
                    results = results
                };

                JobHandle handle = jobData.Schedule(connections.Count, 1024);
                handle.Complete();

                s_SetSolverTransforms.Begin();
                for (int j = 0; j < connections.Count; j++)
                {
                    connections[j].OffsetNodeConnection(results[j]);
                }
                s_SetSolverTransforms.End();
            }
            s_TotalSolverTime.End();

            s_PrepareUpdateJobs.Begin();
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

            for (int i = 0; i < gravNodes.Count; ++i)
            {
                accelerations[i] = gravNodes[i].m_Acceleration;
                prevPositions[i] = gravNodes[i].m_PrevPosition;
                targetPositions[i] = gravNodes[i].m_TargetPosition;
                moveables[i] = gravNodes[i].m_Moveable;
            }
            s_PrepareUpdateJobs.End();

            s_UpdateJobs.Begin();
            JobHandle updateJobHandle = updateJobData.Schedule(gravNodes.Count, 128);
            updateJobHandle.Complete();
            s_UpdateJobs.End();

            s_TransformUpdate.Begin();
            for (int i = 0; i < gravNodes.Count; ++i)
                gravNodes[i].UpdateTransforms(prevPositions[i], positions[i]);
            s_TransformUpdate.End();
        }
    }

    private void OnDestroy()
    {
        //gn1Positions.Dispose();
        //gn2Positions.Dispose();
        gnconnections.Dispose();
        gn1Index.Dispose();
        gn2Index.Dispose();
        restDistances.Dispose();
        stiffnesses.Dispose();
        results.Dispose();
        accumulator.Dispose();

        prevPositions.Dispose();
        positions.Dispose();
        accelerations.Dispose();
        targetPositions.Dispose();
        moveables.Dispose();
    }
}
