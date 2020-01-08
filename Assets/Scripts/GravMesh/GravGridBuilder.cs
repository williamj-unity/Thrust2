using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using Unity.Collections;
using Unity.Jobs;
using System;
using Unity.Mathematics;

public class GravGridBuilder : MonoBehaviour
{
    private List<Link> connections = new List<Link>();
    private GravNode[,] gravGrid;
    public int SolverIterations = 15;

    public float dampingCoefficient = 0.5f;

    public float fixedDeltaTime = 16.0f;
    private float fixedDeltaTimeSeconds;


    [Header("Build time settings")]
    public int mHorizontalParticles;
    public int mVerticalParticles;
    public float spacing;
    public bool drawDiagonals = false;
    public bool drawAnchorConnections = false;
    public float anchorStiffness = 5.0f;
    public float planeLinkStiffness = 1.0f;

    private float mLeftOverTime = 0.0f;

    NativeArray<float3> gn1Positions;
    NativeArray<float3> gn2Positions;
    NativeArray<float> restDistances;
    NativeArray<float> stiffnesses;
    NativeArray<float3> results;

    void Start()
    {
        fixedDeltaTimeSeconds = (float)fixedDeltaTime / 1000.0f;
        gravGrid = new GravNode[mHorizontalParticles, mVerticalParticles];
        for (int i = 0; i < mHorizontalParticles; ++i)
        {
            for (int j = 0; j < mVerticalParticles; ++j)
            {
                Vector3 position = new Vector3(i * spacing, j * spacing, 0) + transform.position;
                gravGrid[i,j] = new GravNode(position, spacing);
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

        gn1Positions = new NativeArray<float3>(connections.Count, Allocator.Persistent);
        gn2Positions = new NativeArray<float3>(connections.Count, Allocator.Persistent);
        restDistances = new NativeArray<float>(connections.Count, Allocator.Persistent);
        stiffnesses = new NativeArray<float>(connections.Count, Allocator.Persistent);
        results = new NativeArray<float3>(connections.Count, Allocator.Persistent);
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
        
        for(int z = 0; z < timeSteps; z++)
        {
            for (int i = 0; i < SolverIterations; i++)
            {
                var jobData = new SolveConstraintsJob()
                {
                    gn1Postions = gn1Positions,
                    gn2Postions = gn2Positions,
                    restDistances = restDistances,
                    linkStiffness = stiffnesses,
                    results = results
                };

                for (int j = 0; j < connections.Count; j++)
                {
                    gn1Positions[j] = connections[j].m_GravNode1.m_Position;
                    gn2Positions[j] = connections[j].m_GravNode2.m_Position;
                    stiffnesses[j] = connections[j].m_Stiffness;
                    restDistances[j] = connections[j].m_RestDistance;
                }

                JobHandle handle = jobData.Schedule(connections.Count, 128);
                handle.Complete();

                for (int j = 0; j < connections.Count; j++)
                {
                    connections[j].OffsetNodeConnection(results[j]);
                }
            }
            for (int i = 0; i < mHorizontalParticles; ++i)
            {
                for (int j = 0; j < mVerticalParticles; ++j)
                {
                    //TODO::jobify update pass.
                    gravGrid[i, j].Update(dampingCoefficient, fixedDeltaTimeSeconds);
                }
            }
        }
    }

    private void OnDestroy()
    {
        gn1Positions.Dispose();
        gn2Positions.Dispose();
        restDistances.Dispose();
        stiffnesses.Dispose();
        results.Dispose();
    }
}
