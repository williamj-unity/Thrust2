using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GravGridBuilder : MonoBehaviour
{
    public int mHorizontalParticles, mVerticalParticles;

    public float spacing;

    private List<GravNodeConnection> connections = new List<GravNodeConnection>();
    private GravNode[,] gravGrid;

    public int SolverIterations = 15;

    public float dampingCoefficient = 0.5f;
    // Start is called before the first frame update
    void Start()
    {
        gravGrid = new GravNode[mHorizontalParticles, mVerticalParticles];
        for (int i = 0; i < mHorizontalParticles; ++i)
        {
            for (int j = 0; j < mVerticalParticles; ++j)
            {
                Vector3 position = new Vector3(i * spacing, j * spacing, 0) + transform.position;
                gravGrid[i,j] = new GravNode(position, spacing);
            }
        }
        
        
        for (int i = 0; i < mHorizontalParticles; ++i)
        {
            for (int j = 0; j < mVerticalParticles; ++j)
            {
                if (i < mHorizontalParticles - 1)
                {
                    LineRenderPair lnp = new GameObject("linerenderpair").AddComponent<LineRenderPair>();

                    lnp.SetupLineRenderPair(gravGrid[i,j].lineRendererTransform, gravGrid[i+1,j].lineRendererTransform, 0.05f);
                }

                if (j < mVerticalParticles - 1)
                {
                    LineRenderPair lnp = new GameObject("linerenderpair").AddComponent<LineRenderPair>();

                    lnp.SetupLineRenderPair(gravGrid[i,j].lineRendererTransform, gravGrid[i,j+1].lineRendererTransform, 0.05f);
                }
            }
        }

        
        for(int x = 0; x < mHorizontalParticles; x++)
        {
            for(int y = 0; y < mVerticalParticles; y++)
            {
                if (x < mHorizontalParticles-1) AddConnection(gravGrid[x,y],gravGrid[x+1,y]);
                if (y < mVerticalParticles-1) AddConnection(gravGrid[x,y],gravGrid[x,y+1]);
                if (x < mHorizontalParticles-1 && y < mVerticalParticles-1) AddConnection(gravGrid[x,y],gravGrid[x+1,y+1]);
                if (x < mHorizontalParticles-1 && y < mVerticalParticles-1) AddConnection(gravGrid[x+1,y],gravGrid[x,y+1]);
            }
        }

        for(int x = 0; x < mHorizontalParticles; x++)
        {
            for(int  y= 0; y < mVerticalParticles; y++)
            {
                if (x < mHorizontalParticles-2) AddConnection(gravGrid[x,y],gravGrid[x+2,y]);
                if (y < mVerticalParticles-2) AddConnection(gravGrid[x,y],gravGrid[x,y+2]);
                if (x < mHorizontalParticles-2 && y<mVerticalParticles-2) AddConnection(gravGrid[x,y],gravGrid[x+2,y+2]);
                if (x  <mHorizontalParticles-2 && y<mVerticalParticles-2) AddConnection(gravGrid[x+2,y],gravGrid[x,y+2]);			}
        }
    }

    public void AddConnection(GravNode gn1, GravNode gn2)
    {
        connections.Add(new GravNodeConnection(gn1, gn2));
    }


    void Update()
    {
        for(int i = 0; i < SolverIterations; i++)
        {
            for(int j = 0; j < connections.Count; j++)
            {
                // satisfy constraint
                connections[j].SatisfyConstraint();
            }
        }
        
        for (int i = 0; i < mHorizontalParticles; ++i)
        {
            for (int j = 0; j < mVerticalParticles; ++j)
            {
                gravGrid[i, j].Update(dampingCoefficient, Time.deltaTime);
            }
        }
    }
    
}
