using System;
using System.Collections;
using System.Collections.Generic;
using Unity.Collections;
using Unity.Mathematics;
using UnityEngine;

public class GravMesh
{
    float lineWidth = 0.1f;
    MeshFilter filter;
    Mesh mesh;

    List<float3> newVertices;
    List<Vector2> newUV;
    List<int> newTriangles;
    List<Vector3> newNormals;
    int index;

    Queue<int> availableTriIndex;

    public NativeArray<float3> vertices;
    public int[] triangles;

    bool requiresFullRebuild = false;

    public GravMesh(float lineWidth, MeshFilter meshFilter)
    {
        mesh = new Mesh();
        filter = meshFilter;
        filter.mesh = mesh;
        index = 0;
        newVertices = new List<float3>();
        newTriangles = new List<int>();
        newNormals = new List<Vector3>();
        newUV = new List<Vector2>();
        mesh.indexFormat = UnityEngine.Rendering.IndexFormat.UInt32;
        availableTriIndex = new Queue<int>();
    }

    //each node will have 8 verticies it can use to form a line segment with up to 4 other nodes
    int vertIndex = 0;
    public int AddNode()
    {
        newVertices.Add(0);
        newVertices.Add(0);
        newNormals.Add(-Vector3.forward);
        newNormals.Add(-Vector3.forward);
        newUV.Add(Vector2.zero);
        newUV.Add(Vector2.zero);

        newVertices.Add(0);
        newVertices.Add(0);
        newNormals.Add(-Vector3.forward);
        newNormals.Add(-Vector3.forward);
        newUV.Add(Vector2.zero);
        newUV.Add(Vector2.zero);

        newVertices.Add(0);
        newVertices.Add(0);
        newNormals.Add(-Vector3.forward);
        newNormals.Add(-Vector3.forward);
        newUV.Add(Vector2.zero);
        newUV.Add(Vector2.zero);

        newVertices.Add(0);
        newVertices.Add(0);
        newNormals.Add(-Vector3.forward);
        newNormals.Add(-Vector3.forward);
        newUV.Add(Vector2.zero);
        newUV.Add(Vector2.zero);

        int ret = vertIndex;
        vertIndex += 8;
        return ret;
    }

    internal void RemovePair(int startIndex)
    {
        availableTriIndex.Enqueue(startIndex);
    }

    public int AddPair(int gn1, int gn2)
    {
        int ret = 0;
        if (availableTriIndex.Count != 0)
        {
            ret = availableTriIndex.Dequeue();
            triangles[ret] = gn1;
            triangles[ret + 1] = gn1 + 1;
            triangles[ret + 2] = gn2 + 1;
            triangles[ret + 3] = gn1;
            triangles[ret + 4] = gn2 + 1;
            triangles[ret + 5] = gn2;
        }
        else
        {
            requiresFullRebuild = true;
            ret = newTriangles.Count;
            newTriangles.Add(gn1);
            newTriangles.Add(gn1 + 1);
            newTriangles.Add(gn2 + 1);
            newTriangles.Add(gn1);
            newTriangles.Add(gn2 + 1);
            newTriangles.Add(gn2);
        }

        newUV[gn1] = new Vector2(0, 0);
        newUV[gn2] = new Vector2(1, 0);
        newUV[gn1 + 1] = new Vector2(0, 1);
        newUV[gn2 + 1] = new Vector2(1, 1);

        return ret;
    }

    public void UpdateMesh()
    {
        mesh.SetVertices(vertices);
        mesh.SetTriangles(triangles, 0, true);
        mesh.SetUVs(0, newUV);
    }

    public void ConstructMesh()
    {
        if(!vertices.IsCreated)
            vertices = new NativeArray<float3>(newVertices.ToArray(), Allocator.Persistent);


        triangles = newTriangles.ToArray();
        var normalsArray = newNormals.ToArray();
        var newUVArray = newUV.ToArray();

        mesh.SetVertices(vertices);
        mesh.uv = newUVArray;
        mesh.triangles = triangles;
        mesh.normals = normalsArray;

        newTriangles.Clear();
    }
}
