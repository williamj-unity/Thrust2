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

    public NativeArray<float3> vertices;

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


    public void AddPair(int gn1, int gn2)
    {
        newTriangles.Add(gn1);
        newTriangles.Add(gn1 + 1);
        newTriangles.Add(gn2 + 1);
        newTriangles.Add(gn1);
        newTriangles.Add(gn2 + 1);
        newTriangles.Add(gn2);

        newUV[gn1] = new Vector2(0, 0);
        newUV[gn2] = new Vector2(1, 0);
        newUV[gn1 + 1] = new Vector2(0, 1);
        newUV[gn2 + 1] = new Vector2(1, 1);
    }

    public void UpdateMesh()
    {
        mesh.SetVertices(vertices);
    }

    public void ConstructMesh()
    {
        vertices = new NativeArray<float3>(newVertices.ToArray(), Allocator.Persistent);
        var newTrianglesArray = newTriangles.ToArray();
        var normalsArray = newNormals.ToArray();
        var newUVArray = newUV.ToArray();

        mesh.SetVertices(vertices);
        mesh.uv = newUVArray;
        mesh.triangles = newTrianglesArray;
        mesh.normals = normalsArray;
    }
}
