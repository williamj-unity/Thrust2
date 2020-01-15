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


    public int AddPair(float3 position1, float3 position2)
    {
        float3 perp = Vector3.Cross(position2 - position1, -Vector3.forward * lineWidth);

        //gn1 node vertices
        newVertices.Add(position1 - perp);
        newVertices.Add(position1 + perp);

        //gn2 node vertices
        newVertices.Add(position2 - perp);
        newVertices.Add(position2 + perp);

        newTriangles.Add(index);
        newTriangles.Add(index + 2);
        newTriangles.Add(index + 1);
        newTriangles.Add(index + 2);
        newTriangles.Add(index + 3);
        newTriangles.Add(index + 1);

        newNormals.Add(-Vector3.forward);
        newNormals.Add(-Vector3.forward);
        newNormals.Add(-Vector3.forward);
        newNormals.Add(-Vector3.forward);

        newUV.Add(new Vector2(0, 0));
        newUV.Add(new Vector2(1, 0));
        newUV.Add(new Vector2(0, 1));
        newUV.Add(new Vector2(1, 1));
        int ret = index;
        index += 4;
        return ret;
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
