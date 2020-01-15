using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class MeshingExperiment2 : MonoBehaviour
{
    public float lineWidth = 1;
    public Vector3 pos1 = Vector3.zero;
    public Vector3 pos2 = Vector3.one;

    Mesh mesh;
    Vector3 newPos2;

    public void Start()
    {
        var mf = GetComponent<MeshFilter>();

        mesh = new Mesh();
        mf.mesh = mesh;

        Vector3 perp = Vector3.Cross(pos1 - pos2, -Vector3.forward).normalized;

        var vertices = new Vector3[4]
        {
            pos1 - perp * lineWidth,
            pos1 + perp * lineWidth,
            pos2 - perp * lineWidth,
            pos2 + perp * lineWidth
        };
        mesh.vertices = vertices;

        var tris = new int[6]
        {
            // lower left triangle
            0, 2, 1,
            // upper right triangle
            2, 3, 1
        };
        mesh.triangles = tris;

        var normals = new Vector3[4]
        {
            -Vector3.forward,
            -Vector3.forward,
            -Vector3.forward,
            -Vector3.forward
        };
        mesh.normals = normals;

        var uv = new Vector2[4]
        {
            new Vector2(0, 0),
            new Vector2(1, 0),
            new Vector2(0, 1),
            new Vector2(1, 1)
        };
        mesh.uv = uv;
    }

    float t;
    void Update()
    {
        t += Time.deltaTime;
        pos2.x = Mathf.Sin(t);
        pos2.y = Mathf.Cos(t);
        Vector3 perp = Vector3.Cross(pos2 - pos1, -Vector3.forward * lineWidth);

        var vertices = new Vector3[4]
        {
            pos1 - perp,
            pos1 + perp,
            pos2 - perp,
            pos2 + perp
        };
        mesh.vertices = vertices;

    }
}
