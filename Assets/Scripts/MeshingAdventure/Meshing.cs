using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Meshing : MonoBehaviour
{
    public float width, heigth;
    Vector3[] newVertices;
    Vector2[] newUV;
    int[] newTriangles;
    Vector3[] normals;
    // Start is called before the first frame update
    void Start()
    {
        Mesh mesh = new Mesh();
        GetComponent<MeshFilter>().mesh = mesh;

        newVertices = new Vector3[4]
        {
            new Vector3(0,0,0),
            new Vector3(width, 0,0),
            new Vector3(0,heigth,0),
            new Vector3(width,heigth,0)
        };
        newTriangles = new int[6]
        {
            0,2,1,
            2,3,1
        };
        normals = new Vector3[4]
        {
            -Vector3.forward,
            -Vector3.forward,
            -Vector3.forward,
            -Vector3.forward,
        };
        newUV = new Vector2[4]
        {
            new Vector2(0,0),
            new Vector2(1,0),
            new Vector2(0,1),
            new Vector2(1,1)
        };
        mesh.vertices = newVertices;
        mesh.uv = newUV;
        mesh.triangles = newTriangles;
        mesh.normals = normals;
    }

    // Update is called once per frame
    void Update()
    {
        
    }
}
