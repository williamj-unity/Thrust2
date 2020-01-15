using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Meshing : MonoBehaviour
{
    public float lineWidth = 0.1f;
    public int width, heigth;
    public float spacing = 0.5f;
    List<Vector3> newVertices;
    List<Vector2> newUV;
    List<int> newTriangles;
    List<Vector3> newNormals;

    int[,] nodeStartIndicies; 
    // Start is called before the first frame update
    void Start()
    {
        nodeStartIndicies = new int[width, heigth];
        Vector3 startPos = transform.position;
         newVertices = new List<Vector3>();
         newTriangles = new List<int>();

        newNormals = new List<Vector3>();
        newUV = new List<Vector2>();

        float startx = startPos.x;
        float starty = startPos.y;
        int index = 0;
        int nodeIndex = 0;
        for(int i = 0; i < width; i++)
        {
            for(int j = 0; j < heigth; j++)
            {
                nodeStartIndicies[i,j] = newVertices.Count;
                float x = startx + spacing * i;
                float y = starty + spacing * j;
                newVertices.Add(new Vector3(x - lineWidth / 2.0f, y - lineWidth / 2.0f, 0));
                newVertices.Add(new Vector3(x + lineWidth / 2.0f, y - lineWidth / 2.0f, 0));
                newVertices.Add(new Vector3(x - lineWidth / 2.0f, y + lineWidth / 2.0f, 0));
                newVertices.Add(new Vector3(x + lineWidth / 2.0f, y + lineWidth / 2.0f, 0));

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

                newUV.Add(new Vector2(0,0));
                newUV.Add(new Vector2(1,0));
                newUV.Add(new Vector2(0,1));
                newUV.Add(new Vector2(1,1));

                index += 4;
                nodeIndex++;
            }
        }

        for (int x = 0; x < width; x++)
        {
            for (int y = 0; y < heigth; y++)
            {
                if (x < width - 1) AddConnectionRight(nodeStartIndicies[x, y], nodeStartIndicies[x + 1, y]);
                if (y < heigth - 1) AddConnectionUp(nodeStartIndicies[x, y], nodeStartIndicies[x, y + 1]);
            }
        }

        Mesh mesh = new Mesh();
        GetComponent<MeshFilter>().mesh = mesh;

        var newVerticesArray = newVertices.ToArray();
        var newTrianglesArray = newTriangles.ToArray();
        var normalsArray = newNormals.ToArray();
        var newUVArray = newUV.ToArray();

        mesh.vertices = newVerticesArray;
        mesh.uv = newUVArray;
        mesh.triangles = newTrianglesArray;
        mesh.normals = normalsArray;
    }

    void AddConnectionRight(int node1, int node2)
    {
        newTriangles.Add(node1 + 1);
        newTriangles.Add(node1 + 3);
        newTriangles.Add(node2);
        newTriangles.Add(node1 + 3);
        newTriangles.Add(node2 + 2);
        newTriangles.Add(node2);
    }

    void AddConnectionUp(int node1, int node2)
    {
        newTriangles.Add(node1 + 3);
        newTriangles.Add(node1 + 2);
        newTriangles.Add(node2);
        newTriangles.Add(node1 + 3);
        newTriangles.Add(node2);
        newTriangles.Add(node2 + 1);
    }
    // Update is called once per frame
    void Update()
    {
        
    }
}
