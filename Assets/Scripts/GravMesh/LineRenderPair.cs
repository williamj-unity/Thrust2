using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class LineRenderPair : MonoBehaviour
{
    private Transform gn1Transform, gn2Transform;

    private LineRenderer lineRenderer;
    public void SetupLineRenderPair(Transform gn1, Transform gn2, float width)
    {
        gn1Transform = gn1;
        gn2Transform = gn2;
        
        lineRenderer = gameObject.AddComponent<LineRenderer>();
        var mat = Resources.Load<Material>("LineRenderer");
        lineRenderer.material = mat;
        lineRenderer.startWidth = width;
        lineRenderer.endWidth = width;
    }

    // Update is called once per frame
    void Update()
    {
        lineRenderer.SetPositions(new []{gn1Transform.position, gn2Transform.position});
    }
}
