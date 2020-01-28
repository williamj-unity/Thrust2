using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GravMeshFader : MonoBehaviour
{
    public CameraZoom cameraZoomer;
    public float min;
    public float max;
    GravGridBuilder grid;
    // Start is called before the first frame update
    void Start()
    {
        if (min > max)
            min = max - 1.0f;

        grid = GetComponent<GravGridBuilder>();
        grid.SetRange(min, max);
    }

    // Update is called once per frame
    void Update()
    {
        grid.SetCurrentZoom(cameraZoomer.m_CurrentScale);
    }
}
