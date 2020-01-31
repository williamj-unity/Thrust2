using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class GravMeshLineWidthScalar : MonoBehaviour
{
    public CameraZoom cameraZoomer;
    public float cameraZoomMin;
    public float cameraZoomMax;

    public float lineWidthMinScalar = 1.0f;
    public float lineWidthMaxScalar = 2.0f;
    GravGridBuilder grid;
    public float currentScalar;
    // Start is called before the first frame update
    void Start()
    {
        grid = GetComponent<GravGridBuilder>();
        if (cameraZoomMin > cameraZoomMax)
            cameraZoomMin = cameraZoomMax - 1.0f;

        if (lineWidthMinScalar > lineWidthMaxScalar)
            lineWidthMinScalar = lineWidthMaxScalar - 1.0f;
    }

    // Update is called once per frame
    void Update()
    {
        float factor = (cameraZoomer.m_CurrentScale - cameraZoomMin) / (cameraZoomMax - cameraZoomMin);
        float newScalar = Mathf.Clamp(factor * (lineWidthMaxScalar - lineWidthMinScalar) + lineWidthMinScalar, lineWidthMinScalar, lineWidthMaxScalar);
        currentScalar = newScalar;
        grid.SetLineWidthScalar(newScalar);
    }
}
