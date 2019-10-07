using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using TMPro;

public class UI_CameraScaleText : MonoBehaviour
{

    
    public CameraZoom m_CurrentZoomController;
    public TMP_Text m_Text;
    // Update is called once per frame
    void Update()
    {
        m_Text.text = String.Format("Scale: {0:0.0}x", m_CurrentZoomController.m_CurrentScale);
    }
}
