using System;
using TMPro;
using UnityEngine;

public class UI_ShipControlsText : MonoBehaviour
{
    public ShipController m_CurrentZoomController;
    public TMP_Text m_Text;
    // Update is called once per frame
    void Update()
    {
        m_Text.text = String.Format("Dampers: {0}", m_CurrentZoomController.dampersOn ? "On" : "Off");
    }
}
