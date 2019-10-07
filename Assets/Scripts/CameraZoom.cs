using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraZoom : MonoBehaviour
{

    // TODO:: this will eventually need to be refactored into an input manager that assigns actions based 
    // on different controller types that are connected. I've added it here now to how this should be done
    // ,albeit in a more generic way, in the future. 
    public enum ZoomControllerType
    {
        Mouse,
        Controller
    }

    public ZoomControllerType m_ControllerType = ZoomControllerType.Mouse;

    // "currentScale" not actually sure what this means but I'm going to use it as a debug value in the UI
    public float m_CurrentScale { get; private set; }
    
    
    public float m_ScrollSpeed = 1.0f;


    private Vector3 m_CameraPosition;
    private Vector3 m_CameraStartPose;

    private void Start()
    {
        m_CurrentScale = 1.0f;
        m_CameraPosition = transform.position;
        m_CameraStartPose = m_CameraPosition;
    }

    private void ScrollAction(float scrollDelta)
    {
        //m_CurrentScale += m_ScrollSpeed * scrollDelta;
        m_CameraPosition.z += m_ScrollSpeed * scrollDelta;
        if (m_CameraPosition.z > -1.0f)
        {
            m_CameraPosition.z = -1.0f;
        }

        m_CurrentScale = Mathf.Abs(m_CameraPosition.z) / 10.0f;
    }

    // Update is called once per frame
    void Update()
    {
        if (m_ControllerType == ZoomControllerType.Mouse)
        {
            ScrollAction(Input.mouseScrollDelta.y);
        } // else do diff controller type, maybe you hold the right bumper to zoom out, left zoom scroll in

        transform.position = m_CameraPosition;
    }
    
}
