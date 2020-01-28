using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CameraZoom : MonoBehaviour
{
    Vector3 prevPosition;
    float acceleration;
    public float accelerationCoef;
    public float momentumDamping;

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
        prevPosition = transform.position;
        acceleration = 0;
        m_CurrentScale = 1.0f;
        m_CameraPosition = transform.position;
        m_CameraStartPose = m_CameraPosition;
    }

    private void ScrollAction(float scrollDelta)
    {
        acceleration += m_ScrollSpeed * scrollDelta;

        m_CurrentScale = Mathf.Abs(m_CameraPosition.z) / 10.0f;
    }

    void Update()
    {
        if (m_ControllerType == ZoomControllerType.Mouse)
        {
            ScrollAction(Input.mouseScrollDelta.y);
        } // else do diff controller type, maybe you hold the right bumper to zoom out, left zoom scroll in
    }

    // Update is called once per frame
    void FixedUpdate()
    {
        m_CameraPosition.x = transform.position.x;
        m_CameraPosition.y = transform.position.y;

        float timeStep = Time.deltaTime;
        float timeStepSq = timeStep * timeStep;

        float velocity = (transform.position.z - prevPosition.z) * momentumDamping;
        float next = transform.position.z + (velocity) + accelerationCoef * acceleration * timeStepSq;
        prevPosition = transform.position;
        if (m_CameraPosition.z > -1.0f)
        {
            next = -1.0f;
        }
        m_CameraPosition.z = next;
        acceleration = 0;

        transform.position = m_CameraPosition;
    }
    
}
