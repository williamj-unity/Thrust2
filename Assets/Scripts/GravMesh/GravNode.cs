using System.Collections;
using System.Collections.Generic;
using System.Numerics;
using UnityEditor.MemoryProfiler;
using UnityEngine;
using Vector3 = UnityEngine.Vector3;

public class GravNode
{
    public GravNodeCollider gravNodeColliderParent { get; private set; }
    public Transform lineRendererTransform { get; private set; }

    public bool m_Moveable;

    public Vector3 m_Position;
    public Vector3 m_PrevPosition;
    private Vector3 m_Acceleration;

    private Vector3 m_TargetPosition;

    private float rigidity = 1.0f;
    public GravNode(Vector3 position, float spacing)
    {
        gravNodeColliderParent = new GameObject("GravNode").AddComponent<GravNodeCollider>();
        gravNodeColliderParent.SetSpacing(spacing);
        gravNodeColliderParent.transform.position = position;
        gravNodeColliderParent.affectorCollisionEnter += AffectorCollisionEnter;
        gravNodeColliderParent.affectorCollisionExit += AffectorCollisionExit;

        
        lineRendererTransform = new GameObject("LineRendererTransform").transform;
        lineRendererTransform.position = position;
        lineRendererTransform.parent = gravNodeColliderParent.transform;

        m_TargetPosition = position;
        m_Position = position;
        m_PrevPosition = position;
        m_Moveable = true;
    }
    
    void AffectorCollisionEnter(float mass)
    {
        // node no longer moveable (driven by affector)
        Vector3 newPose = gravNodeColliderParent.transform.position;
        newPose.z += mass;
        m_TargetPosition = newPose;
        rigidity = mass;
        ResetAcceleration();
    }

    void AffectorCollisionExit()
    {
        // node is moveable
        m_TargetPosition = gravNodeColliderParent.transform.position;
        rigidity = 1.0f;
        ResetAcceleration();
    }
    
    void AddForce(Vector3 f)
    {
        m_Acceleration += f; //mass is always 1 for these particles.
    }


    void ResetAcceleration()
    {
        m_Acceleration = Vector3.zero;
    }
    
    public void OffsetPos(Vector3 correctionVectorHalf)
    {
        if (m_Moveable)
        {
            m_Position += correctionVectorHalf;
        }
    }

    public void SetPosition(Vector3 position)
    {
        m_Position = position;
        lineRendererTransform.position = position;
        m_PrevPosition = position;
        ResetAcceleration();
    }
    
    public void Update(float damping, float timeStep)
    {
        if(m_Moveable)
        {
            AddForce((m_TargetPosition - lineRendererTransform.transform.position));
            Vector3 temp = m_Position;
            m_Position = m_Position + (m_Position - m_PrevPosition) * (1.0f - damping)  + m_Acceleration*timeStep;;
            m_PrevPosition = temp;
            var lineRenderPos = lineRendererTransform.position;
            lineRenderPos.z = m_Position.z;
            if (lineRenderPos.z <= 0)
            {
                lineRenderPos.z = 0;
            }
            lineRendererTransform.position = lineRenderPos;
            ResetAcceleration();
        }
    }
}


